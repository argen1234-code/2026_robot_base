/**
 * @file    bsp_GPS.c
 * @brief   GPS Driver - NMEA parsing and binary AGRIC frame processing.
 *
 * This driver handles reception and decoding of GPS/GNSS data from a
 * dual-antenna RTK module (e.g., NovAtel or compatible) over UART2.
 *
 * Supported message types:
 *   - $GNGGA  : Global Positioning System Fix Data (NMEA-0183)
 *   - $GNTHS  : True Heading and Status (NMEA-0183)
 *   - $GNHPR  : Heading, Pitch, and Roll (proprietary NMEA extension)
 *   - AGRIC   : Binary frame with full position/attitude solution
 *
 * The driver maintains four global data structures (one per message type)
 * accessible via Get*() accessor functions.  Each structure carries a
 * last_update_tick field timestamped with HAL_GetTick() on successful
 * decode, allowing consumers to detect stale data.
 */

#include "bsp_GPS.h"
#include "usart.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "cmsis_os2.h"

/* ---- Global data instances (one per supported message type) ---- */

static T_GNGGA tgngga;
static T_GNTHS tgnths;
static T_AGRIC tagric;
static T_GPHPR tgphpr;

/* Keil Watch-only GPS mirror. No control path reads this object. */
volatile GPS_Debug_t g_gps_debug;

static double GPS_DebugNmeaToDegree(double nmea_coordinate, char direction)
{
	double degrees = (double)((uint32_t)(nmea_coordinate / 100.0));
	double decimal = degrees + (nmea_coordinate - degrees * 100.0) / 60.0;

	if (direction == 'S' || direction == 'W')
	{
		decimal = -decimal;
	}

	return decimal;
}

static void GPS_DebugCaptureRx(uint16_t size)
{
	g_gps_debug.update_sequence++;
	g_gps_debug.rx_event_count++;
	g_gps_debug.rx_byte_count += size;
	g_gps_debug.last_rx_event_tick = HAL_GetTick();
	g_gps_debug.last_rx_size = size;
	g_gps_debug.update_sequence++;
}

static void GPS_DebugUpdateGGA(void)
{
	uint8_t latitude_direction_valid = (tgngga.lat_dir == 'N' || tgngga.lat_dir == 'S') ? 1U : 0U;
	uint8_t longitude_direction_valid = (tgngga.lon_dir == 'E' || tgngga.lon_dir == 'W') ? 1U : 0U;
	uint8_t valid = (tgngga.qf >= 1U && latitude_direction_valid && longitude_direction_valid) ? 1U : 0U;

	g_gps_debug.update_sequence++;
	g_gps_debug.gga_update_count++;
	g_gps_debug.last_message_type = GPS_DEBUG_MSG_GGA;
	g_gps_debug.last_decoded_tick = tgngga.last_update_tick;
	g_gps_debug.position_valid = valid;
	g_gps_debug.utc_time = tgngga.utc_time;
	g_gps_debug.latitude_deg = GPS_DebugNmeaToDegree(tgngga.lat, tgngga.lat_dir);
	g_gps_debug.longitude_deg = GPS_DebugNmeaToDegree(tgngga.lon, tgngga.lon_dir);
	g_gps_debug.altitude_m = tgngga.Alt;
	g_gps_debug.hdop = tgngga.Hdop;
	g_gps_debug.differential_age_s = tgngga.Age;
	g_gps_debug.fix_quality = tgngga.qf;
	g_gps_debug.satellites = tgngga.sats;
	g_gps_debug.latitude_direction = tgngga.lat_dir;
	g_gps_debug.longitude_direction = tgngga.lon_dir;
	g_gps_debug.gga_last_update_tick = tgngga.last_update_tick;
	g_gps_debug.gga = tgngga;
	g_gps_debug.update_sequence++;
}

static void GPS_DebugUpdateTHS(char mode)
{
	g_gps_debug.update_sequence++;
	g_gps_debug.ths_update_count++;
	g_gps_debug.last_message_type = GPS_DEBUG_MSG_THS;
	g_gps_debug.last_decoded_tick = tgnths.last_update_tick;
	g_gps_debug.ths_last_update_tick = tgnths.last_update_tick;
	g_gps_debug.ths = tgnths;
	g_gps_debug.ths.Mode = mode;
	g_gps_debug.update_sequence++;
}

static void GPS_DebugUpdateHPR(void)
{
	g_gps_debug.update_sequence++;
	g_gps_debug.hpr_update_count++;
	g_gps_debug.last_message_type = GPS_DEBUG_MSG_HPR;
	g_gps_debug.last_decoded_tick = tgphpr.last_update_tick;
	g_gps_debug.hpr_last_update_tick = tgphpr.last_update_tick;
	g_gps_debug.hpr = tgphpr;
	g_gps_debug.update_sequence++;
}

static void GPS_DebugUpdateAGRIC(void)
{
	g_gps_debug.update_sequence++;
	g_gps_debug.agric_update_count++;
	g_gps_debug.last_message_type = GPS_DEBUG_MSG_AGRIC;
	g_gps_debug.last_decoded_tick = tagric.last_update_tick;
	g_gps_debug.agric_last_update_tick = tagric.last_update_tick;
	g_gps_debug.agric = tagric;
	g_gps_debug.update_sequence++;
}

/* ---- Accessor functions ---- */

/**
 * @brief  Return pointer to the global GNGGA data structure.
 * @return Pointer to the latest parsed $GNGGA sentence data.
 */
PT_GNGGA GetGNGGA(void) { return &tgngga; }

/**
 * @brief  Return pointer to the global GNTHS data structure.
 * @return Pointer to the latest parsed $GNTHS sentence data.
 */
PT_GNTHS GetGNTHS(void) { return &tgnths; }

/**
 * @brief  Return pointer to the global AGRIC data structure.
 * @return Pointer to the latest received AGRIC binary frame data.
 */
PT_AGRIC GetAGRIC(void) { return &tagric; }

/**
 * @brief  Return pointer to the global GPHPR data structure.
 * @return Pointer to the latest parsed $GNHPR sentence data.
 */
PT_GPHPR GetGPHPR(void) { return &tgphpr; }

/**
 * @brief  Copy raw AGRIC binary frame into the global structure.
 *
 * Performs a struct copy of the incoming AGRIC data and records the
 * reception timestamp via HAL_GetTick().
 *
 * @param ptAGRIC  Pointer to a raw AGRIC frame (cast from ucAGRIC buffer).
 */
void AGRIC_Analy(PT_AGRIC ptAGRIC)
{
	tagric = *ptAGRIC;
	tagric.last_update_tick = HAL_GetTick();
	GPS_DebugUpdateAGRIC();
}

/**
 * @brief  Transmit an ASCII command string to the GPS module over UART2.
 *
 * @param p_cmd  Null-terminated command string (e.g., "gpgga 1\r\n").
 *               The caller is responsible for including any required
 *               line terminator (\r\n).
 */
void GPS_SendCmd(const char* p_cmd)
{
	HAL_UART_Transmit(&huart2, (uint8_t*)p_cmd, strlen(p_cmd), 200);
}

/**
 * @brief  Initialize the GPS module via a multi-step configuration sequence.
 *
 * Sends NovAtel/OEM-style ASCII commands over UART2 to configure the
 * receiver for dual-antenna RTK operation.  The initialization uses a
 * state machine that advances one step per loop iteration, with a 500 ms
 * delay between steps to allow the receiver time to process each command.
 *
 * Step sequence:
 *   1  - Idle / synchronization start
 *   2  - Unlog all previous output logs
 *   3  - Set base station update interval to 60 s
 *   4  - Set rover mode to automotive
 *   5  - Enable $GPGGA output at 1 Hz
 *   6  - Enable $GPTHS output at 1 Hz
 *   7  - Enable AGRICB binary output at 1 Hz
 *   8  - Enable $GPHPR output at 1 Hz
 *   9  - Save configuration to non-volatile memory
 *  10  - Re-apply rover automotive mode (safety)
 *  11  - Done; exit state machine
 *
 * @note  osDelay() calls require that this function be called from an
 *        RTOS task context.
 */
void GPS_Init(void)
{
	uint8_t ucConfigStep = 1;
	while (ucConfigStep)
	{
		osDelay(500);                        /* Wait 500 ms between commands */
		GPS_InitBackgroundHook();             /* Allow lightweight startup services while waiting. */
		switch(ucConfigStep++)
		{
			case 1:break;                    /* Step 1: idle / sync start */
			case 2:GPS_SendCmd("unlog\r\n");break;    /* Step 2: stop all logs */
			case 3:GPS_SendCmd("mode base time 60\r\n");break; /* Step 3: base interval */
			case 4:GPS_SendCmd("mode rover automotive\r\n");break; /* Step 4: rover mode */
			case 5:GPS_SendCmd("gpgga 1\r\n");break;     /* Step 5: enable GGA at 1Hz */
			case 6:GPS_SendCmd("gpths 1\r\n");break;     /* Step 6: enable THS at 1Hz */
			case 7:GPS_SendCmd("agricb 1\r\n");break;    /* Step 7: enable AGRIC binary */
			case 8:GPS_SendCmd("gphpr 1\r\n");break;     /* Step 8: enable HPR at 1Hz */
			case 9:GPS_SendCmd("saveconfig\r\n");break;  /* Step 9: save to NVM */
			case 10:GPS_SendCmd("mode rover automotive\r\n");break; /* Step 10: re-apply rover */
			case 11:ucConfigStep = 0;break;   /* Step 11: exit state machine */
		}
	}
	osDelay(500);                            /* Final settling delay */
	GPS_InitBackgroundHook();
}

/**
 * @brief  Parse a $GNGGA NMEA sentence into the global tgngga structure.
 *
 * $GNGGA is the standard NMEA-0183 Global Positioning System Fix Data
 * sentence.  This parser splits the comma-separated fields in-place by
 * replacing delimiters with null terminators, then converts each field
 * to its typed representation.
 *
 * Fields parsed (order per NMEA GGA specification):
 *  [0] UTC time (hhmmss.ss)
 *  [1] Latitude  (ddmm.mmmmm)
 *  [2] Latitude direction (N/S)
 *  [3] Longitude (dddmm.mmmmm)
 *  [4] Longitude direction (E/W)
 *  [5] GPS quality indicator (0-8)
 *  [6] Number of satellites in use
 *  [7] Horizontal dilution of precision (HDOP)
 *  [8] Antenna altitude above mean sea level (meters)
 *  [9] Altitude units (always 'M')
 * [10] Geoidal separation / undulation (meters)
 * [11] Geoidal separation units (always 'M')
 * [12] Age of differential GPS data (seconds)
 * [13] Differential reference station ID
 * [14] Checksum (hex string after '*')
 *
 * @param str  Pointer into the NMEA sentence, starting at the data field
 *             (immediately after the leading "$GNGGA," prefix).
 */
static void GNGGA_Decode(char* str)
{
	char *data[16];
	unsigned char i;
	/* Split the comma-separated fields in-place */
	for(i = 0; i <14; i++)
	{
		data[i] = str;
		while(*str != (i == 13? '*':','))   /* Field 13 terminates at '*' not ',' */
		{
			if(*str++ == '\0') return;       /* Abort if string ends prematurely */
		}
		*str ++ = '\0';                      /* Replace delimiter with null */
	}
	data[i] = str;                           /* CRC field after the '*' */
	/* Populate the global GNGGA structure with parsed values */
	tgngga.utc_time = atof(data[0]);
	tgngga.lat = atof(data[1]);
	tgngga.lat_dir = *data[2];
	tgngga.lon = atof(data[3]);
	tgngga.lon_dir = *data[4];
	tgngga.qf = *data[5] - '0';              /* Convert ASCII digit to integer */
	tgngga.sats = atoi(data[6]);
	tgngga.Hdop = atof(data[7]);
	tgngga.Alt = atof(data[8]);
	strcpy(tgngga.a_units, data[9]);
	tgngga.undulation = atof(data[10]);
	strcpy(tgngga.u_units, data[11]);
	tgngga.Age = atof(data[12]);
	strcpy(tgngga.stn_ID, data[13]);
	strcpy(tgngga.crc, data[14]);
	tgngga.last_update_tick = HAL_GetTick(); /* Timestamp for staleness detection */
	GPS_DebugUpdateGGA();
}

/**
 * @brief  Parse a $GNTHS NMEA sentence into the global tgnths structure.
 *
 * $GNTHS is the NMEA-0183 True Heading and Status sentence.  It provides
 * the heading angle computed by the dual-antenna system along with a
 * mode indicator that describes the quality of the heading solution.
 *
 * Fields:
 *  [0] Heading (degrees true, 0.0-359.9)
 *  [1] Mode indicator (A=autonomous, E=estimated, etc.)
 *  [2] Checksum (hex string after '*')
 *
 * @param str  Pointer into the NMEA sentence, starting at the data field
 *             (immediately after the leading "$GNTHS," prefix).
 */
static void GNTHS_Decode(char *str)
{
	char *data[8];
	unsigned char i;
	/* Split the comma-separated fields (2 data fields + CRC) */
	for(i = 0; i < 2; i++)
	{
		data[i] = str;
		while(*str != (i == 1? '*':','))   /* Field 1 terminates at '*' not ',' */
		{
			if(*str++ == '\0') return;
		}
		*str ++ = '\0';                      /* Replace delimiter with null */
	}
	data[i] = str;
	tgnths.Heading = atof(data[0]);
	tgnths.Mode = atof(data[1]);
	strcpy(tgnths.crc, data[2]);
	tgnths.last_update_tick = HAL_GetTick();
	GPS_DebugUpdateTHS(*data[1]);
}

/**
 * @brief  Parse a $GNHPR NMEA sentence into the global tgphpr structure.
 *
 * $GNHPR is a proprietary or extended NMEA sentence that provides
 * heading, pitch, and roll angles from the dual-antenna GNSS/INS system.
 *
 * Fields:
 *  [0] UTC time (hhmmss.ss)
 *  [1] Heading (degrees)
 *  [2] Pitch (degrees, positive = nose up)
 *  [3] Roll  (degrees, positive = starboard down)
 *  [4] Quality flag
 *  [5] Number of satellites used
 *  [6] Age of differential corrections (seconds)
 *  [7] Differential reference station ID
 *  [8] Checksum (hex string after '*')
 *
 * @param str  Pointer into the NMEA sentence, starting at the data field
 *             (immediately after the leading "$GNHPR," prefix).
 */
static void GNHPR_Decode(char* str)
{
	char *data[16];
	unsigned char i;
	/* Split the comma-separated fields (8 data fields + CRC) */
	for(i = 0; i < 8; i++)
	{
		data[i] = str;
		while(*str != (i == 7? '*':','))   /* Field 7 terminates at '*' not ',' */
		{
			if(*str++ == '\0') return;
		}
		*str ++ = '\0';                      /* Replace delimiter with null */
	}
	data[i] = str;                           /* CRC field after the '*' */
	tgphpr.utc_time = atof(data[0]);
	tgphpr.Heading = atof(data[1]);
	tgphpr.gphpr_Pitch = atof(data[2]);
	tgphpr.gphpr_Roll = atof(data[3]);
	tgphpr.qf = *data[4] - '0';
	tgphpr.sats = atoi(data[5]);
	tgphpr.Age = atof(data[6]);
	strcpy(tgphpr.stn_ID, data[7]);
	strcpy(tgphpr.crc, data[8]);
	tgphpr.last_update_tick = HAL_GetTick();
	GPS_DebugUpdateHPR();
}

/**
 * @brief  Dispatch a complete NMEA sentence to the appropriate decoder.
 *
 * Examines the NMEA talker+sentence-ID field at positions [3..5] of the
 * message (e.g., for "$GNGGA" the characters are 'G','G','A') to route
 * to the correct parser.  The leading "$--" prefix (talker ID) is
 * ignored; only the 3-character sentence type is matched.
 *
 * Recognized sentence types and their parsers:
 *   - ...GGA  (position fix data)      -> GNGGA_Decode()
 *   - ...THS  (true heading)           -> GNTHS_Decode()
 *   - ...HPR  (heading/pitch/roll)     -> GNHPR_Decode()
 *
 * @param msg  Null-terminated NMEA sentence string starting with '$'.
 *             If msg[0] is not '$', the function returns immediately
 *             without processing.
 */
static void Data_Decode(char* msg)
{
	if(msg[0] != '$') return;              /* Not an NMEA sentence; ignore */

	if(msg[3] == 'G' && msg[4] == 'G' && msg[5] == 'A')
		GNGGA_Decode(msg + 7);             /* $--GGA -> position fix data */
	if(msg[3] == 'T' && msg[4] == 'H' && msg[5] == 'S')
		GNTHS_Decode(msg + 7);             /* $--THS -> true heading data */
	if (msg[3] == 'H' && msg[4] == 'P' && msg[5] == 'R')
		GNHPR_Decode(msg + 7);             /* $--HPR -> heading/pitch/roll */
}

/**
 * @brief  Static buffer for assembling a raw AGRIC binary frame.
 *
 * Holds up to 512 bytes of a received AGRIC binary frame payload.
 * The buffer is filled starting from the sync-header match position
 * in GPS_RxPro_HAL() and is reused on each successful detection.
 * Content is valid only until the next sync-header match overwrites it.
 */
unsigned char ucAGRIC[512] = {0};

/**
 * @brief  Process raw byte stream received from the GPS UART.
 *
 * This is the main receive handler, called from the UART interrupt or
 * DMA-completion callback.  It performs two independent parsing passes
 * over the incoming data buffer:
 *
 *   Pass 1 - Binary AGRIC frame detection:
 *     Scans for the 3-byte sync header (0xAA 0x44 0xB5).  On match,
 *     copies up to 512 bytes into ucAGRIC[] and calls AGRIC_Analy() to
 *     interpret the binary frame.  After the first match, the scan is
 *     stopped (break) -- only one binary frame is extracted per call.
 *
 *   Pass 2 - NMEA sentence extraction:
 *     Scans for '$' start markers and \r\n line terminators.  Each
 *     complete NMEA sentence (from '$' through the CR+LF pair) is
 *     null-terminated and dispatched via Data_Decode().
 *
 * @param pBuf  Pointer to the raw byte buffer received from the UART.
 * @param Size  Number of valid bytes in the buffer.
 */
void GPS_RxPro_HAL(uint8_t* pBuf, uint16_t Size)
{
	if(Size == 0) return;
	GPS_DebugCaptureRx(Size);

	uint16_t i = 0;
	char *pHead = NULL;

	/* ---- Pass 1: Search for AGRIC binary sync header (0xAA 0x44 0xB5) ---- */
	while(i < Size - 2)
	{
		if(pBuf[i] == 0xAA && pBuf[i + 1] == 0x44 && pBuf[i + 2] == 0xB5)
		{
			/* Sync header found; copy frame into buffer (clamp to 512 bytes) */
			memcpy(ucAGRIC, &pBuf[i], (Size - i > 512) ? 512 : (Size - i));
			AGRIC_Analy((PT_AGRIC)ucAGRIC);  /* Parse the binary frame */
			break;
		}
	        i++;
	}

	/* ---- Pass 2: Extract NMEA text sentences (delimited by $ ... \r\n) ---- */
	i = 0;
	while(i < Size)
	{
		if(pBuf[i] == '$')
			pHead = (char*)pBuf + i;         /* Mark start of an NMEA sentence */

		if(i > 0 && pBuf[i-1] == '\r' && pBuf[i] == '\n' && pHead)
		{
			/* CR+LF found; null-terminate the sentence and dispatch */
			pBuf[i-1] = '\0';
			pBuf[i]   = '\0';
			Data_Decode(pHead);
			pHead = NULL;                    /* Reset for next sentence */
		}
	        i++;
	}
}
__weak void GPS_InitBackgroundHook(void)
{
}
