/**
 * @file    bsp_GPS.h
 * @brief   GPS Driver - Type definitions and public API.
 *
 * Defines the data structures for parsed NMEA sentences and binary AGRIC
 * frames from the dual-antenna RTK GPS module, plus the public function
 * prototypes.
 *
 * Structure overview:
 *   - T_GNGGA  : Parsed $GNGGA sentence (position fix data)
 *   - T_GNTHS  : Parsed $GNTHS sentence (true heading)
 *   - T_BinHead: Binary message header (sync + metadata)
 *   - T_AGRIC  : Binary AGRIC frame (full position + attitude solution)
 *   - T_GPHPR  : Parsed $GNHPR sentence (heading, pitch, roll)
 */

#ifndef __BSP__GPS_H
#define __BSP__GPS_H

#include "main.h"

/**
 * @brief  Parsed $GNGGA NMEA sentence (Global Positioning System Fix Data).
 *
 * Standard NMEA-0183 GGA message containing position, fix quality,
 * satellite count, HDOP, altitude, and differential reference station
 * information.
 */
typedef struct{
	double utc_time;            /**< UTC time of fix (hhmmss.ss) */
	double lat;                 /**< Latitude in ddmm.mmmmm format */
	char lat_dir;               /**< Latitude direction: 'N' or 'S' */
	double lon;                 /**< Longitude in dddmm.mmmmm format */
	char lon_dir;               /**< Longitude direction: 'E' or 'W' */
	unsigned char qf;           /**< GPS quality indicator (0=invalid, 1=GPS, 2=DGPS, 4=RTK fix, 5=RTK float) */
	unsigned char sats;         /**< Number of satellites in use */
	float Hdop;                 /**< Horizontal dilution of precision */
	float Alt;                  /**< Antenna altitude above mean sea level (meters) */
	char a_units[8];            /**< Altitude units (always "M") */
	float undulation;           /**< Geoidal separation / undulation (meters) */
	char u_units[8];            /**< Undulation units (always "M") */
	float Age;                  /**< Age of differential corrections (seconds) */
	char stn_ID[8];             /**< Differential reference station ID */
	char crc[8];                /**< NMEA checksum (hex string after '*') */
	uint32_t last_update_tick;  /**< HAL tick when this struct was last updated */
}T_GNGGA, *PT_GNGGA;

/**
 * @brief  Parsed $GNTHS NMEA sentence (True Heading and Status).
 *
 * Provides the true heading angle computed from the dual-antenna
 * baseline, along with a mode indicator describing solution quality.
 */
typedef struct{
	float Heading;              /**< True heading angle (degrees, 0.0-359.9) */
	char Mode;                  /**< Heading mode indicator (A=autonomous, E=estimated, M=manual, etc.) */
	char crc[8];                /**< NMEA checksum (hex string after '*') */
	uint32_t last_update_tick;  /**< HAL tick when this struct was last updated */
}T_GNTHS, *PT_GNTHS;

/**
 * @brief  Binary message header (NovAtel/OEM-style format).
 *
 * Standard 28-byte binary header prepended to every binary output log
 * from the GNSS receiver.  The frame starts with a 3-byte sync sequence
 * (0xAA, 0x44, 0xB5) which is used to locate frame boundaries in the
 * byte stream.
 */
typedef struct{
	unsigned char Sync1;        /**< Sync byte 1 (always 0xAA) */
	unsigned char Sync2;        /**< Sync byte 2 (always 0x44) */
	unsigned char Sync3;        /**< Sync byte 3 (always 0xB5) */
	unsigned char CPUIdle;      /**< CPU idle time indicator */
	unsigned short MessageID;   /**< Message / log type identifier */
	unsigned short MessageLength; /**< Length of the message body (bytes) */
	unsigned char TimeRef;      /**< Time reference status */
	unsigned char TimeStat;     /**< Time status flag */
	unsigned short Wn;          /**< GPS week number */
	unsigned long Ms;           /**< GPS time of week (milliseconds) */
	unsigned long Version;      /**< Firmware / format version number */
	unsigned char Reserved;     /**< Reserved byte (padding) */
	unsigned char LeapSec;      /**< Leap seconds offset (GPS - UTC) */
	unsigned short DelayMs;     /**< Receiver processing delay (milliseconds) */
}T_BinHead, *PT_BinHead;

/**
 * @brief  Parsed AGRIC binary frame (full position + attitude solution).
 *
 * This is the richest data source from the dual-antenna RTK receiver,
 * providing complete navigation and attitude information in a single
 * binary frame:
 *
 *   - Position: lat/lon/alt (WGS-84) and ECEF coordinates
 *   - Attitude: heading, pitch, roll angles
 *   - Velocity: ENU (East/North/Up) components and ground speed
 *   - Baseline: 3D vector between primary and secondary antennas
 *   - Quality:  standard deviations (sigma) for position, velocity,
 *               baseline, and ECEF
 *   - Status:  position type, heading status, satellite counts per
 *              constellation (GPS, BeiDou, GLONASS, Galileo)
 *   - Base station coordinates and secondary antenna position
 *   - Time:  UTC date/time and GPS week-seconds
 *
 * The binary frame payload immediately follows the header (T_BinHead),
 * which is embedded as the first field (tASCIIHead) for direct overlay
 * of the sync sequence.
 *
 * @note  Field names prefixed with "Xigema" represent standard deviation
 *        (sigma) values of the corresponding measurement.
 */
typedef struct{
	T_BinHead tASCIIHead;       /**< Binary frame header (sync + metadata) */
	char GNSS[4];               /**< GNSS system identifier string */
	unsigned char length;       /**< Payload length (bytes) */
	unsigned char Year;         /**< UTC year (e.g., 24 for 2024) */
	unsigned char Month;        /**< UTC month (1-12) */
	unsigned char Day;          /**< UTC day (1-31) */
	unsigned char Hour;         /**< UTC hour (0-23) */
	unsigned char Minute;       /**< UTC minute (0-59) */
	unsigned char Second;       /**< UTC second (0-59) */
	unsigned char Postype;      /**< Position type (0=none, 1=single, 2=DGPS, 4=RTK fix, 5=RTK float) */
	unsigned char HeadingStat;  /**< Heading solution status flag */
	unsigned char NumGPSSta;    /**< Number of GPS satellites used in solution */
	unsigned char NumBDSSta;    /**< Number of BeiDou satellites used in solution */
	unsigned char NumGLOSta;    /**< Number of GLONASS satellites used in solution */
	float Baseline_N;           /**< Baseline vector - North component (meters) */
	float Baseline_E;           /**< Baseline vector - East component (meters) */
	float Baseline_U;           /**< Baseline vector - Up component (meters) */
	float Baseline_NStd;        /**< Baseline North standard deviation (meters) */
	float Baseline_EStd;        /**< Baseline East standard deviation (meters) */
	float Baseline_UStd;        /**< Baseline Up standard deviation (meters) */
	float Heading;              /**< Heading angle from dual antennas (degrees) */
	float agric_Pitch;          /**< Pitch angle (degrees, positive = nose up) */
	float agric_Roll;           /**< Roll angle (degrees, positive = starboard down) */
	float Speed;                /**< Ground speed (m/s) */
	float VelocityOfNorth;      /**< Velocity - North component (m/s) */
	float VelocityOfEast;       /**< Velocity - East component (m/s) */
	float VelocityOfUp;         /**< Velocity - Up component (m/s) */
	float XigemaVx;             /**< Velocity North standard deviation (m/s) */
	float XigemaVy;             /**< Velocity East standard deviation (m/s) */
	float XigemaVz;             /**< Velocity Up standard deviation (m/s) */
	double lat;                 /**< Latitude (degrees, WGS-84 ellipsoid) */
	double lon;                 /**< Longitude (degrees, WGS-84 ellipsoid) */
	double alt;                 /**< Ellipsoidal altitude (meters, WGS-84) */
	double ECEFX;               /**< ECEF X coordinate (meters) */
	double ECEFY;               /**< ECEF Y coordinate (meters) */
	double ECEFZ;               /**< ECEF Z coordinate (meters) */
	float XigemaLat;            /**< Latitude standard deviation (meters) */
	float XigemaLon;            /**< Longitude standard deviation (meters) */
	float XigemaAlt;            /**< Altitude standard deviation (meters) */
	float XigemaECEFX;          /**< ECEF X standard deviation (meters) */
	float XigemaECEFY;          /**< ECEF Y standard deviation (meters) */
	float XigemaECEFZ;          /**< ECEF Z standard deviation (meters) */
	double BaseLat;             /**< Base station latitude (degrees, WGS-84) */
	double BaseLon;             /**< Base station longitude (degrees, WGS-84) */
	double BaseAlt;             /**< Base station altitude (meters, WGS-84) */
	double SecLat;              /**< Secondary antenna latitude (degrees, WGS-84) */
	double SecLon;              /**< Secondary antenna longitude (degrees, WGS-84) */
	double SecAlt;              /**< Secondary antenna altitude (meters, WGS-84) */
	int GPSWeekSecond;          /**< GPS seconds of week (0 to 604799) */
	float Diffage;              /**< Differential correction age (seconds) */
	float SpeedHeading;         /**< Speed-based heading (degrees) */
	float Undulation;           /**< Geoidal undulation at position (meters) */
	float RemainFloat3;         /**< Reserved float field (padding) */
	float RemainFloat4;         /**< Reserved float field (padding) */
	unsigned char NumGalSta;    /**< Number of Galileo satellites used in solution */
	unsigned char SpeedType;    /**< Speed computation type indicator */
	unsigned char RemainChar3;  /**< Reserved byte (padding) */
	unsigned char RemainChar4;  /**< Reserved byte (padding) */
	unsigned int crc32;         /**< CRC-32 checksum of the binary frame */
	uint32_t last_update_tick;  /**< HAL tick when this struct was last updated */
}T_AGRIC, *PT_AGRIC;

/**
 * @brief  Parsed $GNHPR NMEA sentence (Heading, Pitch, Roll).
 *
 * Proprietary or extended NMEA sentence providing attitude angles from
 * the dual-antenna GNSS/INS system.  Supplements the binary AGRIC frame
 * with an NMEA-accessible subset of the attitude data.
 */
typedef struct{
	double utc_time;            /**< UTC time of measurement (hhmmss.ss) */
	float Heading;              /**< Heading angle (degrees) */
	float gphpr_Pitch;          /**< Pitch angle (degrees, positive = nose up) */
	float gphpr_Roll;           /**< Roll angle (degrees, positive = starboard down) */
	unsigned char qf;           /**< Quality flag */
	unsigned char sats;         /**< Number of satellites used */
	float Age;                  /**< Age of differential corrections (seconds) */
	char stn_ID[8];             /**< Differential reference station ID */
	char crc[8];                /**< NMEA checksum (hex string after '*') */
	uint32_t last_update_tick;  /**< HAL tick when this struct was last updated */
}T_GPHPR, *PT_GPHPR;

/* Message identifier stored in GPS_Debug_t.last_message_type. */
#define GPS_DEBUG_MSG_NONE   0U
#define GPS_DEBUG_MSG_GGA    1U
#define GPS_DEBUG_MSG_THS    2U
#define GPS_DEBUG_MSG_HPR    3U
#define GPS_DEBUG_MSG_AGRIC  4U

/* Navigation source stored in GPS_NavigationDebug_t.navigation_mode. */
#define GPS_DEBUG_NAV_MODE_NONE    0U
#define GPS_DEBUG_NAV_MODE_PURE    1U
#define GPS_DEBUG_NAV_MODE_FUSION  2U

/**
 * @brief Navigation target/error snapshot shown as g_gps_debug.navigation.
 *
 * This subgroup is refreshed only by app_Navigation.c while GPS navigation is
 * running or stopping. It mirrors controller state and final Vx/Vy/Wz targets;
 * the control path never reads values back from this object.
 */
typedef struct GPS_NavigationDebug_s
{
    uint32_t update_sequence;
    uint32_t last_update_tick;
    uint32_t dwell_start_tick;

    uint8_t is_navigating;
    uint8_t loop_enable;
    uint8_t phase;                  /* 0 idle, 1 running, 2 dwelling. */
    uint8_t rtk_quality;
    uint8_t current_waypoint_index; /* Zero-based route index. */
    uint8_t total_waypoints;
    uint8_t navigation_mode;        /* GPS_DEBUG_NAV_MODE_* */
    uint8_t reserved;

    double current_latitude_deg;
    double current_longitude_deg;
    double target_latitude_deg;
    double target_longitude_deg;

    float current_heading_deg;
    float target_bearing_deg;
    float distance_error_m;
    float heading_error_deg;
    float command_vx;
    float command_vy;
    float command_wz;
} GPS_NavigationDebug_t;

/**
 * @brief Keil Watch-only mirror of the complete GPS receive/parse state.
 *
 * Add the single symbol g_gps_debug to a Keil Watch window and expand it.
 * Expand navigation for target-point and controller-error data. The GPS driver
 * writes the receiver fields, while app_Navigation.c writes only navigation.
 * Navigation, chassis mode arbitration, PID and motor output never read values
 * back from this object, so it cannot become a control input.
 *
 * update_sequence is incremented before and after each mirror update. An even
 * value represents a complete snapshot; if the debugger stops on an odd value,
 * run/step once and inspect it again.
 */
typedef struct GPS_Debug_s
{
    uint32_t update_sequence;

    /* Target point, waypoint index, distance/heading error and Vx/Vy/Wz. */
    GPS_NavigationDebug_t navigation;

    /* UART receive-path activity. */
    uint32_t rx_event_count;
    uint32_t rx_byte_count;
    uint32_t last_rx_event_tick;
    uint16_t last_rx_size;
    uint8_t last_message_type;  /* GPS_DEBUG_MSG_* */
    uint8_t position_valid;     /* Latest GGA has qf >= 1 and valid N/S + E/W fields. */

    /* Successful decode counters and most recent decoded-message tick. */
    uint32_t gga_update_count;
    uint32_t ths_update_count;
    uint32_t hpr_update_count;
    uint32_t agric_update_count;
    uint32_t last_decoded_tick;

    /* Convenient top-level GGA position/fix fields. */
    double utc_time;
    double latitude_deg;       /* Signed WGS-84 decimal degrees. */
    double longitude_deg;      /* Signed WGS-84 decimal degrees. */
    float altitude_m;
    float hdop;
    float differential_age_s;
    uint8_t fix_quality;       /* 0 invalid, 1 GPS, 2 DGPS, 4 RTK fixed, 5 RTK float. */
    uint8_t satellites;
    char latitude_direction;
    char longitude_direction;

    /* Source-specific freshness timestamps. */
    uint32_t gga_last_update_tick;
    uint32_t ths_last_update_tick;
    uint32_t hpr_last_update_tick;
    uint32_t agric_last_update_tick;

    /* Complete latest decoded source structures for detailed expansion. */
    T_GNGGA gga;
    T_GNTHS ths;
    T_GPHPR hpr;
    T_AGRIC agric;
} GPS_Debug_t;

/* Read-only from control/debugger perspective; written only as a debug mirror. */
extern volatile GPS_Debug_t g_gps_debug;

/* ---- Public function prototypes ---- */

/**
 * @brief  Initialize the GPS module.
 *
 * Sends a sequence of configuration commands over UART2 to set up the
 * receiver for dual-antenna RTK operation with the required NMEA and
 * binary output messages at 1 Hz.  Must be called from an RTOS task
 * context (uses osDelay).
 */
void GPS_Init(void);
void GPS_InitBackgroundHook(void);

/**
 * @brief  Send an ASCII command string to the GPS module over UART2.
 *
 * Typical command format: "command_name arguments\r\n".
 * The caller is responsible for providing the correct line terminator.
 *
 * @param p_cmd  Null-terminated command string with \r\n terminator.
 */
void GPS_SendCmd(const char* p_cmd);

/**
 * @brief  Process raw byte stream received from the GPS UART.
 *
 * Should be called from the UART RX callback (interrupt or DMA) to
 * parse both binary AGRIC frames (sync header 0xAA 0x44 0xB5) and
 * NMEA text sentences ($...\r\n) from the incoming byte stream.
 *
 * @param pBuf  Pointer to the raw received byte buffer.
 * @param Size  Number of valid bytes in the buffer.
 */
void GPS_RxPro_HAL(uint8_t* pBuf, uint16_t Size);

/**
 * @brief  Get pointer to the latest parsed $GNGGA position data.
 * @return Read-only pointer to the global T_GNGGA instance.
 */
PT_GNGGA GetGNGGA(void);

/**
 * @brief  Get pointer to the latest parsed $GNTHS heading data.
 * @return Read-only pointer to the global T_GNTHS instance.
 */
PT_GNTHS GetGNTHS(void);

/**
 * @brief  Get pointer to the latest AGRIC binary frame data.
 * @return Read-only pointer to the global T_AGRIC instance.
 */
PT_AGRIC GetAGRIC(void);

/**
 * @brief  Get pointer to the latest parsed $GNHPR attitude data.
 * @return Read-only pointer to the global T_GPHPR instance.
 */
PT_GPHPR GetGPHPR(void);

#endif
