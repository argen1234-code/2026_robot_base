/****
	***********************************************************************************************************************************************************************************

>>>>> ��Ҫ˵����
	*
	*	1.W25QXX�Ĳ���ʱ�����޶���!!! �ֲ�����ĵ��Ͳο�ֵΪ: 4K-45ms, 32K-120ms ,64K-150ms,��Ƭ����20S
	*
	*	2.W25QXX��д��ʱ�����޶���!!! �ֲ�����ĵ��Ͳο�ֵΪ: 256�ֽ�-0.4ms��Ҳ���� 1M�ֽ�/s ��ʵ������600K�ֽ�/s���ң�
	*
	*	3.���ʹ�ÿ⺯��ֱ�Ӷ�ȡ����ô�Ƿ�ʹ��DMA���Ƿ���Cache�����������Ż��ȼ��Լ����ݴ洢����λ��(�ڲ� TCM SRAM ���� AXI SRAM)����Ӱ���ȡ���ٶ�
	*
	*	4.���ʹ���ڴ�ӳ��ģʽ�����ȡ����ֻ��QSPI������ʱ���Լ��Ƿ���Cache�й�
	*
	*	5.ʹ�ÿ⺯������ֱ�Ӷ�ȡ��keil�汾5.30��������AC6.14������ȼ�Oz image size����ȡ�ٶ�Ϊ 7M�ֽ�/S �����ݷ���TCM SRAM ���� AXI SRAM  
	*    ���ǲ��Ľ������ΪCPUֱ�ӷ�������Ĵ�����Ч�ʺܵͣ�ֱ��ʹ��HAL����ж�ȡ�Ļ����ٶȺ���
	*
	*	6.���ʹ��MDMA���ж�ȡ�����Դﵽ 58M�ֽ�/S��ʹ���ڴ�ӳ��ģʽ�Ļ����������Դﵽ����ʱ�ӵ�ȫ�٣�62.14M/s (133MHzʱ����)
	*
	*  7.W25Q64JV ���������������Ƶ��Ϊ133MHz��750��QSPI�������Ƶ��Ҳ��133MHz �����Ƕ���HAL�⺯��ֱ�Ӷ�ȡ���ԣ�����ʱ�ӳ���15M�Ѿ���������������
	*
	*	8.�����ڴ�ӳ��ģʽֱ�Ӷ�ȡ���ԣ�����ʱ�ӳ���127.5M�Ѿ�������������������ΪQSPI�ں�ʱ������޶�Ϊ250M�����Խ���ʵ��QSPI����ʱ�Ӳ�Ҫ����125M��
	*	  �����ʱ��������ο� SystemClock_Config ����
	*
	*	9.ʵ��ʹ���У������ݱȽϴ�ʱ������ʹ��64K����32K����������ʱ���4K������	
	*
	**************************************************************************************************************************************************************************************FJKE*****
***/

#include "bsp_qspi_flash.h"
#include <stdio.h>

QSPI_HandleTypeDef hqspi;	// ����QSPI��������ﱣ��ʹ��cubeMX���ɵı��������������û��ο�����ֲ

/*************************************************************************************************
*	�� �� ��: HAL_QSPI_MspInit
*	��ڲ���: hqspi - QSPI_HandleTypeDef����ı���������ʾ�����QSPI���
*	�� �� ֵ: ��
*	��������: QSPI���ų�ʼ������
*	˵    ��: �ú����ᱻ	MX_QUADSPI_Init ��������	
*************************************************************************************************/

void HAL_QSPI_MspInit(QSPI_HandleTypeDef* hqspi)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	if(hqspi->Instance==QUADSPI)
	{
		__HAL_RCC_QSPI_CLK_ENABLE();		// ʹ��QSPIʱ��

		__HAL_RCC_QSPI_FORCE_RESET();		// ��λQSPI
		__HAL_RCC_QSPI_RELEASE_RESET();		
		
		GPIO_QUADSPI_CLK_ENABLE;		// ʹ�� QUADSPI_CLK IO��ʱ��
		GPIO_QUADSPI_BK1_NCS_ENABLE;	// ʹ�� QUADSPI_BK1_NCS IO��ʱ��
		GPIO_QUADSPI_BK1_IO0_ENABLE;	// ʹ�� QUADSPI_BK1_IO0 IO��ʱ��
		GPIO_QUADSPI_BK1_IO1_ENABLE;	// ʹ�� QUADSPI_BK1_IO1 IO��ʱ��
		GPIO_QUADSPI_BK1_IO2_ENABLE;	// ʹ�� QUADSPI_BK1_IO2 IO��ʱ��
		GPIO_QUADSPI_BK1_IO3_ENABLE;	// ʹ�� QUADSPI_BK1_IO3 IO��ʱ��
		
		/******************************************************  
		 PG6     ------> QUADSPI_BK1_NCS
		 PF6     ------> QUADSPI_BK1_IO3
		 PF7     ------> QUADSPI_BK1_IO2
		 PF8     ------> QUADSPI_BK1_IO0
		 PF10     ------> QUADSPI_CLK
		 PF9     ------> QUADSPI_BK1_IO1
		*******************************************************/
		
		GPIO_InitStruct.Mode 		= GPIO_MODE_AF_PP;				// �����������ģʽ
		GPIO_InitStruct.Pull 		= GPIO_NOPULL;						// ��������
		GPIO_InitStruct.Speed 		= GPIO_SPEED_FREQ_VERY_HIGH;	// ������IO���ٶ�
		
		GPIO_InitStruct.Pin 			= QUADSPI_CLK_PIN;				// QUADSPI_CLK ����
		GPIO_InitStruct.Alternate 	= QUADSPI_CLK_AF;					// QUADSPI_CLK ����
		HAL_GPIO_Init(QUADSPI_CLK_PORT, &GPIO_InitStruct);			// ��ʼ�� QUADSPI_CLK ����

		GPIO_InitStruct.Pin 			= QUADSPI_BK1_NCS_PIN;			// QUADSPI_BK1_NCS ����
		GPIO_InitStruct.Alternate 	= QUADSPI_BK1_NCS_AF;			// QUADSPI_BK1_NCS ����
		HAL_GPIO_Init(QUADSPI_BK1_NCS_PORT, &GPIO_InitStruct);   // ��ʼ�� QUADSPI_BK1_NCS ����
		
		GPIO_InitStruct.Pin 			= QUADSPI_BK1_IO0_PIN;			// QUADSPI_BK1_IO0 ����
		GPIO_InitStruct.Alternate 	= QUADSPI_BK1_IO0_AF;			// QUADSPI_BK1_IO0 ����
		HAL_GPIO_Init(QUADSPI_BK1_IO0_PORT, &GPIO_InitStruct);	// ��ʼ�� QUADSPI_BK1_IO0 ����	
		
		GPIO_InitStruct.Pin 			= QUADSPI_BK1_IO1_PIN;			// QUADSPI_BK1_IO1 ����
		GPIO_InitStruct.Alternate 	= QUADSPI_BK1_IO1_AF;			// QUADSPI_BK1_IO1 ����
		HAL_GPIO_Init(QUADSPI_BK1_IO1_PORT, &GPIO_InitStruct);   // ��ʼ�� QUADSPI_BK1_IO1 ����
		
		GPIO_InitStruct.Pin 			= QUADSPI_BK1_IO2_PIN;			// QUADSPI_BK1_IO2 ����
		GPIO_InitStruct.Alternate 	= QUADSPI_BK1_IO2_AF;			// QUADSPI_BK1_IO2 ����
		HAL_GPIO_Init(QUADSPI_BK1_IO2_PORT, &GPIO_InitStruct);	// ��ʼ�� QUADSPI_BK1_IO2 ����			
		
		GPIO_InitStruct.Pin 			= QUADSPI_BK1_IO3_PIN;			// QUADSPI_BK1_IO3 ����
		GPIO_InitStruct.Alternate 	= QUADSPI_BK1_IO3_AF;			// QUADSPI_BK1_IO3 ����
		HAL_GPIO_Init(QUADSPI_BK1_IO3_PORT, &GPIO_InitStruct);	// ��ʼ�� QUADSPI_BK1_IO3 ����
	}
}

/*************************************************************************************************
*	�� �� ��: MX_QUADSPI_Init
*	��ڲ���: ��
*	�� �� ֵ: ��
*	��������: ��ʼ�� QSPI ����
*	˵    ��: ��	
**************************************************************************************************/

void MX_QUADSPI_Init(void)
{
	
/*��ĳЩ���ϣ��������������㷨ʱ����Ҫ�ֶ��������Ȳ���������Ṥ��������*/	
	uint32_t i;
	char *p;

	/* �˴��ο��������Ĵ��룬��ҿ���ȥѧϰ�������Ľ̳̣��ǳ�רҵ */
	p = (char *)&hqspi;
	for (i = 0; i < sizeof(hqspi); i++)
	{
		*p++ = 0;
	}	
	hqspi.Instance  = QUADSPI;   
	HAL_QSPI_DeInit(&hqspi) ;		// ��λQSPI
/********************/	
	
	hqspi.Instance 					= QUADSPI;									// QSPI����
	
	/*������ѡ�� HCLK ��ΪQSPI���ں�ʱ�ӣ��ٶ�Ϊ240M���پ���2��Ƶ�õ�120M����ʱ�ӣ�
	  ���� QSPI�ں�ʱ�� �����ã���ο� main.c�ļ���� SystemClock_Config ����*/
	// ��Ҫע����ǣ���ʹ���ڴ�ӳ��ģʽʱ������ķ�Ƶϵ����������Ϊ0����������ȡ����
	hqspi.Init.ClockPrescaler 		= 1;											// ʱ�ӷ�Ƶֵ����QSPI�ں�ʱ�ӽ��� 1+1 ��Ƶ�õ�QSPIͨ������ʱ��

	hqspi.Init.FifoThreshold 		= 32;											// FIFO��ֵ
	hqspi.Init.SampleShifting		= QSPI_SAMPLE_SHIFTING_HALFCYCLE;	// ���CLK����֮����в���
	hqspi.Init.FlashSize 			= 22;   // 2^(22+1) = 8MB   // 2^(24+1) = 32MB											// flash��С��FLASH �е��ֽ��� = 2^[FSIZE+1]�����İ������8M�ֽڵ�W25Q64����������Ϊ22
	hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;			// Ƭѡ���ָߵ�ƽ��ʱ��
	hqspi.Init.ClockMode 			= QSPI_CLOCK_MODE_3;						// ģʽ3
	hqspi.Init.FlashID 				= QSPI_FLASH_ID_1;						// ʹ��QSPI1
	hqspi.Init.DualFlash 			= QSPI_DUALFLASH_DISABLE;				// ��ֹ˫����ģʽ

	HAL_QSPI_Init(&hqspi); // ��ʼ������
}
 
/*************************************************************************************************
*	�� �� ��: QSPI_W25Qxx_Init
*	��ڲ���: ��
*	�� �� ֵ: QSPI_W25Qxx_OK - ��ʼ���ɹ���W25Qxx_ERROR_INIT - ��ʼ������
*	��������: ��ʼ�� QSPI ���ã���ȡW25Q64ID
*	˵    ��: ��	
*************************************************************************************************/

int8_t QSPI_W25Qxx_Init(void)
{
	uint32_t	Device_ID;	// ����ID
	
	MX_QUADSPI_Init();							// ��ʼ�� QSPI ����
	QSPI_W25Qxx_Reset();							// ��λ����
	Device_ID = QSPI_W25Qxx_ReadID(); 		// ��ȡ����ID
	
	if( Device_ID == W25Qxx_FLASH_ID )		// ����ƥ��
	{
		printf ("W25Q64 OK,flash ID:%X\r\n",Device_ID);		// ��ʼ���ɹ�
		return QSPI_W25Qxx_OK;			// ���سɹ���־		
	}
	else
	{
		printf ("W25Q64 ERROR!!!!!  ID:%X\r\n",Device_ID);	// ��ʼ��ʧ��	
		return W25Qxx_ERROR_INIT;		// ���ش����־
	}	
}

/*************************************************************************************************
*	�� �� ��: QSPI_W25Qxx_AutoPollingMemReady
*	��ڲ���: ��
*	�� �� ֵ: QSPI_W25Qxx_OK - ͨ������������W25Qxx_ERROR_AUTOPOLLING - ��ѯ�ȴ�����Ӧ
*	��������: ʹ���Զ���ѯ��־��ѯ���ȴ�ͨ�Ž���
*	˵    ��: ÿһ��ͨ�Ŷ�Ӧ�õ��ôκ������ȴ�ͨ�Ž������������Ĳ���	
**************************************************************************************************/

int8_t QSPI_W25Qxx_AutoPollingMemReady(void)
{
	QSPI_CommandTypeDef     s_command;	   // QSPI��������
	QSPI_AutoPollingTypeDef s_config;		// ��ѯ�Ƚ�������ò���

	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;			// 1��ָ��ģʽ
	s_command.AddressMode       = QSPI_ADDRESS_NONE;					// �޵�ַģʽ
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;			//	�޽����ֽ� 
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;	     	 	// ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;	   	// DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;	   	//	ÿ�δ������ݶ�����ָ��	
	s_command.DataMode          = QSPI_DATA_1_LINE;						// 1������ģʽ
	s_command.DummyCycles       = 0;											//	�����ڸ���
	s_command.Instruction       = W25Qxx_CMD_ReadStatus_REG1;	   // ��״̬��Ϣ�Ĵ���
																					
// ��ͣ�Ĳ�ѯ W25Qxx_CMD_ReadStatus_REG1 �Ĵ���������ȡ����״̬�ֽ��е� W25Qxx_Status_REG1_BUSY ��ͣ����0���Ƚ�
// ��״̬�Ĵ���1�ĵ�0λ��ֻ������Busy��־λ�������ڲ���/д������/д����ʱ�ᱻ��1�����л�ͨ�Ž���Ϊ0
	
	s_config.Match           = 0;   									//	ƥ��ֵ
	s_config.MatchMode       = QSPI_MATCH_MODE_AND;	      	//	������
	s_config.Interval        = 0x10;	                     	//	��ѯ���
	s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;	// �Զ�ֹͣģʽ
	s_config.StatusBytesSize = 1;	                        	//	״̬�ֽ���
	s_config.Mask            = W25Qxx_Status_REG1_BUSY;	   // ������ѯģʽ�½��յ�״̬�ֽڽ������Σ�ֻ�Ƚ���Ҫ�õ���λ
		
	// ������ѯ�ȴ�����
	if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING; // ��ѯ�ȴ�����Ӧ
	}
	return QSPI_W25Qxx_OK; // ͨ����������

}

/*************************************************************************************************
*	�� �� ��: QSPI_W25Qxx_Reset
*	��ڲ���: ��
*	�� �� ֵ: QSPI_W25Qxx_OK - ��λ�ɹ���W25Qxx_ERROR_INIT - ��ʼ������
*	��������: ��λ����
*	˵    ��: ��	
*************************************************************************************************/

int8_t QSPI_W25Qxx_Reset(void)	
{
	QSPI_CommandTypeDef s_command;	// QSPI��������

	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;   	// 1��ָ��ģʽ
	s_command.AddressMode 		 = QSPI_ADDRESS_NONE;   			// �޵�ַģʽ
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE; 	// �޽����ֽ� 
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;     	// ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY; 	// DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;	 	// ÿ�δ������ݶ�����ָ��
	s_command.DataMode 			 = QSPI_DATA_NONE;       			// ������ģʽ	
	s_command.DummyCycles 		 = 0;                     			// �����ڸ���
	s_command.Instruction 		 = W25Qxx_CMD_EnableReset;       // ִ�и�λʹ������

	// ���͸�λʹ������
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) 
	{
		return W25Qxx_ERROR_INIT;			// �������ʧ�ܣ����ش�����Ϣ
	}
	// ʹ���Զ���ѯ��־λ���ȴ�ͨ�Ž���
	if (QSPI_W25Qxx_AutoPollingMemReady() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING;	// ��ѯ�ȴ�����Ӧ
	}

	s_command.Instruction  = W25Qxx_CMD_ResetDevice;     // ��λ��������    

	//���͸�λ��������
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) 
	{
		return W25Qxx_ERROR_INIT;		  // �������ʧ�ܣ����ش�����Ϣ
	}
	// ʹ���Զ���ѯ��־λ���ȴ�ͨ�Ž���
	if (QSPI_W25Qxx_AutoPollingMemReady() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING;	// ��ѯ�ȴ�����Ӧ
	}	
	return QSPI_W25Qxx_OK;	// ��λ�ɹ�
}

/*************************************************************************************************
*	�� �� ��: QSPI_W25Qxx_ReadID
*	��ڲ���: ��
*	�� �� ֵ: W25Qxx_ID - ��ȡ��������ID��W25Qxx_ERROR_INIT - ͨ�š���ʼ������
*	��������: ��ʼ�� QSPI ���ã���ȡ����ID
*	˵    ��: ��	
**************************************************************************************************/

uint32_t QSPI_W25Qxx_ReadID(void)	
{
	QSPI_CommandTypeDef s_command;	// QSPI��������
	uint8_t	QSPI_ReceiveBuff[3];		// �洢QSPI����������
	uint32_t	W25Qxx_ID;					// ������ID

	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;    // 1��ָ��ģʽ
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;     	 // 24λ��ַ
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;  // �޽����ֽ� 
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;      // ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY;  // DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;	 // ÿ�δ������ݶ�����ָ��
	s_command.AddressMode		 = QSPI_ADDRESS_NONE;   		 // �޵�ַģʽ
	s_command.DataMode			 = QSPI_DATA_1_LINE;       	 // 1������ģʽ
	s_command.DummyCycles 		 = 0;                   		 // �����ڸ���
	s_command.NbData 				 = 3;                          // �������ݵĳ���
	s_command.Instruction 		 = W25Qxx_CMD_JedecID;         // ִ�ж�����ID����

	// ����ָ��
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) 
	{
//		return W25Qxx_ERROR_INIT;		// �������ʧ�ܣ����ش�����Ϣ
	}
	// ��������
	if (HAL_QSPI_Receive(&hqspi, QSPI_ReceiveBuff, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) 
	{
//		return W25Qxx_ERROR_TRANSMIT;  // �������ʧ�ܣ����ش�����Ϣ
	}
	// ���õ���������ϳ�ID
	W25Qxx_ID = (QSPI_ReceiveBuff[0] << 16) | (QSPI_ReceiveBuff[1] << 8 ) | QSPI_ReceiveBuff[2];

	return W25Qxx_ID; // ����ID
}



/*************************************************************************************************
*	�� �� ��: QSPI_W25Qxx_MemoryMappedMode
*	��ڲ���: ��
*	�� �� ֵ: QSPI_W25Qxx_OK - дʹ�ܳɹ���W25Qxx_ERROR_WriteEnable - дʹ��ʧ��
*	��������: ��QSPI����Ϊ�ڴ�ӳ��ģʽ
*	˵    ��: ����Ϊ�ڴ�ӳ��ģʽʱ��ֻ�ܶ�������д������	
**************************************************************************************************/

int8_t QSPI_W25Qxx_MemoryMappedMode(void)
{
	QSPI_CommandTypeDef      s_command;				 // QSPI��������
	QSPI_MemoryMappedTypeDef s_mem_mapped_cfg;	 // �ڴ�ӳ����ʲ���

	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;    		// 1��ָ��ģʽ
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;            // 24λ��ַ
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;  		// �޽����ֽ� 
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;     		// ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY; 		// DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;			// ÿ�δ������ݶ�����ָ��	
	s_command.AddressMode 		 = QSPI_ADDRESS_4_LINES; 				// 4�ߵ�ַģʽ
	s_command.DataMode    		 = QSPI_DATA_4_LINES;    				// 4������ģʽ
	s_command.DummyCycles 		 = 6;                    				// �����ڸ���
	s_command.Instruction 		 = W25Qxx_CMD_FastReadQuad_IO; 		// 1-4-4ģʽ��(1��ָ��4�ߵ�ַ4������)�����ٶ�ȡָ��
	
	s_mem_mapped_cfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE; // ���ó�ʱ������, nCS ���ּ���״̬
	s_mem_mapped_cfg.TimeOutPeriod     = 0;									 // ��ʱ�ж�����

	QSPI_W25Qxx_Reset();		// ��λW25Qxx
	
	if (HAL_QSPI_MemoryMapped(&hqspi, &s_command, &s_mem_mapped_cfg) != HAL_OK)	// ��������
	{
		return W25Qxx_ERROR_MemoryMapped; 	// �����ڴ�ӳ��ģʽ����
	}

	return QSPI_W25Qxx_OK; // ���óɹ�
}

/*************************************************************************************************
*	�� �� ��: QSPI_W25Qxx_WriteEnable
*	��ڲ���: ��
*	�� �� ֵ: QSPI_W25Qxx_OK - дʹ�ܳɹ���W25Qxx_ERROR_WriteEnable - дʹ��ʧ��
*	��������: ����дʹ������
*	˵    ��: ��	
**************************************************************************************************/

int8_t QSPI_W25Qxx_WriteEnable(void)
{
	QSPI_CommandTypeDef     s_command;	   // QSPI��������
	QSPI_AutoPollingTypeDef s_config;		// ��ѯ�Ƚ�������ò���

	s_command.InstructionMode   	= QSPI_INSTRUCTION_1_LINE;    	// 1��ָ��ģʽ
	s_command.AddressMode 			= QSPI_ADDRESS_NONE;   		      // �޵�ַģʽ
	s_command.AlternateByteMode 	= QSPI_ALTERNATE_BYTES_NONE;  	// �޽����ֽ� 
	s_command.DdrMode           	= QSPI_DDR_MODE_DISABLE;      	// ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  	= QSPI_DDR_HHC_ANALOG_DELAY;  	// DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          	= QSPI_SIOO_INST_EVERY_CMD;		// ÿ�δ������ݶ�����ָ��	
	s_command.DataMode 				= QSPI_DATA_NONE;       	      // ������ģʽ
	s_command.DummyCycles 			= 0;                   	         // �����ڸ���
	s_command.Instruction	 		= W25Qxx_CMD_WriteEnable;      	// ����дʹ������

	// ����дʹ������
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK) 
	{
		return W25Qxx_ERROR_WriteEnable;	//
	}
	
// ��ͣ�Ĳ�ѯ W25Qxx_CMD_ReadStatus_REG1 �Ĵ���������ȡ����״̬�ֽ��е� W25Qxx_Status_REG1_WEL ��ͣ���� 0x02 ���Ƚ�
// ��״̬�Ĵ���1�ĵ�1λ��ֻ������WELдʹ�ܱ�־λ���ñ�־λΪ1ʱ���������Խ���д����
	
	s_config.Match           = 0x02;  								// ƥ��ֵ
	s_config.Mask            = W25Qxx_Status_REG1_WEL;	 		// ��״̬�Ĵ���1�ĵ�1λ��ֻ������WELдʹ�ܱ�־λ���ñ�־λΪ1ʱ���������Խ���д����
	s_config.MatchMode       = QSPI_MATCH_MODE_AND;			 	// ������
	s_config.StatusBytesSize = 1;									 	// ״̬�ֽ���
	s_config.Interval        = 0x10;							 		// ��ѯ���
	s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;	// �Զ�ֹͣģʽ

	s_command.Instruction    = W25Qxx_CMD_ReadStatus_REG1;	// ��״̬��Ϣ�Ĵ���
	s_command.DataMode       = QSPI_DATA_1_LINE;					// 1������ģʽ
	s_command.NbData         = 1;										// ���ݳ���

	// ������ѯ�ȴ�����	
	if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING; 	// ��ѯ�ȴ�����Ӧ
	}	
	return QSPI_W25Qxx_OK;  // ͨ����������
}

/*************************************************************************************************
*
*	�� �� ��: QSPI_W25Qxx_SectorErase
*
*	��ڲ���: SectorAddress - Ҫ�����ĵ�ַ
*
*	�� �� ֵ: QSPI_W25Qxx_OK - �����ɹ�
*			    W25Qxx_ERROR_Erase - ����ʧ��
*				 W25Qxx_ERROR_AUTOPOLLING - ��ѯ�ȴ�����Ӧ
*
*	��������: ������������������ÿ�β���4K�ֽ�
*
*	˵    ��: 1.���� W25Q64JV �����ֲ�����Ĳ����ο�ʱ�䣬����ֵΪ 45ms�����ֵΪ400ms
*				 2.ʵ�ʵĲ����ٶȿ��ܴ���45ms��Ҳ����С��45ms
*				 3.flashʹ�õ�ʱ��Խ������������ʱ��Ҳ��Խ��
*
**************************************************************************************************/

int8_t QSPI_W25Qxx_SectorErase(uint32_t SectorAddress)	
{
	QSPI_CommandTypeDef s_command;	// QSPI��������
	
	s_command.InstructionMode   	= QSPI_INSTRUCTION_1_LINE;    // 1��ָ��ģʽ
	s_command.AddressSize       	= QSPI_ADDRESS_24_BITS;       // 24λ��ַģʽ
	s_command.AlternateByteMode 	= QSPI_ALTERNATE_BYTES_NONE;  //	�޽����ֽ� 
	s_command.DdrMode           	= QSPI_DDR_MODE_DISABLE;      // ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  	= QSPI_DDR_HHC_ANALOG_DELAY;  // DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          	= QSPI_SIOO_INST_EVERY_CMD;	// ÿ�δ������ݶ�����ָ��
	s_command.AddressMode 			= QSPI_ADDRESS_1_LINE;        // 1�ߵ�ַģʽ
	s_command.DataMode 				= QSPI_DATA_NONE;             // ������
	s_command.DummyCycles 			= 0;                          // �����ڸ���
	s_command.Address           	= SectorAddress;              // Ҫ�����ĵ�ַ
	s_command.Instruction	 		= W25Qxx_CMD_SectorErase;     // ������������

	// ����дʹ��
	if (QSPI_W25Qxx_WriteEnable() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_WriteEnable;		// дʹ��ʧ��
	}
	// ������������
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_Erase;				// ����ʧ��
	}
	// ʹ���Զ���ѯ��־λ���ȴ������Ľ��� 
	if (QSPI_W25Qxx_AutoPollingMemReady() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING;		// ��ѯ�ȴ�����Ӧ
	}
	return QSPI_W25Qxx_OK; // �����ɹ�
}

/*************************************************************************************************
*
*	�� �� ��: QSPI_W25Qxx_BlockErase_32K
*
*	��ڲ���: SectorAddress - Ҫ�����ĵ�ַ
*
*	�� �� ֵ: QSPI_W25Qxx_OK - �����ɹ�
*			    W25Qxx_ERROR_Erase - ����ʧ��
*				 W25Qxx_ERROR_AUTOPOLLING - ��ѯ�ȴ�����Ӧ
*
*	��������: ���п����������ÿ�β���32K�ֽ�
*
*	˵    ��: 1.���� W25Q64JV �����ֲ�����Ĳ����ο�ʱ�䣬����ֵΪ 120ms�����ֵΪ1600ms
*				 2.ʵ�ʵĲ����ٶȿ��ܴ���120ms��Ҳ����С��120ms
*				 3.flashʹ�õ�ʱ��Խ������������ʱ��Ҳ��Խ��
*
*************************************************************************************************/

int8_t QSPI_W25Qxx_BlockErase_32K (uint32_t SectorAddress)	
{
	QSPI_CommandTypeDef s_command;	// QSPI��������
	
	s_command.InstructionMode   	= QSPI_INSTRUCTION_1_LINE;    // 1��ָ��ģʽ
	s_command.AddressSize       	= QSPI_ADDRESS_24_BITS;       // 24λ��ַģʽ
	s_command.AlternateByteMode 	= QSPI_ALTERNATE_BYTES_NONE;  //	�޽����ֽ� 
	s_command.DdrMode           	= QSPI_DDR_MODE_DISABLE;      // ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  	= QSPI_DDR_HHC_ANALOG_DELAY;  // DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          	= QSPI_SIOO_INST_EVERY_CMD;	// ÿ�δ������ݶ�����ָ��
	s_command.AddressMode 			= QSPI_ADDRESS_1_LINE;        // 1�ߵ�ַģʽ
	s_command.DataMode 				= QSPI_DATA_NONE;             // ������
	s_command.DummyCycles 			= 0;                          // �����ڸ���
	s_command.Address           	= SectorAddress;              // Ҫ�����ĵ�ַ
	s_command.Instruction	 		= W25Qxx_CMD_BlockErase_32K;  // ��������ÿ�β���32K�ֽ�

	// ����дʹ��	
	if (QSPI_W25Qxx_WriteEnable() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_WriteEnable;		// дʹ��ʧ��
	}
	// ������������
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_Erase;				// ����ʧ��
	}
	// ʹ���Զ���ѯ��־λ���ȴ������Ľ��� 
	if (QSPI_W25Qxx_AutoPollingMemReady() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING;		// ��ѯ�ȴ�����Ӧ
	}
	return QSPI_W25Qxx_OK;	// �����ɹ�
}

/*************************************************************************************************
*
*	�� �� ��: QSPI_W25Qxx_BlockErase_64K
*
*	��ڲ���: SectorAddress - Ҫ�����ĵ�ַ
*
*	�� �� ֵ: QSPI_W25Qxx_OK - �����ɹ�
*			    W25Qxx_ERROR_Erase - ����ʧ��
*				 W25Qxx_ERROR_AUTOPOLLING - ��ѯ�ȴ�����Ӧ
*
*	��������: ���п����������ÿ�β���64K�ֽ�
*
*	˵    ��: 1.���� W25Q64JV �����ֲ�����Ĳ����ο�ʱ�䣬����ֵΪ 150ms�����ֵΪ2000ms
*				 2.ʵ�ʵĲ����ٶȿ��ܴ���150ms��Ҳ����С��150ms
*				 3.flashʹ�õ�ʱ��Խ������������ʱ��Ҳ��Խ��
*				 4.ʵ��ʹ�ý���ʹ��64K������������ʱ�����
*
**************************************************************************************************/

int8_t QSPI_W25Qxx_BlockErase_64K (uint32_t SectorAddress)	
{
	QSPI_CommandTypeDef s_command;	// QSPI��������
	
	s_command.InstructionMode   	= QSPI_INSTRUCTION_1_LINE;    // 1��ָ��ģʽ
	s_command.AddressSize       	= QSPI_ADDRESS_24_BITS;       // 24λ��ַģʽ
	s_command.AlternateByteMode 	= QSPI_ALTERNATE_BYTES_NONE;  //	�޽����ֽ� 
	s_command.DdrMode           	= QSPI_DDR_MODE_DISABLE;      // ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  	= QSPI_DDR_HHC_ANALOG_DELAY;  // DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          	= QSPI_SIOO_INST_EVERY_CMD;	// ÿ�δ������ݶ�����ָ��
	s_command.AddressMode 			= QSPI_ADDRESS_1_LINE;        // 1�ߵ�ַģʽ
	s_command.DataMode 				= QSPI_DATA_NONE;             // ������
	s_command.DummyCycles 			= 0;                          // �����ڸ���
	s_command.Address           	= SectorAddress;              // Ҫ�����ĵ�ַ
	s_command.Instruction	 		= W25Qxx_CMD_BlockErase_64K;  // ��������ÿ�β���64K�ֽ�	

	// ����дʹ��
	if (QSPI_W25Qxx_WriteEnable() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_WriteEnable;	// дʹ��ʧ��
	}
	// ������������
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_Erase;			// ����ʧ��
	}
	// ʹ���Զ���ѯ��־λ���ȴ������Ľ��� 
	if (QSPI_W25Qxx_AutoPollingMemReady() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING;	// ��ѯ�ȴ�����Ӧ
	}
	return QSPI_W25Qxx_OK;		// �����ɹ�
}

/*************************************************************************************************
*
*	�� �� ��: QSPI_W25Qxx_ChipErase
*
*	��ڲ���: ��
*
*	�� �� ֵ: QSPI_W25Qxx_OK - �����ɹ�
*			    W25Qxx_ERROR_Erase - ����ʧ��
*				 W25Qxx_ERROR_AUTOPOLLING - ��ѯ�ȴ�����Ӧ
*
*	��������: ������Ƭ��������
*
*	˵    ��: 1.���� W25Q64JV �����ֲ�����Ĳ����ο�ʱ�䣬����ֵΪ 20s�����ֵΪ100s
*				 2.ʵ�ʵĲ����ٶȿ��ܴ���20s��Ҳ����С��20s
*				 3.flashʹ�õ�ʱ��Խ������������ʱ��Ҳ��Խ��
*
*************************************************************************************************/

int8_t QSPI_W25Qxx_ChipErase (void)	
{
	QSPI_CommandTypeDef s_command;		// QSPI��������
	QSPI_AutoPollingTypeDef s_config;	// ��ѯ�ȴ����ò���

	s_command.InstructionMode   	= QSPI_INSTRUCTION_1_LINE;    // 1��ָ��ģʽ
	s_command.AddressSize       	= QSPI_ADDRESS_24_BITS;       // 24λ��ַģʽ
	s_command.AlternateByteMode 	= QSPI_ALTERNATE_BYTES_NONE;  //	�޽����ֽ� 
	s_command.DdrMode           	= QSPI_DDR_MODE_DISABLE;      // ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  	= QSPI_DDR_HHC_ANALOG_DELAY;  // DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          	= QSPI_SIOO_INST_EVERY_CMD;	// ÿ�δ������ݶ�����ָ��
	s_command.AddressMode 			= QSPI_ADDRESS_NONE;       	// �޵�ַ
	s_command.DataMode 				= QSPI_DATA_NONE;             // ������
	s_command.DummyCycles 			= 0;                          // �����ڸ���
	s_command.Instruction	 		= W25Qxx_CMD_ChipErase;       // �������������Ƭ����

	// ����дʹ��	
	if (QSPI_W25Qxx_WriteEnable() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_WriteEnable;	// дʹ��ʧ��
	}
	// ������������
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_Erase;		 // ����ʧ��
	}

// ��ͣ�Ĳ�ѯ W25Qxx_CMD_ReadStatus_REG1 �Ĵ���������ȡ����״̬�ֽ��е� W25Qxx_Status_REG1_BUSY ��ͣ����0���Ƚ�
// ��״̬�Ĵ���1�ĵ�0λ��ֻ������Busy��־λ�������ڲ���/д������/д����ʱ�ᱻ��1�����л�ͨ�Ž���Ϊ0
	
	s_config.Match           = 0;   									//	ƥ��ֵ
	s_config.MatchMode       = QSPI_MATCH_MODE_AND;	      	//	������
	s_config.Interval        = 0x10;	                     	//	��ѯ���
	s_config.AutomaticStop   = QSPI_AUTOMATIC_STOP_ENABLE;	// �Զ�ֹͣģʽ
	s_config.StatusBytesSize = 1;	                        	//	״̬�ֽ���
	s_config.Mask            = W25Qxx_Status_REG1_BUSY;	   // ������ѯģʽ�½��յ�״̬�ֽڽ������Σ�ֻ�Ƚ���Ҫ�õ���λ
	
	s_command.Instruction    = W25Qxx_CMD_ReadStatus_REG1;	// ��״̬��Ϣ�Ĵ���
	s_command.DataMode       = QSPI_DATA_1_LINE;					// 1������ģʽ
	s_command.NbData         = 1;										// ���ݳ���

	// W25Q64��Ƭ�����ĵ��Ͳο�ʱ��Ϊ20s�����ʱ��Ϊ100s������ĳ�ʱ�ȴ�ֵ W25Qxx_ChipErase_TIMEOUT_MAX Ϊ 100S
	if (HAL_QSPI_AutoPolling(&hqspi, &s_command, &s_config, W25Qxx_ChipErase_TIMEOUT_MAX) != HAL_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING;	 // ��ѯ�ȴ�����Ӧ
	}
	return QSPI_W25Qxx_OK;
}

/**********************************************************************************************************
*
*	�� �� ��: QSPI_W25Qxx_WritePage
*
*	��ڲ���: pBuffer 		 - Ҫд�������
*				 WriteAddr 		 - Ҫд�� W25Qxx �ĵ�ַ
*				 NumByteToWrite - ���ݳ��ȣ����ֻ��256�ֽ�
*
*	�� �� ֵ: QSPI_W25Qxx_OK 		     - д���ݳɹ�
*			    W25Qxx_ERROR_WriteEnable - дʹ��ʧ��
*				 W25Qxx_ERROR_TRANSMIT	  - ����ʧ��
*				 W25Qxx_ERROR_AUTOPOLLING - ��ѯ�ȴ�����Ӧ
*
*	��������: ��ҳд�룬���ֻ��256�ֽڣ�������д��֮ǰ���������ɲ�������
*
*	˵    ��: 1.Flash��д��ʱ��Ͳ���ʱ��һ�������޶��ģ�������˵QSPI����ʱ��133M�Ϳ���������ٶȽ���д��
*				 2.���� W25Q64JV �����ֲ������ ҳ(256�ֽ�) д��ο�ʱ�䣬����ֵΪ 0.4ms�����ֵΪ3ms
*				 3.ʵ�ʵ�д���ٶȿ��ܴ���0.4ms��Ҳ����С��0.4ms
*				 4.Flashʹ�õ�ʱ��Խ����д������ʱ��Ҳ��Խ��
*				 5.������д��֮ǰ���������ɲ�������
*
***********************************************************************************************************/

int8_t QSPI_W25Qxx_WritePage(uint8_t* pBuffer, uint32_t WriteAddr, uint16_t NumByteToWrite)
{
	QSPI_CommandTypeDef s_command;	// QSPI��������	
	
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;    		// 1��ָ��ģʽ
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;            // 24λ��ַ
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;  		// �޽����ֽ� 
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;     		// ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY; 		// DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;			// ÿ�δ������ݶ�����ָ��	
	s_command.AddressMode 		 = QSPI_ADDRESS_1_LINE; 				// 1�ߵ�ַģʽ
	s_command.DataMode    		 = QSPI_DATA_4_LINES;    				// 4������ģʽ
	s_command.DummyCycles 		 = 0;                    				// �����ڸ���
	s_command.NbData      		 = NumByteToWrite;      			   // ���ݳ��ȣ����ֻ��256�ֽ�
	s_command.Address     		 = WriteAddr;         					// Ҫд�� W25Qxx �ĵ�ַ
	s_command.Instruction 		 = W25Qxx_CMD_QuadInputPageProgram; // 1-1-4ģʽ��(1��ָ��1�ߵ�ַ4������)��ҳ���ָ��
	
	// дʹ��
	if (QSPI_W25Qxx_WriteEnable() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_WriteEnable;	// дʹ��ʧ��
	}
	// д����
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_TRANSMIT;		// �������ݴ���
	}
	// ��ʼ��������
	if (HAL_QSPI_Transmit(&hqspi, pBuffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_TRANSMIT;		// �������ݴ���
	}
	// ʹ���Զ���ѯ��־λ���ȴ�д��Ľ��� 
	if (QSPI_W25Qxx_AutoPollingMemReady() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING; // ��ѯ�ȴ�����Ӧ
	}
	return QSPI_W25Qxx_OK;	// д���ݳɹ�
}

/**********************************************************************************************************
*
*	�� �� ��: QSPI_W25Qxx_WriteBuffer
*
*	��ڲ���: pBuffer 		 - Ҫд�������
*				 WriteAddr 		 - Ҫд�� W25Qxx �ĵ�ַ
*				 NumByteToWrite - ���ݳ��ȣ�����ܳ���flashоƬ�Ĵ�С
*
*	�� �� ֵ: QSPI_W25Qxx_OK 		     - д���ݳɹ�
*			    W25Qxx_ERROR_WriteEnable - дʹ��ʧ��
*				 W25Qxx_ERROR_TRANSMIT	  - ����ʧ��
*				 W25Qxx_ERROR_AUTOPOLLING - ��ѯ�ȴ�����Ӧ
*
*	��������: д�����ݣ�����ܳ���flashоƬ�Ĵ�С���������ɲ�������
*
*	˵    ��: 1.Flash��д��ʱ��Ͳ���ʱ��һ���������޶��ģ�������˵QSPI����ʱ��133M�Ϳ���������ٶȽ���д��
*				 2.���� W25Q64JV �����ֲ������ ҳ д��ο�ʱ�䣬����ֵΪ 0.4ms�����ֵΪ3ms
*				 3.ʵ�ʵ�д���ٶȿ��ܴ���0.4ms��Ҳ����С��0.4ms
*				 4.Flashʹ�õ�ʱ��Խ����д������ʱ��Ҳ��Խ��
*				 5.������д��֮ǰ���������ɲ�������
*				 6.�ú�����ֲ�� stm32h743i_eval_qspi.c
*
**********************************************************************************************************/

int8_t QSPI_W25Qxx_WriteBuffer(uint8_t* pBuffer, uint32_t WriteAddr, uint32_t Size)
{	
	uint32_t end_addr, current_size, current_addr;
	uint8_t *write_data;  // Ҫд�������

	current_size = W25Qxx_PageSize - (WriteAddr % W25Qxx_PageSize); // ���㵱ǰҳ��ʣ��Ŀռ�

	if (current_size > Size)	// �жϵ�ǰҳʣ��Ŀռ��Ƿ��㹻д����������
	{
		current_size = Size;		// ����㹻����ֱ�ӻ�ȡ��ǰ����
	}

	current_addr = WriteAddr;		// ��ȡҪд��ĵ�ַ
	end_addr = WriteAddr + Size;	// ���������ַ
	write_data = pBuffer;			// ��ȡҪд�������

	do
	{
		// ����дʹ��
		if (QSPI_W25Qxx_WriteEnable() != QSPI_W25Qxx_OK)
		{
			return W25Qxx_ERROR_WriteEnable;
		}

		// ��ҳд������
		else if(QSPI_W25Qxx_WritePage(write_data, current_addr, current_size) != QSPI_W25Qxx_OK)
		{
			return W25Qxx_ERROR_TRANSMIT;
		}

		// ʹ���Զ���ѯ��־λ���ȴ�д��Ľ��� 
		else 	if (QSPI_W25Qxx_AutoPollingMemReady() != QSPI_W25Qxx_OK)
		{
			return W25Qxx_ERROR_AUTOPOLLING;
		}

		else // ��ҳд�����ݳɹ���������һ��д���ݵ�׼������
		{
			current_addr += current_size;	// ������һ��Ҫд��ĵ�ַ
			write_data += current_size;	// ��ȡ��һ��Ҫд������ݴ洢����ַ
			// ������һ��д���ݵĳ���
			current_size = ((current_addr + W25Qxx_PageSize) > end_addr) ? (end_addr - current_addr) : W25Qxx_PageSize;
		}
	}
	while (current_addr < end_addr) ; // �ж������Ƿ�ȫ��д�����

	return QSPI_W25Qxx_OK;	// д�����ݳɹ�

}

/**********************************************************************************************************************************
*
*	�� �� ��: QSPI_W25Qxx_ReadBuffer
*
*	��ڲ���: pBuffer 		 - Ҫ��ȡ������
*				 ReadAddr 		 - Ҫ��ȡ W25Qxx �ĵ�ַ
*				 NumByteToRead  - ���ݳ��ȣ�����ܳ���flashоƬ�Ĵ�С
*
*	�� �� ֵ: QSPI_W25Qxx_OK 		     - �����ݳɹ�
*				 W25Qxx_ERROR_TRANSMIT	  - ����ʧ��
*				 W25Qxx_ERROR_AUTOPOLLING - ��ѯ�ȴ�����Ӧ
*
*	��������: ��ȡ���ݣ�����ܳ���flashоƬ�Ĵ�С
*
*	˵    ��: 1.Flash�Ķ�ȡ�ٶ�ȡ����QSPI��ͨ��ʱ�ӣ�����ܳ���133M
*				 2.����ʹ�õ���1-4-4ģʽ��(1��ָ��4�ߵ�ַ4������)�����ٶ�ȡָ�� Fast Read Quad I/O
*				 3.ʹ�ÿ��ٶ�ȡָ�����п����ڵģ�����ο�W25Q64JV���ֲ�  Fast Read Quad I/O  ��0xEB��ָ��
*				 4.ʵ��ʹ���У��Ƿ�ʹ��DMA�����������Ż��ȼ��Լ����ݴ洢����λ��(�ڲ� TCM SRAM ���� AXI SRAM)����Ӱ���ȡ���ٶ�
*			    5.�ڱ������У�ʹ�õ��ǿ⺯������ֱ�Ӷ�д��keil�汾5.30��������AC6.14������ȼ�Oz image size����ȡ�ٶ�Ϊ 7M�ֽ�/S ��
*		         ���ݷ��� TCM SRAM ���� AXI SRAM ���ǲ��Ľ��
*		       6.��ΪCPUֱ�ӷ�������Ĵ�����Ч�ʺܵͣ�ֱ��ʹ��HAL����ж�д�Ļ����ٶȺ�����ʹ��MDMA���ж�ȡ�����Դﵽ 58M�ֽ�/S
*	          7. W25Q64JV ���������������Ƶ��Ϊ133MHz��750��QSPI�������Ƶ��Ҳ��133MHz �����Ƕ���HAL�⺯��ֱ�Ӷ�ȡ���ԣ�
*		          ����ʱ�ӳ���15M�Ѿ���������������������ٶ�Ҫ��ߵĳ��Ͽ�����MDMA�ķ�ʽ
*
*****************************************************************************************************************FJKE************/

int8_t QSPI_W25Qxx_ReadBuffer(uint8_t* pBuffer, uint32_t ReadAddr, uint32_t NumByteToRead)
{
	QSPI_CommandTypeDef s_command;	// QSPI��������
	
	s_command.InstructionMode   = QSPI_INSTRUCTION_1_LINE;    		// 1��ָ��ģʽ
	s_command.AddressSize       = QSPI_ADDRESS_24_BITS;            // 24λ��ַ
	s_command.AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;  		// �޽����ֽ� 
	s_command.DdrMode           = QSPI_DDR_MODE_DISABLE;     		// ��ֹDDRģʽ
	s_command.DdrHoldHalfCycle  = QSPI_DDR_HHC_ANALOG_DELAY; 		// DDRģʽ�������ӳ٣������ò���
	s_command.SIOOMode          = QSPI_SIOO_INST_EVERY_CMD;			// ÿ�δ������ݶ�����ָ��	
	s_command.AddressMode 		 = QSPI_ADDRESS_4_LINES; 				// 4�ߵ�ַģʽ
	s_command.DataMode    		 = QSPI_DATA_4_LINES;    				// 4������ģʽ
	s_command.DummyCycles 		 = 6;                    				// �����ڸ���
	s_command.NbData      		 = NumByteToRead;      			   	// ���ݳ��ȣ�����ܳ���flashоƬ�Ĵ�С
	s_command.Address     		 = ReadAddr;         					// Ҫ��ȡ W25Qxx �ĵ�ַ
	s_command.Instruction 		 = W25Qxx_CMD_FastReadQuad_IO; 		// 1-4-4ģʽ��(1��ָ��4�ߵ�ַ4������)�����ٶ�ȡָ��
	
	// ���Ͷ�ȡ����
	if (HAL_QSPI_Command(&hqspi, &s_command, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_TRANSMIT;		// �������ݴ���
	}

	//	��������
	
	if (HAL_QSPI_Receive(&hqspi, pBuffer, HAL_QPSI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
	{
		return W25Qxx_ERROR_TRANSMIT;		// �������ݴ���
	}

	// ʹ���Զ���ѯ��־λ���ȴ����յĽ��� 
	if (QSPI_W25Qxx_AutoPollingMemReady() != QSPI_W25Qxx_OK)
	{
		return W25Qxx_ERROR_AUTOPOLLING; // ��ѯ�ȴ�����Ӧ
	}
	return QSPI_W25Qxx_OK;	// ��ȡ���ݳɹ�
}



/********************************************************************************************************************************************************************************************************FJKE**********/

