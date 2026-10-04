#include "bsp_oled.h"
#include "bsp_oled_font.h"
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "stdarg.h"

#if I2C2_HARDWARE

extern I2C_HandleTypeDef hi2c2;

/**
  * @brief  OLED写命令（硬件I2C版本）
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
    uint8_t buf[2] = {0x00, Command};  // 0x00表示命令模式
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
}

/**
  * @brief  OLED写数据（硬件I2C版本）
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
    uint8_t buf[2] = {0x40, Data};  // 0x40表示数据模式
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDRESS, buf, 2, HAL_MAX_DELAY);
}

/**
  * @brief  批量写数据（优化性能）
  * @param  Data 数据数组
  * @param  Count 数据数量
  * @retval 无
  */
void OLED_WriteDataBuffer(uint8_t *Data, uint16_t Count)
{
    // 为每个数据添加数据头(0x40)
    uint8_t *buffer = (uint8_t*)malloc(Count + 1);
    if(buffer == NULL) return;
    
    buffer[0] = 0x40;  // 数据模式
    memcpy(&buffer[1], Data, Count);
    
    HAL_I2C_Master_Transmit(&hi2c2, OLED_I2C_ADDRESS, buffer, Count + 1, HAL_MAX_DELAY);
    
    free(buffer);
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
    OLED_WriteCommand(0xB0 | Y);					// 设置Y位置
    OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	// 设置X位置高4位
    OLED_WriteCommand(0x00 | (X & 0x0F));			// 设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{  
    uint8_t i, j;
    
    for (j = 0; j < 8; j++)
    {
        OLED_SetCursor(j, 0);
        // 使用批量写入优化性能
        for(i = 0; i < 128; i++)
        {
            OLED_WriteData(0x00);
        }
        // 或者使用批量写入函数（需要先实现OLED_WriteDataBuffer）
        // OLED_WriteDataBuffer(empty_line, 128);
    }
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
    uint8_t i;
    OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		// 设置光标位置在上半部分
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i]);			// 显示上半部分内容
    }
    OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	// 设置光标位置在下半部分
    for (i = 0; i < 8; i++)
    {
        OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		// 显示下半部分内容
    }
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
    uint8_t i;
    for (i = 0; String[i] != '\0'; i++)
    {
        OLED_ShowChar(Line, Column + i, String[i]);
    }
}

// 以下函数保持不变...
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
    {
        Result *= X;
    }
    return Result;
}

void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)							
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
    uint8_t i;
    uint32_t Number1;
    if (Number >= 0)
    {
        OLED_ShowChar(Line, Column, '+');
        Number1 = Number;
    }
    else
    {
        OLED_ShowChar(Line, Column, '-');
        Number1 = -Number;
    }
    for (i = 0; i < Length; i++)							
    {
        OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
    }
}

void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i, SingleNumber;
    for (i = 0; i < Length; i++)							
    {
        SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
        if (SingleNumber < 10)
        {
            OLED_ShowChar(Line, Column + i, SingleNumber + '0');
        }
        else
        {
            OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
        }
    }
}

void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)							
    {
        OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
    }
}

/**
  * @brief  OLED初始化（硬件I2C版本）
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
    
    // 上电延时
    HAL_Delay(100);
    
    // 注意：不再需要软件I2C初始化
    
    OLED_WriteCommand(0xAE);	// 关闭显示
    
    OLED_WriteCommand(0xD5);	// 设置显示时钟分频比/振荡器频率
    OLED_WriteCommand(0x80);
    
    OLED_WriteCommand(0xA8);	// 设置多路复用率
    OLED_WriteCommand(0x3F);
    
    OLED_WriteCommand(0xD3);	// 设置显示偏移
    OLED_WriteCommand(0x00);
    
    OLED_WriteCommand(0x40);	// 设置显示开始行
    
    OLED_WriteCommand(0xA1);	// 设置左右方向，0xA1正常 0xA0左右反置
    
    OLED_WriteCommand(0xC8);	// 设置上下方向，0xC8正常 0xC0上下反置

    OLED_WriteCommand(0xDA);	// 设置COM引脚硬件配置
    OLED_WriteCommand(0x12);
    
    OLED_WriteCommand(0x81);	// 设置对比度控制
    OLED_WriteCommand(0xCF);

    OLED_WriteCommand(0xD9);	// 设置预充电周期
    OLED_WriteCommand(0xF1);

    OLED_WriteCommand(0xDB);	// 设置VCOMH取消选择级别
    OLED_WriteCommand(0x30);

    OLED_WriteCommand(0xA4);	// 设置整个显示打开/关闭

    OLED_WriteCommand(0xA6);	// 设置正常/倒转显示

    OLED_WriteCommand(0x8D);	// 设置充电泵
    OLED_WriteCommand(0x14);

    OLED_WriteCommand(0xAF);	// 开启显示
        
    OLED_Clear();				// OLED清屏
}

#endif

#if I2C_SOFTWARE



/*引脚初始化*/
void OLED_I2C_Init(void)
{
	
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C开始
  * @param  无
  * @retval 无
  */
void OLED_I2C_Start(void)
{
	OLED_W_SDA(1);
	OLED_W_SCL(1);
	OLED_W_SDA(0);
	OLED_W_SCL(0);
}

/**
  * @brief  I2C停止
  * @param  无
  * @retval 无
  */
void OLED_I2C_Stop(void)
{
	OLED_W_SDA(0);
	OLED_W_SCL(1);
	OLED_W_SDA(1);
}

/**
  * @brief  I2C发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void OLED_I2C_SendByte(uint8_t Byte)
{
	uint8_t i;
	for (i = 0; i < 8; i++)
	{
		OLED_W_SDA(Byte & (0x80 >> i));
		OLED_W_SCL(1);
		OLED_W_SCL(0);
	}
	OLED_W_SCL(1);	//额外的一个时钟，不处理应答信号
	OLED_W_SCL(0);
}

/**
  * @brief  OLED写命令
  * @param  Command 要写入的命令
  * @retval 无
  */
void OLED_WriteCommand(uint8_t Command)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(OLED_I2C_ADDRESS);		//从机地址
	OLED_I2C_SendByte(0x00);		//写命令
	OLED_I2C_SendByte(Command); 
	OLED_I2C_Stop();
}

/**
  * @brief  OLED写数据
  * @param  Data 要写入的数据
  * @retval 无
  */
void OLED_WriteData(uint8_t Data)
{
	OLED_I2C_Start();
	OLED_I2C_SendByte(OLED_I2C_ADDRESS);		//从机地址
	OLED_I2C_SendByte(0x40);		//写数据
	OLED_I2C_SendByte(Data);
	OLED_I2C_Stop();
}

/**
  * @brief  OLED设置光标位置
  * @param  Y 以左上角为原点，向下方向的坐标，范围：0~7
  * @param  X 以左上角为原点，向右方向的坐标，范围：0~127
  * @retval 无
  */
void OLED_SetCursor(uint8_t Y, uint8_t X)
{
	OLED_WriteCommand(0xB0 | Y);					//设置Y位置
	OLED_WriteCommand(0x10 | ((X & 0xF0) >> 4));	//设置X位置高4位
	OLED_WriteCommand(0x00 | (X & 0x0F));			//设置X位置低4位
}

/**
  * @brief  OLED清屏
  * @param  无
  * @retval 无
  */
void OLED_Clear(void)
{  
	uint8_t i, j;
	for (j = 0; j < 8; j++)
	{
		OLED_SetCursor(j, 0);
		for(i = 0; i < 128; i++)
		{
			OLED_WriteData(0x00);
		}
	}
}

/**
  * @brief  OLED显示一个字符
  * @param  Line 行位置，范围：1~4
  * @param  Column 列位置，范围：1~16
  * @param  Char 要显示的一个字符，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowChar(uint8_t Line, uint8_t Column, char Char)
{      	
	uint8_t i;
	OLED_SetCursor((Line - 1) * 2, (Column - 1) * 8);		//设置光标位置在上半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i]);			//显示上半部分内容
	}
	OLED_SetCursor((Line - 1) * 2 + 1, (Column - 1) * 8);	//设置光标位置在下半部分
	for (i = 0; i < 8; i++)
	{
		OLED_WriteData(OLED_F8x16[Char - ' '][i + 8]);		//显示下半部分内容
	}
}

/**
  * @brief  OLED显示字符串
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  String 要显示的字符串，范围：ASCII可见字符
  * @retval 无
  */
void OLED_ShowString(uint8_t Line, uint8_t Column, char *String)
{
	uint8_t i;
	for (i = 0; String[i] != '\0'; i++)
	{
		OLED_ShowChar(Line, Column + i, String[i]);
	}
}

/**
  * @brief  OLED次方函数
  * @retval 返回值等于X的Y次方
  */
uint32_t OLED_Pow(uint32_t X, uint32_t Y)
{
	uint32_t Result = 1;
	while (Y--)
	{
		Result *= X;
	}
	return Result;
}

/**
  * @brief  OLED显示数字（十进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~4294967295
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十进制，带符号数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：-2147483648~2147483647
  * @param  Length 要显示数字的长度，范围：1~10
  * @retval 无
  */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Column, int32_t Number, uint8_t Length)
{
	uint8_t i;
	uint32_t Number1;
	if (Number >= 0)
	{
		OLED_ShowChar(Line, Column, '+');
		Number1 = Number;
	}
	else
	{
		OLED_ShowChar(Line, Column, '-');
		Number1 = -Number;
	}
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i + 1, Number1 / OLED_Pow(10, Length - i - 1) % 10 + '0');
	}
}

/**
  * @brief  OLED显示数字（十六进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~0xFFFFFFFF
  * @param  Length 要显示数字的长度，范围：1~8
  * @retval 无
  */
void OLED_ShowHexNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i, SingleNumber;
	for (i = 0; i < Length; i++)							
	{
		SingleNumber = Number / OLED_Pow(16, Length - i - 1) % 16;
		if (SingleNumber < 10)
		{
			OLED_ShowChar(Line, Column + i, SingleNumber + '0');
		}
		else
		{
			OLED_ShowChar(Line, Column + i, SingleNumber - 10 + 'A');
		}
	}
}

/**
  * @brief  OLED显示数字（二进制，正数）
  * @param  Line 起始行位置，范围：1~4
  * @param  Column 起始列位置，范围：1~16
  * @param  Number 要显示的数字，范围：0~1111 1111 1111 1111
  * @param  Length 要显示数字的长度，范围：1~16
  * @retval 无
  */
void OLED_ShowBinNum(uint8_t Line, uint8_t Column, uint32_t Number, uint8_t Length)
{
	uint8_t i;
	for (i = 0; i < Length; i++)							
	{
		OLED_ShowChar(Line, Column + i, Number / OLED_Pow(2, Length - i - 1) % 2 + '0');
	}
}

/**
  * @brief  OLED初始化
  * @param  无
  * @retval 无
  */
void OLED_Init(void)
{
	
	HAL_Delay(100); //上电延时
	
	OLED_I2C_Init();			//端口初始化
	
	OLED_WriteCommand(0xAE);	//关闭显示
	
	OLED_WriteCommand(0xD5);	//设置显示时钟分频比/振荡器频率
	OLED_WriteCommand(0x80);
	
	OLED_WriteCommand(0xA8);	//设置多路复用率
	OLED_WriteCommand(0x3F);
	
	OLED_WriteCommand(0xD3);	//设置显示偏移
	OLED_WriteCommand(0x00);
	
	OLED_WriteCommand(0x40);	//设置显示开始行
	
	OLED_WriteCommand(0xA1);	//设置左右方向，0xA1正常 0xA0左右反置
	
	OLED_WriteCommand(0xC8);	//设置上下方向，0xC8正常 0xC0上下反置

	OLED_WriteCommand(0xDA);	//设置COM引脚硬件配置
	OLED_WriteCommand(0x12);
	
	OLED_WriteCommand(0x81);	//设置对比度控制
	OLED_WriteCommand(0xCF);

	OLED_WriteCommand(0xD9);	//设置预充电周期
	OLED_WriteCommand(0xF1);

	OLED_WriteCommand(0xDB);	//设置VCOMH取消选择级别
	OLED_WriteCommand(0x30);

	OLED_WriteCommand(0xA4);	//设置整个显示打开/关闭

	OLED_WriteCommand(0xA6);	//设置正常/倒转显示

	OLED_WriteCommand(0x8D);	//设置充电泵
	OLED_WriteCommand(0x14);

	OLED_WriteCommand(0xAF);	//开启显示
		
	OLED_Clear();				//OLED清屏
}

#endif


#if OLED_PRINTF

/**
  * @brief  OLED格式化打印函数（支持浮点数指定位小数）
  * @param  Line 起始行位置，范围：1~8
  * @param  Column 起始列位置，范围：1~16
  * @param  format 格式化字符串
  * @param  ... 可变参数
  * @retval 实际显示的字符数
  */
int oled_printf(uint8_t Line, uint8_t Column, const char *format, ...)
{
    char buffer[256];
    va_list args;
    uint8_t current_line = Line;
    uint8_t current_col = Column;
    uint8_t i = 0;
    int char_count = 0;
    
    // 参数验证
    if (Line < 1 || Line > 8 || Column < 1 || Column > 16) {
        return -1;
    }
    
    // 参数解析
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    if (len < 0) {
        return -1;  // 格式化错误
    }
    
    // 逐个字符处理
    while (buffer[i] != '\0' && current_line <= 8)
    {
        // 处理回车+换行 "\r\n"
        if (buffer[i] == '\r' && buffer[i+1] == '\n') {
            current_col = 1;
            current_line++;
            i += 2;  // 跳过两个字符
            continue;
        }
        
        // 处理换行符 '\n'
        if (buffer[i] == '\n') {
            current_col = 1;
            current_line++;
            i++;
            
            // 检查是否超出屏幕
            if (current_line > 8) {
                break;
            }
            continue;
        }
        
        // 处理回车符 '\r'（单独出现）
        if (buffer[i] == '\r') {
            current_col = 1;  // 回到行首，不换行
            i++;
            continue;
        }
        
        // 处理制表符 '\t'（可选功能）
        if (buffer[i] == '\t') {
            // 制表符宽度为4个空格
            uint8_t spaces = 4 - ((current_col - 1) % 4);
            for (uint8_t s = 0; s < spaces && current_col <= 16; s++) {
                OLED_ShowChar(current_line, current_col, ' ');
                current_col++;
                char_count++;
                
                // 检查自动换行
                if (current_col > 16) {
                    current_col = 1;
                    current_line++;
                    if (current_line > 8) break;
                }
            }
            i++;
            continue;
        }
        
        // 显示普通字符
        OLED_ShowChar(current_line, current_col, buffer[i]);
        char_count++;
        
        // 更新位置
        current_col++;
        
        // 检查是否需要自动换行（到达行末）
        if (current_col > 16) {
            current_col = 1;
            current_line++;
            
            // 检查是否超出屏幕范围
            if (current_line > 8) {
                break;
            }
        }
        
        i++;
    }
    
    return char_count;
}


/**
  * @brief  清除从指定位置开始到行尾的内容
  * @param  Line 行位置，范围：1~8
  * @param  Column 列位置，范围：1~16
  * @retval 无
  */
void oled_clear_line_from(uint8_t Line, uint8_t Column)
{
    if (Line < 1 || Line > 8 || Column < 1 || Column > 16) {
        return;
    }
    
    for (uint8_t col = Column; col <= 16; col++) {
        OLED_ShowChar(Line, col, ' ');
    }
}

/**
  * @brief  清除从指定位置开始到屏幕末尾的内容
  * @param  Line 起始行位置，范围：1~8
  * @param  Column 起始列位置，范围：1~16
  * @retval 无
  */
void oled_clear_from(uint8_t Line, uint8_t Column)
{
    if (Line < 1 || Line > 8 || Column < 1 || Column > 16) {
        return;
    }
    
    // 清除当前行从指定列开始
    oled_clear_line_from(Line, Column);
    
    // 清除后续所有行
    for (uint8_t line = Line + 1; line <= 8; line++) {
        oled_clear_line_from(line, 1);
    }
}

#endif
