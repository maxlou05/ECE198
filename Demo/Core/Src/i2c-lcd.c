
/** Put this in the src folder **/

#include "i2c-lcd.h"
extern I2C_HandleTypeDef hi2c1;  // change your handler here accordingly

#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void lcd_send_cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	HAL_Delay(1);
}

void lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c1, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
	HAL_Delay(1);
}

void lcd_clear (void)
{
	lcd_send_cmd (0x00);
	for (int i=0; i<100; i++)
	{
		lcd_send_data (' ');
	}
}

void lcd_init (void)
{
	// 4 bit initialization
	HAL_Delay(50);  // wait for >40ms
	lcd_send_cmd (0x30);
	HAL_Delay(10);  // wait for >4.1ms
	lcd_send_cmd (0x30);
	HAL_Delay(1);  // wait for >100us
	lcd_send_cmd (0x30);
	HAL_Delay(10);
	lcd_send_cmd (0x20);  // 4bit mode
	HAL_Delay(10);

  // display initialization
	lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	HAL_Delay(1);
	lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	HAL_Delay(1);
	lcd_send_cmd (0x01);  // clear display
	HAL_Delay(1);
	lcd_send_cmd (0x06); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	HAL_Delay(1);
	lcd_send_cmd (0x0C); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
	HAL_Delay(1);
	lcd_clear();
}

void lcd_set_cursor(int row, int col)
{
	/* IMPORTANT!!! Addresses of the display lines:
	   * Line 1: 0x80|0x00 --> from 0x00(0)  to 0x13(19)
	   * Line 3: 0x80|0x14 --> from 0x14(20) to 0x27(39)
	   * Line 2: 0x80|0x40 --> from 0x40(40) to 0x53(59)
	   * Line 4: 0x80|0x54 --> from 0x54(60) to 0x67(79)
	   * NOTE!!! You always need to apply OR with 0x80 (1000 0000) (set bit 8 to 1)
	   * 	ie. lcd_send_cmd(0x80|0x00); to move the cursor to line 1 position 1
	*/

	if((row < 0) || (row > 3)) return;
	if((col < 0) || (col > 19)) return;

    switch (row)
    {
        case 0:
            lcd_send_cmd((0x80|0x00) + col);
            break;
        case 1:
        	lcd_send_cmd((0x80|0x40) + col);
            break;
        case 2:
        	lcd_send_cmd((0x80|0x14) + col);
        	break;
        case 3:
        	lcd_send_cmd((0x80|0x54) + col);
        	break;
    }
}

void lcd_send_string (char *str)
{
	while (*str) lcd_send_data (*str++);
}

void lcd_set_custom_char(char* custom_char, int addr){
	if((addr < 0) || (addr > 7)) return;

	lcd_send_cmd(64 + 8*addr);
	for(int i=0; i<8; i++){
		lcd_send_data(custom_char[i]);
	}
}
