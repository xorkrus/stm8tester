#ifndef __HD44780_H__
#define __HD44780_H__

void InitLcd(GPIO_TypeDef* port, GPIO_Pin_TypeDef rs, 
					   GPIO_Pin_TypeDef e, GPIO_Pin_TypeDef data);

void ClearLcd(int dummy);//dummy needs for Cosmic - WTF?

void Outline(int line, char *str);

void SetLine(int line);

void Out(char *str);

void SendData(unsigned char cmd);

void SendCommand(unsigned char cmd);

#define SetCursor(y, x) SendCommand((uint8_t)(0x80 + (0x40*(y-1)) + x))

#endif
