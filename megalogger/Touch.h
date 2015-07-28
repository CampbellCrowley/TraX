#include <Arduino.h>

#define X_CONST 240
#define Y_CONST 320

#define T_CLK 6
#define T_CS 5
#define T_DIN 4
#define T_DOUT 3
#define T_IRQ 2

#define PREC_TOUCH_CONST 10

#define PixSizeX	13.78
#define PixOffsX	411

#define PixSizeY	11.01
#define PixOffsY	378

int TP_X,TP_Y;

void TouchInit(void)
{
	pinMode(T_CLK,  OUTPUT);
    pinMode(T_CS,   OUTPUT);
    pinMode(T_DIN,  OUTPUT);
    pinMode(T_DOUT, INPUT);
    pinMode(T_IRQ,  INPUT);

	digitalWrite(T_CS,  HIGH);
	digitalWrite(T_CLK, HIGH);
	digitalWrite(T_DIN, HIGH);
	digitalWrite(T_CLK, HIGH);
}

void TouchWriteData(unsigned char data)
{
	unsigned char temp;
	unsigned char nop;
	unsigned char count;

	temp=data;
	digitalWrite(T_CLK,LOW);

	for(count=0; count<8; count++)
	{
		if(temp & 0x80)
			digitalWrite(T_DIN, HIGH);
		else
			digitalWrite(T_DIN, LOW);
		temp = temp << 1; 
		digitalWrite(T_CLK, LOW);                
		nop++;
		digitalWrite(T_CLK, HIGH);
		nop++;
	}
}

unsigned int TouchReadData()
{
	unsigned char nop;
	unsigned int data = 0;
	unsigned char count;
	for(count=0; count<12; count++)
	{
		data <<= 1;
		digitalWrite(T_CLK, HIGH);               
		nop++;
		digitalWrite(T_CLK, LOW);
		nop++;
		if (digitalRead(T_DOUT))
			data++;
	}
	return(data);
}
void TouchRead()
{
	unsigned long tx=0;
	unsigned long ty=0;

	digitalWrite(T_CS,LOW);                    

	for (int i=0; i<PREC_TOUCH_CONST; i++)
	{
		TouchWriteData(0x90);        
		digitalWrite(T_CLK,HIGH);
		digitalWrite(T_CLK,LOW); 
		ty+=TouchReadData();

		TouchWriteData(0xD0);      
		digitalWrite(T_CLK,HIGH);
		digitalWrite(T_CLK,LOW);
		tx+=TouchReadData();
	}

	digitalWrite(T_CS,HIGH);

	TP_X=tx/PREC_TOUCH_CONST;
	TP_Y=ty/PREC_TOUCH_CONST;
}

char TouchDataAvailable()
{
  char avail;
  avail = !digitalRead(T_IRQ);
  return avail;
}

int TouchGetX()
{
	int value;
	value = ((TP_X-PixOffsX)/PixSizeX);
	if (value < 0)
		value = 0;
	return value;
}
int TouchGetY()
{
	int value;
	value = ((TP_Y-PixOffsY)/PixSizeY);
	if (value < 0)
		value = 0;
	return value;
}