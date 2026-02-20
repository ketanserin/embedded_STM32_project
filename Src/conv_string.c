
#include "conv_string.h"

unsigned int power(unsigned char n)
{
	unsigned int val = 1;
	unsigned char loop;
	if(n == 0) val = 1;
	else{
	val = 10;
	for(loop=0;loop<n-1;loop++){
		val *= 10;
		}
	}
	return val;
}
unsigned int uint2dec( unsigned char* buf,unsigned char length)
{
	unsigned char loop;
	unsigned int dec = 0;

	for(loop=0;loop<length;loop++) {
		dec += (buf[length-loop-1]-0x30) * power(loop);//pow(10,loop);
	}
	return dec;
}
unsigned int char2dec( char* buf,unsigned char length)
{
	unsigned char loop;
	unsigned int dec = 0;

	for(loop=0;loop<length;loop++) {
		dec += (buf[length-loop-1]-0x30) * power(loop);//pow(10,loop);
	}
	return dec;
}
//data = char2dec(&data_buf[3],4);	//DAC 0~4095

//dec2char((char*)&crc_buf[0],0,5,crc);
void dec2char(char* buf,unsigned char dot,unsigned char length,int val)
{
	unsigned char loop;
	unsigned int val_c = 0;

	if(dot != 0) length += 1;
	val_c = sizeof(buf);
	for(loop=0;loop<length;loop++){buf[loop] = 0;}//buffer clear
	//val_c = abs(val);	
	if(val < 0) val_c = (val *-1);
	else val_c = val;
	for(loop=0;loop<length;loop++){
		if((dot != 0) &(dot == loop))buf[length-loop-1] = '.';//7
		else {
			buf[length-loop-1] = val_c % 10;
			buf[length-loop-1] += 0x30;
			val_c /= 10;
		}
	}
}
