/*
 * crc.c
 *
 *  Created on: 2022. 2. 22.
 *      Author: USER
 */

//gen_crc_table()은 MCU 초기화 후 한번 실행
//update_crc()는 받은 데이터의 CRC 검증 , 보낼 데이터의 CRC 계산에 사용합니다.

#include "crc.h"

#define POLYNOMIAL     0x8005
unsigned short g_crc_table[256];
void gen_crc_table(void)
{
	unsigned short i, j;
	unsigned short crc_accum;

	/* 0 ~ 255 까지의 CRC를 미리 계산 */
	/* ex) i=2 이면 0x020000 을 polynomial로 CRC 나눗셈 하는 것임 */
	for(i=0; i<256; i++)
	{
		crc_accum = ((unsigned short)i<<8);
		/* CRC나눗셈이 가능하도록 8-bit shift */
		for(j=0; j<8; j++)
		{
			/* 나머지항의 MSB가 1인지 검사 */
			if(crc_accum & 0x8000)
			crc_accum = (crc_accum << 1) ^ POLYNOMIAL;
			/* 참이면 1-bit shift하고 polynomial을 빼줌(==XOR) */
			else
			crc_accum = (crc_accum << 1);
			/* 거짓이면 1-bit shift만 함*/
		}
		/* 결과적으로 총 16-bit가 shift 되었고 */
		/* 나머지만 남게 됨 */
		g_crc_table[i] = crc_accum;
	}
}
unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,unsigned short data_blk_size)
{
	//register
	unsigned short i, j;
	for(j=0; j<data_blk_size; j++)
	{
		/* 추가된 데이터 ZZ는                      */
		/* 이전 계산시 나머지항의 상위 바이트와    */
		/* 자리수가 맞으므로 그 둘을 합하고 다시   */
		/* 그 자리까지의 나머지를 구함             */
		//               i = ((unsigned short)(crc_accum >> 8) ^ *data_blk_ptr++) & 0xff;
		i = ((unsigned short)(crc_accum >> 8) ^ data_blk_ptr[j]) & 0xff;
		crc_accum = (crc_accum << 8) ^ g_crc_table[i];
		/* 나머지항의 하위 바이트는 뒤에 0x00 이   */
		/* 추가되어 자리수가 올라가고 앞자리의     */
		/* 나머지와 더해짐 */
		//uart1_putc(data_blk_ptr[j]);
	}
	return crc_accum;
}


