/*
 * pGC_main.c
 *
 *  Created on: 2025. 7. 2.
 *      Author: ykh
 */

/*
 * Created on: 2025. 7. 2.
 * 해야 되는 것
 * 1.
 * 2.
 * 3.
 *
 *  */



#include "main.h"
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>

//#include "pGC_io.h"
#include "pGC_system.h"
#include "pGC_cmd.h"
#include "pGC_adc.h"
#include "pGC_temp.h"
#include "pGC_IR_i2c.h"
#include "crc.h"

extern float g_temp_degree[];
extern uint16_t g_adcval[];
extern UART_HandleTypeDef huart1;				//optical_rs232_USB
extern UART_HandleTypeDef huart2;				//optical_rs232_USB
extern SPI_HandleTypeDef hspi2;					//dac
extern SPI_HandleTypeDef hspi3;					//adc
extern unsigned char g_rcv_buf[512];
extern unsigned char g_rcv_flag;
extern unsigned char g_rcv_start;
extern unsigned int g_rcv_count;
extern unsigned char g_rcv_bufb[512];
extern unsigned char g_rcv_flagb;
extern unsigned char g_rcv_startb;
extern unsigned int g_rcv_countb;

extern char send_1s_flag;
extern unsigned short update_crc(unsigned short crc_accum, unsigned char *data_blk_ptr,unsigned short data_blk_size);
extern void ePrintf(const char* log, ...);
extern unsigned int char2dec(char* buf,unsigned char length);
extern unsigned int uint2dec(unsigned char* buf,unsigned char length);

#define FIRMWARE_DATE				__DATE__
#define ABS(x) ( ((x)<0)?-(x):(x) )



//#define GROUND
#define FLOATING


/* Method Sending -----------------------------------------------------------*/
void send_to_pc(char* pData,unsigned int length)
{
	HAL_UART_Transmit(&huart2,(unsigned char*)pData,length,5);
}
void send_to_gnd(char* pData,unsigned int length)
{
	HAL_UART_Transmit(&huart1,(unsigned char*)pData,length,5);
}
void send_to_pc_inverted(const void* pData, unsigned int length)
{
    uint8_t buf[length];
    const uint8_t* src = (const uint8_t*)pData;

    /* 1) 구조체를 로컬 버퍼에 복사 */
    memcpy(buf, src, length);

    /* 2) 비트 반전 */
    for (unsigned int i = 0; i < length; i++) {
    	buf[i] = (uint8_t)~(buf[i]);
    }

    /* 3) 전송 */
    HAL_UART_Transmit(&huart1, buf, length, /*timeout*/ 10);
}
/* Command Sending -----------------------------------------------------------*/
void ePrintf(const char* log, ...)
{
	va_list ap;
	char buf[256] = {0};

	va_start(ap,log);
	vsprintf(buf,log,ap);
	va_end(ap);
	HAL_UART_Transmit(&huart2, (unsigned char*)buf,strlen(buf), 0xffff);
}



typedef struct
{
	char st;
	uint8_t adress[2];
	uint8_t AD0[5];
	uint8_t AD1[5];
	uint8_t AD2[5];
	uint8_t AD3[5];
	uint8_t AD4[5];
	uint8_t AD5[5];
	uint8_t AD6[5];
	uint8_t AD7[5];
	uint8_t _CRC_[5];
	char et;
}PC_SEND_data_struct;
PC_SEND_data_struct PC_SEND_data;

typedef struct
{
	char sst;
	uint8_t AD0[5];
	char set;
}sPC_SEND_data_struct;
sPC_SEND_data_struct sPC_SEND_data;

char G0_FLAG=0;
char G1_FLAG=0;
char G2_FLAG=0;
char G3_FLAG=0;
char F0_FLAG=0;
char F1_FLAG=0;
char F2_FLAG=0;
char F3_FLAG=0;
uint16_t DA0=0;
unsigned char G_da_buf_0[5];
unsigned char G_da_buf_1[5];
unsigned char G_da_buf_2[5];
unsigned char G_da_buf_3[5];
unsigned char F_da_buf_0[5];
unsigned char F_da_buf_1[5];
unsigned char F_da_buf_2[5];
unsigned char F_da_buf_3[5];
char _CRC_[5] = {'_','C','R','C','_'};
const char *months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug","Sep", "Oct", "Nov", "Dec"};
unsigned char month = 0;
char m1, m2;

static GPIO_TypeDef * const CS_PORT[] = {
    GPIOB,     // adc_ch == 0일 때 사용할 포트
    GPIOA      // adc_ch == 1일 때 사용할 포트
};

static const uint16_t CS_PIN[] = {
    ADC_ch_IO_0_Pin,  // adc_ch == 0일 때 사용할 핀
    ADC_ch_IO_1_Pin   // adc_ch == 1일 때 사용할 핀
};

int32_t adc_buf[4];
uint8_t  adc_status;

#define DA_CHANNELS   3
#define DA_BUF_LEN    5

/* 채널별 수신 버퍼 */
char G_da_buf[DA_CHANNELS][DA_BUF_LEN];
char F_da_buf[DA_CHANNELS][DA_BUF_LEN];

/* 채널별 플래그 */
volatile uint8_t G_flag[DA_CHANNELS];
volatile uint8_t F_flag[DA_CHANNELS];
/* 채널별 변환 후 값 */
uint16_t G_value[DA_CHANNELS];
uint16_t F_value[DA_CHANNELS];


int16_t  ad0_raw[4];
int16_t  ad1_raw[4];
uint8_t  status;
uint16_t ad0_data[4];
uint16_t ad1_data[4];




void ReadCheckCommand(void) //pc->h/w struct reading
{
    char type = g_rcv_bufb[0];
    char ch   = g_rcv_bufb[1];

    if (ch >= '0' && ch < '0' + DA_CHANNELS) {
        int idx = ch - '0';

        if (type == 'F') {
            // G 타입: 내부 버퍼 복사 + 플래그 세팅
            memcpy(F_da_buf[idx], &g_rcv_bufb[2], DA_BUF_LEN);
            F_flag[idx] = 1;
        }
        // 그 외 타입은 무시
    }
	g_rcv_flagb = 0;

}
/* --- ADS130B04-Q1 레지스터 주소 정의 --- */
#define REG_ID               0x00  // Read-only: 장치 ID  (리셋값 0x54??h)
#define REG_STATUS           0x01  // Read-only: 상태     (리셋값 0x0500h)
#define REG_MODE             0x02  // MODE: 전역 모드 설정 (리셋값 0x0510h)
#define REG_CLOCK            0x03  // CLOCK: 클럭/채널 ON  (리셋값 0x0F8Eh)
#define REG_GAIN             0x04  // GAIN: PGA 게인 설정  (리셋값 0x0000h)
#define REG_GLOBAL_CHOP_CFG  0x06  // CHOP: 채핑 제어      (리셋값 0x0600h)
#define REG_CH0_CFG          0x09  // CH0_CFG: 채널0 MUX  (리셋값 0x0000h)
#define REG_CH1_CFG          0x0E  // CH1_CFG: 채널1 MUX  (리셋값 0x0000h)
#define REG_CH2_CFG          0x13  // CH2_CFG: 채널2 MUX  (리셋값 0x0000h)
#define REG_CH3_CFG          0x18  // CH3_CFG: 채널3 MUX  (리셋값 0x0000h)

#define CMD_RESET    0x06  // 소프트 리셋
#define CMD_SDATAC   0x11  // Stop continuous-read
#define CMD_RDATA    0x12  // Single-shot read data
#define CMD_RDATAC   0x10  // Continuous-read mode



// 읽어올 레지스터 범위 정의
#define ADS130B04_FIRST_REG   0x00    // 시작 주소
#define ADS130B04_LAST_REG    0x3E    // 마지막 주소 (REGMAP_CRC)
#define ADS130B04_NUM_REGS    (ADS130B04_LAST_REG - ADS130B04_FIRST_REG + 1)

// ------------------------------------------------------
// 함수: ADS130B04_ReadAllRegisters
// 설명: ADS130B04의 레지스터(0x00~0x3E)를 RREG로 한 번에 읽어
//       16-bit 값 배열 `regs[]`에 채워 줍니다.
// 파라미터:
//   dev   : 0 또는 1 (ADC 인스턴스 선택)
//   regs[]: 크기 ADS130B04_NUM_REGS 이상의 uint16_t 배열
// 리턴값:
//   HAL_OK 이면 성공, 그 외 오류 코드
// ------------------------------------------------------
HAL_StatusTypeDef ADS130B04_ReadAllRegisters(uint8_t dev, uint16_t regs[])
{
    HAL_StatusTypeDef ret;
    uint8_t cmd[2];

    cmd[0] = 0xA0 | (ADS130B04_FIRST_REG & 0x1F);
    cmd[1] = (uint8_t)(ADS130B04_NUM_REGS - 1);

    // 2) CS Low → SPI 통신 시작
    HAL_GPIO_WritePin(CS_PORT[dev], CS_PIN[dev], GPIO_PIN_RESET);

    // 3) RREG 명령 전송
    ret = HAL_SPI_Transmit(&hspi3, cmd, 2, HAL_MAX_DELAY);
    if (ret != HAL_OK) {
        HAL_GPIO_WritePin(CS_PORT[dev], CS_PIN[dev], GPIO_PIN_SET);
        return ret;
    }

    // 4) 레지스터 값 수신
    //    각 레지스터는 16bit → 2바이트
    //    총 바이트 = ADS130B04_NUM_REGS * 2
    {
        uint8_t rx_buf[ADS130B04_NUM_REGS * 2];
        ret = HAL_SPI_Receive(&hspi3, rx_buf, sizeof(rx_buf), HAL_MAX_DELAY);
        // 5) CS High → SPI 통신 종료
        HAL_GPIO_WritePin(CS_PORT[dev], CS_PIN[dev], GPIO_PIN_SET);
        if (ret != HAL_OK) {
            return ret;
        }

        // 6) 바이트 배열 → 16-bit 배열로 변환
        for (int i = 0; i < ADS130B04_NUM_REGS; i++) {
            regs[i] = (uint16_t)(rx_buf[2*i] << 8)
                    | (uint16_t)(rx_buf[2*i + 1]);
        }
    }

    return HAL_OK;
}


HAL_StatusTypeDef ADS130B04_Init(void)
{
    HAL_StatusTypeDef ret;
    uint8_t cmd;

    for (int dev = 0; dev < 2; dev++) {
        /* 1) CS Low → SPI 명령 송신 준비 */
        HAL_GPIO_WritePin(CS_PORT[dev], CS_PIN[dev], GPIO_PIN_RESET);

        /* 2) 소프트 리셋
         *    CMD_RESET 전송 → 내부 레지스터가 리셋값으로 복귀
         *    (tPOR ≥ 250 µs 기다림 필요) */
        cmd = CMD_RESET;
        ret = HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
        if (ret != HAL_OK) { goto cleanup; }
        HAL_Delay(1);
        /* 3) SDATAC: 연속 읽기 모드 해제
         *    이후 WREG 명령으로 레지스터 설정 가능 */
        cmd = CMD_SDATAC;
        ret = HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
        if (ret != HAL_OK) { goto cleanup; }
        HAL_Delay(1);

        /* 4-1) WREG @MODE(0x02)~GAIN(0x04)
         *   레지스터 0x02(Mode), 0x03(Clock), 0x04(Gain) 순
         *   • MODE   리셋 0x0510h → 16-bit word 모드로 변경 : 0x0410h
         *       WLENGTH[1:0]=00b → 16bit,
         *       나머지 비트는 리셋값 0x0510h에서 ‘0x0400h’만 바꿈
         *   • CLOCK  리셋 0x0F8Eh → default OSR, 채널 ON 설정 유지
         *   • GAIN   리셋 0x0000h → PGA bypass(×1) 기본 유지 */
        {
            uint8_t wreg1[] = {
                0x50 | REG_MODE,     // WREG + 시작주소 0x02
                (0x04 - 0x02),       // 쓰기 개수-1 = 2 → MODE~GAIN 3개
                /* MODE   */ 0x04, 0x10,
                /* CLOCK  */ 0x0F, 0x8E,
                /* GAIN   */ 0x00, 0x00
            };
            ret = HAL_SPI_Transmit(&hspi3, wreg1, sizeof(wreg1), HAL_MAX_DELAY);
            if (ret != HAL_OK) { goto cleanup; }
        }
        HAL_Delay(1);
        /* 4-2) WREG @GLOBAL_CHOP_CFG(0x06)
         *   리셋값 0x0600h
         *     • GC_DLY[3:0]=0110b(6) → 채핑 딜레이 6샘플
         *     • GC_EN=0 → 채핑 비활성 */
        {
            uint8_t wreg2[] = {
                0x50 | REG_GLOBAL_CHOP_CFG,  // WREG + 0x06
                0x00,                        // 쓰기 개수-1 = 0 → 하나만
                0x06, 0x00                   // 0x0600h
            };
            ret = HAL_SPI_Transmit(&hspi3, wreg2, sizeof(wreg2), HAL_MAX_DELAY);
            if (ret != HAL_OK) { goto cleanup; }
        }
        HAL_Delay(1);

        /* 4-3) WREG @CHn_CFG(0x09,0x0E,0x13,0x18)
         *   리셋값 모두 0x0000h → 각 채널 차동입력(AINnP/N) 기본 */
        {
            uint8_t wreg3[] = {
                0x50 | REG_CH0_CFG,  // WREG + 시작주소 0x09
                0x03,                // 쓰기 개수-1 = 3 → CH0~CH3 4개
                0x00,0x00,           // CH0_CFG = 0x0000h
                0x00,0x00,           // CH1_CFG = 0x0000h
                0x00,0x01,           // CH2_CFG = 0x0000h
                0x00,0x01            // CH3_CFG = 0x0000h
            };
            ret = HAL_SPI_Transmit(&hspi3, wreg3, sizeof(wreg3), HAL_MAX_DELAY);
            if (ret != HAL_OK) { goto cleanup; }
        }
        HAL_Delay(1);

        /* 5) RDATC: cntread 모드 진입 */
        //cmd = CMD_RDATAC;
        //ret = HAL_SPI_Transmit(&hspi3, &cmd, 1, HAL_MAX_DELAY);
        // 1) SDATAC: continuous-read 해제
        uint8_t cmd_sdatac[2] = { 0x00, 0x11 };
        HAL_SPI_Transmit(&hspi3, cmd_sdatac, 2, HAL_MAX_DELAY);
        uint8_t cmd_standby[2] = { 0x00, 0x22 }; // STANDBY
        HAL_GPIO_WritePin(CS_PORT[dev], CS_PIN[dev], GPIO_PIN_RESET);
        HAL_SPI_Transmit(&hspi3, cmd_standby, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(CS_PORT[dev], CS_PIN[dev], GPIO_PIN_SET);
        /* 6) CS High → 초기화 종료 */
    cleanup:
        HAL_GPIO_WritePin(CS_PORT[dev], CS_PIN[dev], GPIO_PIN_SET);
        if (ret != HAL_OK) {
            return ret;
        }
    }
    return HAL_OK;
}

HAL_StatusTypeDef ADS130B04_Read16(int16_t adc_ch, int16_t ch_data[4],  uint8_t *status)
{
    uint8_t rx[6 * 3];  // 6 워드 × 2바이트
    HAL_StatusTypeDef ret;
    // 2) (원할 때마다) RDATA: 단일-샷 변환 & 읽기
    // 1) CS Low
    HAL_GPIO_WritePin(CS_PORT[adc_ch], CS_PIN[adc_ch], GPIO_PIN_RESET);

    // 2) 12바이트 수신 (STATUS + CH0…CH3 + CRC)
    ret = HAL_SPI_Receive(&hspi3, rx, sizeof(rx), HAL_MAX_DELAY);

    // 3) CS High
    HAL_GPIO_WritePin(CS_PORT[adc_ch], CS_PIN[adc_ch], GPIO_PIN_SET);
    if (ret != HAL_OK) return ret;

    // 4) STATUS
    *status = rx[0];

    // 5) 채널 0~3 데이터 파싱 (워드 MSB→LSB)
    for (int i = 0; i < 4; i++) {
        uint16_t raw = (rx[2 + i*2] << 8) | rx[3 + i*2];
        // 부호 확장
        ch_data[i] = (raw & 0x8000) ? (int16_t)raw : (int16_t)raw;
    }

    return HAL_OK;
}

uint8_t g_raw[32];
uint8_t g_raw2[32];
uint32_t g_sumraw[4];
uint32_t g_sumraw2[4];

HAL_StatusTypeDef ADS130B04_Read(void)
{
    //uint8_t rx[6 * 3];  // 6 워드 × 2바이트
    HAL_StatusTypeDef ret;
    HAL_GPIO_WritePin(CS_PORT[0], CS_PIN[0], GPIO_PIN_RESET);
    ret = HAL_SPI_Receive(&hspi3, g_raw, sizeof(g_raw), HAL_MAX_DELAY);
    if (ret != HAL_OK) return ret;

    HAL_GPIO_WritePin(CS_PORT[0], CS_PIN[0], GPIO_PIN_SET);


    return HAL_OK;
}
HAL_StatusTypeDef ADS130B04_2_Read(void)
{
    //uint8_t rx[6 * 3];  // 6 워드 × 2바이트
    HAL_StatusTypeDef ret;
    HAL_GPIO_WritePin(CS_PORT[1], CS_PIN[1], GPIO_PIN_RESET);
    ret = HAL_SPI_Receive(&hspi3, g_raw2, sizeof(g_raw2), HAL_MAX_DELAY);
    HAL_GPIO_WritePin(CS_PORT[1], CS_PIN[1], GPIO_PIN_SET);
    if (ret != HAL_OK) return ret;


    return HAL_OK;
}


HAL_StatusTypeDef DAC8551_Write(uint16_t value, uint8_t ch)
{
    uint8_t  txBuf[3];
    GPIO_TypeDef *cs_port;
    uint16_t      cs_pin;

    /* 1) 제어 바이트: PD1=0, PD0=0 → 정상 모드 */
    txBuf[0] = 0x00;
    /* 2) 데이터 바이트 (MSB → LSB) */
    txBuf[1] = (uint8_t)(value >> 8);
    txBuf[2] = (uint8_t)(value & 0xFF);

    /* 채널별 CS 포트·핀 매핑 */
    switch (ch) {
        case 0:
            cs_port = GPIOB;  cs_pin = DAC_ch_IO_0_Pin;
            break;
        case 1:
            cs_port = GPIOA;  cs_pin = DAC_ch_IO_1_Pin;
            break;
        case 2:
            cs_port = GPIOB;  cs_pin = DAC_ch_IO_2_Pin;
            break;
        case 3:
            cs_port = GPIOA;  cs_pin = DAC_ch_IO_3_Pin;
            break;
        default:
            /* 유효 범위 외 입력 시 무시 */
        	break;
    }
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_RESET);
    HAL_StatusTypeDef ret = HAL_SPI_Transmit(&hspi2, txBuf, 3, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(cs_port, cs_pin, GPIO_PIN_SET);

    return ret;
}
// 5자리 ASCII 로 변환하는 헬퍼
void Fill5Ascii(uint8_t dest[5], uint16_t value) {
    // "%05u" 로 0패딩 5자리 문자열을 얻되 널문자 없이 복사
    char buf[6];
    snprintf(buf, sizeof(buf), "%05u", value);
    memcpy(dest, buf, 5);
}

void init_command(void){
	PC_SEND_data.st = 0x02;
	PC_SEND_data.adress[0] = 'F'; //Floating to Ground
	PC_SEND_data.adress[1] = 'G'; //Floating to Ground
	memcpy((char*)&PC_SEND_data._CRC_[0],(char*)("_CRC_"),5);
	PC_SEND_data.et = 0x03;


}

#define SAMPLE_INTERVAL_MS   100      // 루프 반복 주기
#define REPORT_PERIOD_MS     1000     // 출력 주기
#define AVG_WINDOW_SAMPLES   (REPORT_PERIOD_MS / SAMPLE_INTERVAL_MS)

#define ZE08_FRAME_LEN                9U
#define ZE08_QUERY_PERIOD_MS          1000U
#define ZE08_SENSOR_TIMEOUT_MS        150U

typedef struct
{
    uint16_t concentration_ppb;
    uint16_t full_range_ppb;
    uint8_t gas_id;
    uint8_t unit;
    uint8_t checksum;
    uint32_t last_update_tick;
    uint8_t valid;
}ZE08_Data_t;

static ZE08_Data_t g_ze08_data;
static uint8_t g_ze08_rx_buf[ZE08_FRAME_LEN];

static uint8_t ZE08_CalcChecksum(const uint8_t *frame)
{
    uint16_t sum = 0;
    for (uint8_t i = 1; i < ZE08_FRAME_LEN - 1; i++) {
        sum += frame[i];
    }
    return (uint8_t)(~sum + 1U);
}

static void ZE08_SendCommand(uint8_t cmd, uint8_t data)
{
    uint8_t tx[ZE08_FRAME_LEN] = {0xFF, 0x01, cmd, data, 0x00, 0x00, 0x00, 0x00, 0x00};
    tx[8] = ZE08_CalcChecksum(tx);
    HAL_UART_Transmit(&huart1, tx, ZE08_FRAME_LEN, 100);
}

static HAL_StatusTypeDef ZE08_ReadFrame(uint8_t *frame)
{
    return HAL_UART_Receive(&huart1, frame, ZE08_FRAME_LEN, ZE08_SENSOR_TIMEOUT_MS);
}

static uint8_t ZE08_ParseFrame(const uint8_t *frame, ZE08_Data_t *out)
{
    if ((frame[0] != 0xFFU) || (ZE08_CalcChecksum(frame) != frame[8])) {
        return 0;
    }

    out->gas_id = frame[1];
    out->unit = frame[2];
    out->concentration_ppb = (uint16_t)(((uint16_t)frame[4] << 8) | frame[5]);
    out->full_range_ppb = (uint16_t)(((uint16_t)frame[6] << 8) | frame[7]);
    out->checksum = frame[8];
    out->last_update_tick = HAL_GetTick();
    out->valid = 1;

    return 1;
}

static void ZE08_Init(void)
{
    memset(&g_ze08_data, 0, sizeof(g_ze08_data));
    /* 0x78: 모드 전환, 0x40: 질의응답(QA) 모드 */
    ZE08_SendCommand(0x78, 0x40);
}

static void ZE08_PollAndUpdate(void)
{
    ZE08_SendCommand(0x86, 0x00);
    if ((ZE08_ReadFrame(g_ze08_rx_buf) == HAL_OK) && ZE08_ParseFrame(g_ze08_rx_buf, &g_ze08_data)) {
        Fill5Ascii(PC_SEND_data.AD0, g_ze08_data.concentration_ppb);
        Fill5Ascii(PC_SEND_data.AD1, g_ze08_data.full_range_ppb);
    }
}
//#define TT
void pGC_main(void)
{
	//initailze();														//Variable initialize
	init_command();
	ADS130B04_Init();
	ZE08_Init();
	/* Loop 			--------------------------------------------------*/
	while(1){
		g_sys_tick = 1;													//Delay start
		while(g_sys_tick);
		if(g_rcv_flagb){ ReadCheckCommand(); }

		if ((HAL_GetTick() - g_ze08_data.last_update_tick) >= ZE08_QUERY_PERIOD_MS) {
			ZE08_PollAndUpdate();
		}

		switch(g_sys_tick_count){
			case 5 :
			    for (int i = 0; i < DA_CHANNELS; i++) {
			        if (F_flag[i]) {
			            F_flag[i]      = 0;
			            F_value[i]     = char2dec(F_da_buf[i], DA_BUF_LEN);
			            DAC8551_Write(F_value[i], i);
			            if (DAC8551_Write(F_value[i], i) != HAL_OK) {
			                // 오류 처리 (재시도, 로깅 등)
			            }
#ifdef TT
			            ePrintf("%u",F_value[i]);
#endif
			        }
			    }
			    break;

			case 10 :	//Monitoring ADC read protocol
				ADS130B04_Read();
				for (int i = 0; i < 4; i++) {
					uint16_t msb    = (uint16_t)g_raw[3 + 3*i];
					uint16_t mid    = (uint16_t)g_raw[3 + 3*i + 1];
					uint16_t raw16  = (msb << 8) | mid;       // 상위 16비트만
					//uint16_t code15 = 0x7FFF - raw16;         // 0…32767
					g_sumraw[i] = raw16;
				}
			    Fill5Ascii(PC_SEND_data.AD0, g_sumraw[0]);
			    Fill5Ascii(PC_SEND_data.AD1, g_sumraw[1]);
			    Fill5Ascii(PC_SEND_data.AD2, g_sumraw[2]);
			    Fill5Ascii(PC_SEND_data.AD3, g_sumraw[3]);
				break;

			case 40 :	//Monitoring ADC read protocol
				ADS130B04_2_Read();
				for (int i = 0; i < 4; i++) {
					uint16_t msb    = (uint16_t)g_raw2[3 + 3*i];
					uint16_t mid    = (uint16_t)g_raw2[3 + 3*i + 1];
					uint16_t raw16  = (msb << 8) | mid;       // 상위 16비트만
					g_sumraw2[i] = raw16;
				}
			    Fill5Ascii(PC_SEND_data.AD4, g_sumraw2[0]);
			    Fill5Ascii(PC_SEND_data.AD5, g_sumraw2[1]);
			    Fill5Ascii(PC_SEND_data.AD6, g_sumraw2[2]);
			    Fill5Ascii(PC_SEND_data.AD7, g_sumraw2[3]);
				break;


			case 70 : //Monitoring SEND protocol
				if(send_1s_flag){
					send_1s_flag=0;
#ifdef TT
					send_to_pc((char*)&PC_SEND_data,sizeof(PC_SEND_data_struct));
#endif
					send_to_gnd((char*)&PC_SEND_data,sizeof(PC_SEND_data_struct));
					//send_to_pc_inverted((char*)&PC_SEND_data,sizeof(PC_SEND_data_struct));
					//send_to_pc_inverted((char*)&PC_SEND_data,sizeof(PC_SEND_data_struct));
				}
				break;

			default :
				break;
		}
		/* 1 ms Timing end				--------------------------------------------------*/
	}
}
