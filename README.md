# embedded_STM32_project

STM32 NUCLEO-F446 기반 공기질 센서 프로젝트입니다.

## ZE08-CH2O 연동
- 센서 UART를 `USART1`에 연결합니다.
  - `PA9`  : TX (MCU -> ZE08)
  - `PA10` : RX (ZE08 -> MCU)
- `pGC_main.c`에 ZE08 질의응답(QA) 모드 전환(`0x78 0x40`) 및 주기적 농도 조회(`0x86`) 로직을 추가했습니다.

## Python GUI
`ze08_uart_gui.py`는 PC에서 ZE08 센서에 UART 명령을 보내고 응답을 파싱하는 간단한 GUI입니다.

### 실행
```bash
python ze08_uart_gui.py
```

### 필요 패키지
```bash
pip install pyserial
```
