# STM32F411RE Autonomous RC Car

STM32F411RE와 FreeRTOS를 기반으로 제작한 자율주행 RC Car 프로젝트

3개의 초음파 센서를 이용하여 전방 및 좌우 장애물을 감지하고, 장애물 상황에 따라 주행 방향을 결정
HC-06 Bluetooth 모듈을 통해 수동 주행과 자율주행 모드를 전환 가능



## Development Environment

- STM32F411RE
- STM32 HAL
- FreeRTOS
- STM32CubeIDE



## 주요 기능

* FreeRTOS 기반 멀티태스킹
* Bluetooth를 이용
* Manual / Autonomous 주행 모드 전환
* 좌 / 중앙 / 우측 초음파 센서를 이용한 거리 측정
* 전방 장애물 감지 및 회피
* L298N 모터 드라이버 PWM 속도 제어



## Hardware

| Component         | Description |
| ----------------- | ----------- |
| MCU               | STM32F411RE |
| Motor Driver      | L298N       |
| DC Motor          | 4개          |
| Ultrasonic Sensor | 3개          |
| Bluetooth Module  | HC-06       |
| RTOS              | FreeRTOS    |



## Pin Mapping

| 구분                | 기능              | STM32 Pin | 설정             | 연결 대상                  |
| ----------------- | --------------- | --------- | -------------- | ---------------------- |
| Motor             | Left Motor PWM  | PB3       | TIM2_CH2 / PWM | L298N Left Enable      |
| Motor             | Right Motor PWM | PB10      | TIM2_CH3 / PWM | L298N Right Enable     |
| Motor             | Left IN1        | PB5       | GPIO Output    | L298N Left Direction   |
| Motor             | Left IN2        | PA8       | GPIO Output    | L298N Left Direction   |
| Motor             | Right IN1       | PA9       | GPIO Output    | L298N Right Direction  |
| Motor             | Right IN2       | PB6       | GPIO Output    | L298N Right Direction  |
| Ultrasonic Left   | TRIG            | PC0       | GPIO Output    | Left Ultrasonic TRIG   |
| Ultrasonic Left   | ECHO            | PC6       | EXTI Input     | Left Ultrasonic ECHO   |
| Ultrasonic Center | TRIG            | PC1       | GPIO Output    | Center Ultrasonic TRIG |
| Ultrasonic Center | ECHO            | PC7       | EXTI Input     | Center Ultrasonic ECHO |
| Ultrasonic Right  | TRIG            | PC2       | GPIO Output    | Right Ultrasonic TRIG  |
| Ultrasonic Right  | ECHO            | PC8       | EXTI Input     | Right Ultrasonic ECHO  |
| Bluetooth         | USART1 TX       | PA15      | USART1_TX      | HC-06 RX               |
| Bluetooth         | USART1 RX       | PA10      | USART1_RX      | HC-06 TX               |
| Debug             | USART2 TX       | PA2       | USART2_TX      | Serial Output          |
| Debug             | USART2 RX       | PA3       | USART2_RX      | PC → STM32             |



## Operating Modes

### Manual Mode

Bluetooth를 통해 사용자가 직접 차량을 조작

| Command | 동작        |
| ------- | ----------- |
|  M, m   | Manual Mode |
|  W, w   | Forward     |
|  S, s   | Backward    |
|  A, a   | Left        |
|  D, d   | Right       |
|  X, x   | Stop        |



### Autonomous Mode

`U` 명령을 입력하면 Autonomous Mode가 시작

차량은 좌 / 중앙 / 우측 초음파 센서 데이터를 이용하여 장애물을 감지하고 주행 방향을 결정

| Command | 동작                         |
| ------- | ---------------------------- |
|    U    | Autonomous Mode 시작 / 재시작 |
|    X    | Stop                         |
|    M    | Manual Mode 전환             |



## Autonomous Driving Logic

자율주행은 상태 기반으로 동작

STOPPED
   |
   v
DRIVE
   |
   | Front obstacle detected
   v
STOP_BEFORE_TURN
   |
   +--------------+
   |              |
   v              v
PIVOT_LEFT    PIVOT_RIGHT
   |              |
   +------+-------+
          |
          v
 STOP_AFTER_TURN
          |
          v
        DRIVE



## FreeRTOS Architecture

### ControlTask

차량의 전체 주행 제어

- Bluetooth 명령 처리
- Manual / Autonomous Mode 관리
- 모터 제어
- 자율주행 State Machine 실행
- 최신 초음파 센서 데이터 기반 장애물 회피

### UltrasonicTask

3개의 초음파 센서 관리

- Left / Center / Right 거리 측정
- 센서 데이터 유효성 확인
- Timeout 관리
- 측정 데이터의 갱신 시간 관리
- 측정 결과를 Message Queue를 통해 ControlTask로 전달

### BluetoothTask

HC-06에서 USART1을 통해 명령을 수신

명령:
W / S / A / D
M / U / X

수신된 명령은 Message Queue를 통해 ControlTask로 전달

### LoggerTask

USART2를 이용하여 초음파 센서 측정값을 출력



## Software Structure

Core/
├── Inc/
│   ├── FreeRTOSConfig.h
│   ├── gpio.h
│   ├── main.h
│   ├── move.h
│   ├── pwm.h
│   ├── sensor_data.h
│   ├── stm32f4xx_hal_conf.h
│   ├── stm32f4xx_it.h
│   ├── tim.h
│   ├── ultrasonic.h
│   └── usart.h
│
└── Src/
    ├── freertos.c
    ├── gpio.c
    ├── main.c
    ├── move.c
    ├── pwm.c
    ├── stm32f4xx_hal_msp.c
    ├── stm32f4xx_hal_timebase_tim.c
    ├── stm32f4xx_it.c
    ├── syscalls.c
    ├── sysmem.c
    ├── system_stm32f4xx.c
    ├── tim.c
    ├── ultrasonic.c
    └── usart.c


