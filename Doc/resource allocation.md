# Resource Allocation

## 功能

序号 | 功能               | MCU资源   
---- | ------------------ | ------------
 1   | 电压采样 V         | ADC 
 2   | 电流采样 A         | ADC
 3   | 推力采样1 g        | SCL SDA
 4   | 推力采样2 g        | SCL SDA
 5   | 推力采样3 g        | SCL SDA
 6   | 油门输出 %         | PWM Output
 7   | 转速采样 rpm/min   | GPIO Input
 8   | 振动采样 g         | I2C
 9   | 与上位机通信        | UART

## STM32F103C8T6 - 20180822

~~~
Pin Function                                      Definition
1   VBAT                                          -
2   PC13/TAMPER-RTC                               LED_OUT
3   PC14/OSC32_IN                                 -
4   PC15/OSC32_OUT                                -
5   OSC_IN/CANRX                                  OSC_IN
6   OSC_OUT/CANTX                                 OSC_OUT
7   NRST                                          NRST
8   VSSA                                          GND
9   VDDA                                          VCC
10  PA0/WKUP/USART2_CTS/ADC12_IN0/TIM2_CH1_ETR    -
11  PA1/USART2_RTS/ADC12_IN1/TIM2_CH2             -
12  PA2/USART2_TX/ADC12_IN2/TIM2_CH3              -
13  PA3/USART2_RX/ADC12_IN3/TIM2_CH4              -
14  PA4/SPI1_NSS/USART2_CK/ADC12_IN4              -
15  PA5/SPI1_SCK/ADC12_IN5                        -
16  PA6/SPI1_MISO/ADC12_IN6/TIM3_CH1/TIM1_BKIN    VOL_AD
17  PA7/SPI1_MOSI/ADC12_IN7/TIM3_CH2/TIM1_CH1N    CUR_AD
18  PB0/ADC12_IN8/TIM3_CH3/TIM1_CH2N              -
19  PB1/ADC12_IN9/TIM3_CH4/TIM1_CH3N              DEBUG_OUT
20  PB2/BOOT1                                     BOOT1
21  PB10/I2C2_SCL/USART3_TX/TIM2_CH3              PRINT_TX
22  PB11/I2C2_SDA/USART3_RX/TIM2_CH4              PRINT_RX
23  VSS_1                                         GND
24  VDD_1                                         VCC
25  PB12/SPI2_NSS/I2C2_SMBAI/USART3_CK/TIM1_BKIN  RPM_IN
26  PB13/SPI2_SCK/USART3_CTS/TIM1_CH1N            HX711_SDA1
27  PB14/SPI2_MISO/USART3_RTS/TIM1_CH2N           HX711_SCL1
28  PB15/SPI2_MOSI/TIM1_CH3N                      -
29  PA8/USART1_CK/TIM1_CH1/MCO                    PWM_OUT
30  PA9/USART1_TX/TIM1_CH2                        -                 
31  PA10/USART1_RX/TIM1_CH3                       -
32  PA11/USART1_CTS/CANRX/USBDM/TIM1_CH4          -            
33  PA12/USART1_RTS/CANTX/USBDP/TIM1_ETR          -              
34  PA13/JTMS/SWDIO                               SWDIO
35  VSS_2                                         GND
36  VDD_2                                         VCC
37  PA14/JTCK/SWCLK                               SWCLK
38  PA15/JTDI/TIM2_CH1_ETR/SPI1_NSS               HX711_SDA2
39  PB3/JTDO/TIM2_CH2/TRACESWO/SPI1_SCK           HX711_SCL2
40  PB4/JNTRST/TIM3_CH1/SPI1_MISO                 -
41  PB5/I2C1_SMBAI/TIM3_CH2/SPI1_MOSI             MPU6050_INT
42  PB6/I2C1_SCL/TIM4_CH1/USART1_TX               MPU6050_SCL   
43  PB7/I2C1_SDA/TIM4_CH2/USART1_RX               MPU6050_SDA  
44  BOOT0                                         BOOT0
45  PB8/TIM4_CH3/I2C1_SCL/CANRX                   HX711_SDA3
46  PB9/TIM4_CH4/I2C1_SDA/CANTX                   HX711_SCL3  
47  VSS_3                                         GND
48  VDD_3                                         VCC
~~~
