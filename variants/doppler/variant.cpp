// dadamachines - doppler

#include "variant.h"

const PinDescription g_APinDescription[] = {
  [0]  = {PORTA, 13, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, TC2_CH1, EXTERNAL_INT_13},            // D0  SERCOM 2.1 SCL
  [1]  = {PORTA, 12, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, TC2_CH0, EXTERNAL_INT_12},            // D1  SERCOM 2.0 SDA
  [2]  = {PORTB, 11, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11},             // D2  QSPI CS
  [3]  = {PORTA, 14, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC2_CH0, TC3_CH0, EXTERNAL_INT_14},               // D3
  [4]  = {PORTA, 15, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC2_CH1, TC3_CH1, EXTERNAL_INT_15},               // D4
  [5]  = {PORTB, 10, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10},             // D5  QSPI SCK
  [6]  = {PORTA, 19, PIO_SERCOM_ALT, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH3, TC3_CH1, EXTERNAL_INT_3},             // D6  SHARED FPGA sercom5 SPI
  [7]  = {PORTA, 20, PIO_SERCOM_ALT, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH0, NOT_ON_TIMER, EXTERNAL_INT_4},        // D7  SHARED FPGA sercom5 SPI
  [8]  = {PORTA, 21, PIO_SERCOM_ALT, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH1, NOT_ON_TIMER, EXTERNAL_INT_5},        // D8  SHARED FPGA sercom5 SPI
  [9]  = {PORTA, 22, PIO_SERCOM, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH2, NOT_ON_TIMER, EXTERNAL_INT_6},            // D9  SHARED FPGA sercom5 SPI
  [10] = {PORTA, 2, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2},            // A0 DAC0
  [11] = {PORTB, 8, PIO_SERCOM_ALT, PIN_ATTR_ANALOG, ADC_Channel2, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8},        // A1 MOSI
  [12] = {PORTB, 9, PIO_SERCOM_ALT, PIN_ATTR_ANALOG, ADC_Channel3, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9},        // A2 SCK
  [13] = {PORTA, 4, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel4, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_4},            // A3
  [14] = {PORTA, 5, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel5, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5},            // A4 DAC1
  [15] = {PORTA, 6, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel6, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6},            // A5
  [16] = {PORTA, 7, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel7, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7},            // A6
  [17] = {PORTA, 8, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel8, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_NMI},          // A7  QSPI D0
  [18] = {PORTA, 9, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel9, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9},            // A8  QSPI D1
  [19] = {PORTA, 10, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel10, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_10},         // A9  QSPI D2
  [20] = {PORTA, 11, PIO_ANALOG, PIN_ATTR_ANALOG, ADC_Channel11, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11},         // A10 QSPI D3
  [21] = {PORTB, 3, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3},         // 21 ICE40_CLK
  [22] = {PORTB, 23, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_7},        // 22 ICE40_MOSI
  [23] = {PORTB, 2, PIO_SERCOM, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2},         // 23 ICE40_MISO
  [24] = {PORTB, 22, PIO_SERCOM_ALT, PIN_ATTR_DIGITAL, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_6},    // 24 ICE40_CS
  [25] = {PORTA, 16, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH0, TC2_CH0, EXTERNAL_INT_0},                // 25 ICE40_CDONE
  [26] = {PORTA, 17, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH1, TC2_CH1, EXTERNAL_INT_1},                // 26 ICE40_RESET
  [27] = {PORTA, 27, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_11},             // USB Host enable
  [28] = {PORTA, 24, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_8},              // USB/DM
  [29] = {PORTA, 25, PIO_COM, PIN_ATTR_NONE, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_9},              // USB/DP
  [30] = {PORTA, 3, PIO_ANALOG, PIN_ATTR_ANALOG, No_ADC_Channel, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_3},          // DAC/VREFP
  [31] = {PORTA, 1, PIO_SERCOM_ALT, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH1, TC2_CH1, EXTERNAL_INT_1},               // SCK  SERCOM 1.1
  [32] = {PORTA, 0, PIO_SERCOM_ALT, PIN_ATTR_PWM_E, No_ADC_Channel, TC2_CH0, TC2_CH0, EXTERNAL_INT_0},               // MOSI SERCOM 1.0
  [33] = {PORTA, 23, PIO_DIGITAL, PIN_ATTR_PWM_G, No_ADC_Channel, TCC0_CH3, NOT_ON_TIMER, EXTERNAL_INT_7},           //
  [34] = {PORTA, 2, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel0, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_2},            // DAC/VOUT[0]
  [35] = {PORTA, 5, PIO_ANALOG, PIN_ATTR_ANALOG, DAC_Channel1, NOT_ON_PWM, NOT_ON_TIMER, EXTERNAL_INT_5},            // DAC/VOUT[1]
  [36] = {PORTA, 7, PIO_ANALOG, (PIN_ATTR_ANALOG | PIN_ATTR_PWM_E), ADC_Channel7, TC1_CH1, TC1_CH1, EXTERNAL_INT_7}, //
  [37] = {PORTA, 18, PIO_DIGITAL, PIN_ATTR_PWM_F, No_ADC_Channel, TCC1_CH2, TC3_CH0, EXTERNAL_INT_2}                 //
};

const void *g_apTCInstances[TCC_INST_NUM + TC_INST_NUM] = {TCC0, TCC1, TCC2};

SERCOM sercom0(SERCOM0);
SERCOM sercom1(SERCOM1);
SERCOM sercom2(SERCOM2);
SERCOM sercom3(SERCOM3);
SERCOM sercom4(SERCOM4);
SERCOM sercom5(SERCOM5);

Uart Serial1(&sercom3, PIN_SERIAL1_RX, PIN_SERIAL1_TX, PAD_SERIAL1_RX, PAD_SERIAL1_TX);
void SERCOM3_0_Handler() {
  Serial1.IrqHandler();
}

void SERCOM3_1_Handler() {
  Serial1.IrqHandler();
}

void SERCOM3_2_Handler() {
  Serial1.IrqHandler();
}

void SERCOM3_3_Handler() {
  Serial1.IrqHandler();
}