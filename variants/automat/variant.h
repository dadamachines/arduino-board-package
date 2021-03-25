#pragma once

#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610
#define VARIANT_MAINOSC (32768ul)
#define VARIANT_MCK (48000000ul)

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define PINS_COUNT (26u)
#define NUM_DIGITAL_PINS (20u)
#define NUM_ANALOG_INPUTS (6u)
#define NUM_ANALOG_OUTPUTS (1u)
#define analogInputToDigitalPin(p) ((p < 6u) ? (p) + 14u : -1)
#define digitalPinToPort(P) (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P) (1 << g_APinDescription[P].ulPin)
#define portOutputRegister(port) (&(port->OUT.reg))
#define portInputRegister(port) (&(port->IN.reg))
#define portModeRegister(port) (&(port->DIR.reg))
#define digitalPinHasPWM(P) (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)

#define PIN_LED_13 (13u)
#define PIN_LED_RXL (25u)
#define PIN_LED_TXL (26u)
#define PIN_LED PIN_LED_13
#define PIN_LED2 PIN_LED_RXL
#define PIN_LED3 PIN_LED_TXL
#define LED_BUILTIN PIN_LED_13

#define PIN_A0 (14ul)
#define PIN_A1 (15ul)
#define PIN_A2 (16ul)
#define PIN_A3 (17ul)
#define PIN_A4 (18ul)
#define PIN_A5 (19ul)
#define PIN_DAC0 (14ul)

static const uint8_t A0   = PIN_A0;
static const uint8_t A1   = PIN_A1;
static const uint8_t A2   = PIN_A2;
static const uint8_t A3   = PIN_A3;
static const uint8_t A4   = PIN_A4;
static const uint8_t A5   = PIN_A5;
static const uint8_t DAC0 = PIN_DAC0;
#define ADC_RESOLUTION 12

#define PIN_ATN (38ul)
static const uint8_t ATN = PIN_ATN;

#define PIN_SERIAL_RX (31ul)
#define PIN_SERIAL_TX (30ul)
#define PAD_SERIAL_TX (UART_TX_PAD_2)
#define PAD_SERIAL_RX (SERCOM_RX_PAD_3)

#define PIN_SERIAL1_RX (0ul)
#define PIN_SERIAL1_TX (1ul)
#define PAD_SERIAL1_TX (UART_TX_PAD_2)
#define PAD_SERIAL1_RX (SERCOM_RX_PAD_3)

#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_MISO (22u)
#define PIN_SPI_MOSI (23u)
#define PIN_SPI_SCK (24u)
#define PERIPH_SPI sercom4
#define PAD_SPI_TX SPI_PAD_2_SCK_3
#define PAD_SPI_RX SERCOM_RX_PAD_0

static const uint8_t SS   = PIN_A2;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA (20u)
#define PIN_WIRE_SCL (21u)
#define PERIPH_WIRE sercom3
#define WIRE_IT_HANDLER SERCOM3_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_USB_HOST_ENABLE (-1)
#define PIN_USB_DM (28ul)
#define PIN_USB_DP (29ul)

#define I2S_INTERFACES_COUNT 1

#define I2S_DEVICE 0
#define I2S_CLOCK_GENERATOR 3
#define PIN_I2S_SD (9u)
#define PIN_I2S_SCK (1u)
#define PIN_I2S_FS (0u)

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus
extern SERCOM sercom0;
extern SERCOM sercom1;
extern SERCOM sercom2;
extern SERCOM sercom3;
extern SERCOM sercom4;
extern SERCOM sercom5;
extern Uart Serial1;
#endif

#define SERIAL_PORT_USBVIRTUAL SerialUSB
#define SERIAL_PORT_MONITOR Serial
#define SERIAL_PORT_HARDWARE Serial1
#define SERIAL_PORT_HARDWARE_OPEN Serial1
