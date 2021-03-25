#pragma once

#define ARDUINO_SAMD_VARIANT_COMPLIANCE 10610
#define VARIANT_MAINOSC (32768ul)
#define VARIANT_MCK (120000000ul)
#define VARIANT_GCLK0_FREQ (120000000UL)
#define VARIANT_GCLK1_FREQ (48000000UL)
#define VARIANT_GCLK2_FREQ (100000000UL)

#include "WVariant.h"

#ifdef __cplusplus
#include "SERCOM.h"
#include "Uart.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define NUM_DIGITAL_PINS (27u)
#define NUM_ANALOG_INPUTS (10u)
#define NUM_ANALOG_OUTPUTS (1u)
#define analogInputToDigitalPin(p) ((p < NUM_ANALOG_INPUTS) ? (p) + PIN_A0 : -1)
#define digitalPinToPort(P) (&(PORT->Group[g_APinDescription[P].ulPort]))
#define digitalPinToBitMask(P) (1 << g_APinDescription[P].ulPin)
#define portOutputRegister(port) (&(port->OUT.reg))
#define portInputRegister(port) (&(port->IN.reg))
#define portModeRegister(port) (&(port->DIR.reg))
#define digitalPinHasPWM(P) (g_APinDescription[P].ulPWMChannel != NOT_ON_PWM || g_APinDescription[P].ulTCChannel != NOT_ON_TIMER)

#define PIN_LED_33 (33u)
#define PIN_LED PIN_LED_33
#define LED_BUILTIN PIN_LED_33

#define ICE_CLK 21
#define ICE_MOSI 22
#define ICE_MISO 23
#define ICE_CS 24
#define ICE_CDONE 25
#define ICE_CRESET 26

#define PIN_A0 (10ul)
#define PIN_A1 (PIN_A0 + 1)
#define PIN_A2 (PIN_A0 + 2)
#define PIN_A3 (PIN_A0 + 3)
#define PIN_A4 (PIN_A0 + 4)
#define PIN_A5 (PIN_A0 + 5)
#define PIN_A6 (PIN_A0 + 6)
#define PIN_A7 (PIN_A0 + 7)
#define PIN_A8 (PIN_A0 + 8)
#define PIN_A9 (PIN_A0 + 9)
#define PIN_A10 (PIN_A0 + 10)

#define PIN_DAC0 34
#define PIN_DAC1 35

static const uint8_t A0  = PIN_A0;
static const uint8_t A1  = PIN_A1;
static const uint8_t A2  = PIN_A2;
static const uint8_t A3  = PIN_A3;
static const uint8_t A4  = PIN_A4;
static const uint8_t A5  = PIN_A5;
static const uint8_t A6  = PIN_A6;
static const uint8_t A7  = PIN_A7;
static const uint8_t A8  = PIN_A8;
static const uint8_t A9  = PIN_A9;
static const uint8_t A10 = PIN_A10;

static const uint8_t DAC0 = PIN_DAC0;
static const uint8_t DAC1 = PIN_DAC1;

#define ADC_RESOLUTION 12

#define PIN_ATN (26ul)
static const uint8_t ATN = PIN_ATN;

#define PIN_SERIAL1_RX (0ul)
#define PIN_SERIAL1_TX (1ul)
#define PAD_SERIAL1_RX (SERCOM_RX_PAD_1)
#define PAD_SERIAL1_TX (UART_TX_PAD_0)

#define SPI_INTERFACES_COUNT 1
#define PIN_SPI_CS (5u)
#define PIN_SPI_MISO (2u)
#define PIN_SPI_SCK (12u)
#define PIN_SPI_MOSI (11u)
#define PERIPH_SPI sercom4
#define PAD_SPI_TX SPI_PAD_0_SCK_1
#define PAD_SPI_RX SERCOM_RX_PAD_3
static const uint8_t SS   = PIN_A2;
static const uint8_t MOSI = PIN_SPI_MOSI;
static const uint8_t MISO = PIN_SPI_MISO;
static const uint8_t SCK  = PIN_SPI_SCK;

#define WIRE_INTERFACES_COUNT 1
#define PIN_WIRE_SDA (1u)
#define PIN_WIRE_SCL (0u)
#define PERIPH_WIRE sercom2
#define WIRE_IT_HANDLER SERCOM2_Handler

static const uint8_t SDA = PIN_WIRE_SDA;
static const uint8_t SCL = PIN_WIRE_SCL;

#define PIN_USB_HOST_ENABLE (27ul)
#define PIN_USB_DM (28ul)
#define PIN_USB_DP (29ul)

#define I2S_INTERFACES_COUNT 0
#define I2S_DEVICE 0

#define PIN_QSPI_SCK (5u)
#define PIN_QSPI_CS (2u)
#define PIN_QSPI_IO0 (17u)
#define PIN_QSPI_IO1 (18u)
#define PIN_QSPI_IO2 (19u)
#define PIN_QSPI_IO3 (20u)
#define VARIANT_QSPI_BAUD_DEFAULT 5000000

void dacInit();
void dacWrite(uint16_t left, uint16_t right);

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

#define SERIAL_PORT_USBVIRTUAL Serial
#define SERIAL_PORT_MONITOR Serial
#define SERIAL_PORT_HARDWARE Serial1
#define SERIAL_PORT_HARDWARE_OPEN Serial1
