/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "variant.h"

/*
 * DUE Board pin   |  PORT  | Label
 * ----------------+--------+-------
 *   0             |  PA8   | "RX0"
 *   1             |  PA9   | "TX0"
 *   2       TIOA0 |  PB25  |
 *   3       TIOA7 |  PC28  |
 *   4       NPCS1 |  PA29  |
 *           TIOB6 |  PC26  |
 *   5       TIOA6 |  PC25  |
 *   6       PWML7 |  PC24  |
 *   7       PWML6 |  PC23  |
 *   8       PWML5 |  PC22  |
 *   9       PWML4 |  PC21  |
 *  10       NPCS0 |  PA28  |
 *           TIOB7 |  PC29  |
 *  11       TIOA8 |  PD7   |
 *  12       TIOB8 |  PD8   |
 *  13       TIOB0 |  PB27  | LED AMBER "L"
 *  14       TXD3  |  PD4   | "TX3"
 *  15       RXD3  |  PD5   | "RX3"
 *  16       TXD1  |  PA13  | "TX2"
 *  17       RXD1  |  PA12  | "RX2"
 *  18       TXD0  |  PA11  | "TX1"
 *  19       RXD0  |  PA10  | "RX1"
 *  20             |  PB12  | "SDA"
 *  21             |  PB13  | "SCL"
 *  22             |  PB26  |
 *  23             |  PA14  |
 *  24             |  PA15  |
 *  25             |  PD0   |
 *  26             |  PD1   |
 *  27             |  PD2   |
 *  28             |  PD3   |
 *  29             |  PD6   |
 *  30             |  PD9   |
 *  31             |  PA7   |
 *  32             |  PD10  |
 *  33             |  PC1   |
 *  34             |  PC2   |
 *  35             |  PC3   |
 *  36             |  PC4   |
 *  37             |  PC5   |
 *  38             |  PC6   |
 *  39             |  PC7   |
 *  40             |  PC8   |
 *  41             |  PC9   |
 *  42             |  PA19  |
 *  43             |  PA20  |
 *  44             |  PC19  |
 *  45             |  PC18  |
 *  46             |  PC17  |
 *  47             |  PC16  |
 *  48             |  PC15  |
 *  49             |  PC14  |
 *  50             |  PC13  |
 *  51             |  PC12  |
 *  52       NPCS2 |  PB21  |
 *  53             |  PB14  |
 *  54             |  PA16  | "A0"
 *  55             |  PA24  | "A1"
 *  56             |  PA23  | "A2"
 *  57             |  PA22  | "A3"
 *  58       TIOB2 |  PA6   | "A4"
 *  69             |  PA4   | "A5"
 *  60       TIOB1 |  PA3   | "A6"
 *  61       TIOA1 |  PA2   | "A7"
 *  62             |  PB17  | "A8"
 *  63             |  PB18  | "A9"
 *  64             |  PB19  | "A10"
 *  65             |  PB20  | "A11"
 *  66             |  PB15  | "DAC0"
 *  67             |  PB16  | "DAC1"
 *  68             |  PA1   | "CANRX"
 *  69             |  PA0   | "CANTX"
 *  70             |  PA17  | "SDA1"
 *  71             |  PA18  | "SCL1"
 *  72             |  PC30  | LED AMBER "RX"
 *  73             |  PA21  | LED AMBER "TX"
 *  74       MISO  |  PA25  |
 *  75       MOSI  |  PA26  |
 *  76       SCLK  |  PA27  |
 *  77       NPCS0 |  PA28  |
 *  78       NPCS3 |  PB23  | unconnected!
 *
 * USB pin         |  PORT
 * ----------------+--------
 *  ID             |  PB11
 *  VBOF           |  PB10
 *
 */

/*
D00	PA5	RXD0	NPCS3	WKUP4
D01	PA6	TXD0	PCK0
D02	PA24	RTS1	PWMH1	A20	PIODC0
D03	PA25	CTS1	PWMH2	A23	PIODC1
D04	PA10	UTXD0	NPCS2
D05	PA7	RTS0	PWMH3	XIN32
D06	PA8	CTS0	ADTRG	WKUP5	XOUT32
D07	PA9	URXD0	NPCS1	PWMFI0	WKUP6
D08	PA11	NPCS0	PWMH0	WKUP7
D09	PA26	DCD1	TIOA2	MCDA2	PIODC2
D10	PA23	SCK1	PWMH0	A19	PIODCCLK
D11	PA13	MOSI	PWMH2
D12	PA12	MISO	PWMH1
D13	PA14	SPCK	PWMH3	WKUP8
A00	PB0	PWMH0	AD4
A01	PB1	PWMH1	AD5
A02	PB2	URXD1	NPCS2	AD6/	WKUP12
A03	PB3	UTXD1	PCK2	AD7
A04	PA17	TD	PCK1	PWMH3	AD0
A05	PA18	RD	PCK2	A14	AD1
A06	PA19	RK	PWML0	A15	AD2/WKUP9
A07	PA20	RF	PWML1	A16	AD3/WKUP10
	PA0	PWMH0	TIOA0	A17	WKUP0
	PA15	TF	TIOA1	PWML3	WKUP14/PIODCEN1
	PA16	TK	TIOB1	PWML2	WKUP15/PIODCEN2
	PA1	PWMH1	TIOB0	A18	WKUP1
	PA21	RXD1	PCK1	AD8
	PA22	TXD1	NPCS3	NCS2	AD9
	PA27	DTR1	TIOB2	MCDA3	PIODC3
	PA28	DSR1	TCLK1	MCCDA	PIODC4
	PA29	RI1	TCLK2	MCCK	PIODC5
	PA2	PWMH2	SCK0	DATRG	WKUP2
	PA30	PWML2	NPCS2	MCDA0	WKUP11/PIODC6
	PA31	NPCS1	PCK2	MCDA1	PIODC7
	PA3	TWD0	NPCS3
	PA4	TWCK0	TCLK0	WKUP3
	PB10	DDM
	PB11	DDP
	PB12	PWML1	ERASE
	PB13	PWML2	PCK0	DAC0
	PB14	NPCS1	PWMH3	DAC1
	PB4	TWD1	PWMH2	TDI
	PB5	TWCK1	PWML0	WKUP13	TDO/TRACESWO
	PB6	TMS/SWDIO
	PB7	TCK/SWCLK
	PB8	XOUT
	PB9	XIN
*/

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Pins descriptions
 */
extern const PinDescription g_APinDescription[]=
{
  // 0 .. 13 - Digital pins
  // ----------------------
  // 0/1 - UART (Serial)
  { PIOA, PIO_PA5,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // RX
  { PIOA, PIO_PA6,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // TX

  // 2
  { PIOA, PIO_PA24, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // D2
  { PIOA, PIO_PA25, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC,    PWM_CH2, NOT_ON_TIMER }, // D3
  { PIOA, PIO_PA10, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // D4
  { PIOA, PIO_PA7,  ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC,    PWM_CH3, NOT_ON_TIMER }, // D5
  { PIOA, PIO_PA8,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // D6
  { PIOA, PIO_PA9,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // D7
  { PIOA, PIO_PA11, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // D8
  { PIOA, PIO_PA26, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,     TC0_CHA2 }, // D9
  { PIOA, PIO_PA23, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC,    PWM_CH0, NOT_ON_TIMER }, // D10
  { PIOA, PIO_PA13, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM),   NO_ADC, NO_ADC,    PWM_CH2, NOT_ON_TIMER }, // D11
  { PIOA, PIO_PA12, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // D12
  { PIOA, PIO_PA14, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // D13

  // 14 .. 21 - Analog pins
  // ----------------------
  { PIOB, PIO_PB0,  ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC0,   ADC4,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD0
  { PIOB, PIO_PB1,  ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC1,   ADC5,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD1
  { PIOB, PIO_PB2,  ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC2,   ADC6,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD2
  { PIOB, PIO_PB3,  ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC3,   ADC7,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD3
  // 18
  { PIOA, PIO_PA17, ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC4,   ADC0,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD4
  { PIOA, PIO_PA18, ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC5,   ADC1,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD5
  { PIOA, PIO_PA19, ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC6,   ADC2,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD6
  { PIOA, PIO_PA20, ID_PIOA, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   ADC7,   ADC3,   NOT_ON_PWM,  NOT_ON_TIMER }, // AD7

  // 22 .. 36 - Aux pins
  // ----------------------
  { PIOA, PIO_PA15, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,     TC0_CHA1 }, // AUX0
  { PIOA, PIO_PA16, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,     TC0_CHB1 }, // AUX1
  { PIOB, PIO_PB11, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX2
  { PIOB, PIO_PB10, ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX3
  { PIOA, PIO_PA31, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX4
  { PIOA, PIO_PA27, ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC, NO_ADC, NOT_ON_PWM,     TC0_CHB2 }, // AUX5
  { PIOA, PIO_PA28, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX6
  { PIOA, PIO_PA29, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX7
  { PIOA, PIO_PA30, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX8
  { PIOA, PIO_PA2,  ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX9
  { PIOB, PIO_PB4,  ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX10
  { PIOB, PIO_PB5,  ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX11
  { PIOB, PIO_PB6,  ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX12
  { PIOB, PIO_PB7,  ID_PIOB, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL,                  NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }, // AUX13
  { PIOB, PIO_PB13, ID_PIOB, PIO_INPUT,    PIO_DEFAULT, PIN_ATTR_ANALOG,                   NO_ADC, DA0,    NOT_ON_PWM, NOT_ON_TIMER }, // AUX14

  // LEDS
  //{ PIOA, PIO_PA0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), NO_ADC,  NO_ADC, PWM_CH0,  NOT_ON_TIMER },
  //{ PIOA, PIO_PA1,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_PWM), NO_ADC,  NO_ADC, PWM_CH1,  NOT_ON_TIMER },
  { PIOA, PIO_PA0,   ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC,  NO_ADC, NOT_ON_PWM,  TC0_CHA0 }, // LED GREEN
  { PIOA, PIO_PA1,   ID_PIOA, PIO_PERIPH_B, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_TIMER), NO_ADC,  NO_ADC, NOT_ON_PWM,  TC0_CHB0 }, // LED RED
  //{ PIOA, PIO_PA0,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },
  //{ PIOA, PIO_PA1,   ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, PIN_ATTR_DIGITAL, NO_ADC,  NO_ADC, NOT_ON_PWM,  NOT_ON_TIMER },

  // 39 - USART1 (Serial2) all pins
  { PIOA, PIO_PA21|PIO_PA22, ID_PIOA, PIO_PERIPH_A, PIO_DEFAULT, (PIN_ATTR_DIGITAL|PIN_ATTR_COMBO), NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER },

  // END
  { NULL, 0, 0, PIO_NOT_A_PIN, PIO_DEFAULT, 0, NO_ADC, NO_ADC, NOT_ON_PWM, NOT_ON_TIMER }
} ;

#ifdef __cplusplus
}
#endif

/*
 * UART objects
 */
RingBuffer rx_buffer1;
RingBuffer rx_buffer4;

UARTClass Serial2(UART0, UART0_IRQn, ID_UART0, &rx_buffer1);
void serialEvent2() __attribute__((weak));
void serialEvent2() { }

// IT handlers
void UART0_Handler(void)
{
  Serial2.IrqHandler();
}

UARTClass Serial3(UART1, UART1_IRQn, ID_UART1, &rx_buffer4);
void serialEvent3() __attribute__((weak));
void serialEvent3() { }

// IT handlers
void UART1_Handler(void)
{
  Serial3.IrqHandler();
}

// ----------------------------------------------------------------------------
/*
 * USART objects
 */
RingBuffer rx_buffer2;
RingBuffer rx_buffer3;

USARTClass Serial1(USART0, USART0_IRQn, ID_USART0, &rx_buffer2);
void serialEvent1() __attribute__((weak));
void serialEvent1() { }
USARTClass Serial(USART1, USART1_IRQn, ID_USART1, &rx_buffer3);
void serialEvent() __attribute__((weak));
void serialEvent() { }

// IT handlers
void USART0_Handler(void)
{
  Serial1.IrqHandler();
}

void USART1_Handler(void)
{
  Serial.IrqHandler();
}

// ----------------------------------------------------------------------------

void serialEventRun(void)
{
  if (Serial.available()) serialEvent();
  if (Serial1.available()) serialEvent1();
  if (Serial2.available()) serialEvent2();
  if (Serial3.available()) serialEvent3();
}

// ----------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

void __libc_init_array(void);

void init( void )
{
  SystemInit();

  // Set Systick to 1ms interval, common to all SAM3 variants
  if (SysTick_Config(SystemCoreClock / 1000))
  {
    // Capture error
    while (true);
  }

  // Disable watchdog
  WDT_Disable(WDT);

  // Initialize C library
  __libc_init_array();

  // Disable pull-up on every pin
  for (unsigned i = 0; i < PINS_COUNT; i++)
	  digitalWrite(i, LOW);

  // Enable parallel access on PIO output data registers
  PIOA->PIO_OWER = 0xFFFFFFFF;
  PIOB->PIO_OWER = 0xFFFFFFFF;

  // Initialize Serial port U(S)ART pins
  PIO_Configure(
    g_APinDescription[PINS_UART].pPort,
    g_APinDescription[PINS_UART].ulPinType,
    g_APinDescription[PINS_UART].ulPin,
    g_APinDescription[PINS_UART].ulPinConfiguration);
  digitalWrite(0, HIGH); // Enable pullup for RX0
  PIO_Configure(
    g_APinDescription[PINS_USART0].pPort,
    g_APinDescription[PINS_USART0].ulPinType,
    g_APinDescription[PINS_USART0].ulPin,
    g_APinDescription[PINS_USART0].ulPinConfiguration);
  PIO_Configure(
    g_APinDescription[PINS_USART1].pPort,
    g_APinDescription[PINS_USART1].ulPinType,
    g_APinDescription[PINS_USART1].ulPin,
    g_APinDescription[PINS_USART1].ulPinConfiguration);

  // Initialize USB pins
  PIO_Configure(
    g_APinDescription[PINS_USB].pPort,
    g_APinDescription[PINS_USB].ulPinType,
    g_APinDescription[PINS_USB].ulPin,
    g_APinDescription[PINS_USB].ulPinConfiguration);

  // Initialize Analog Controller
  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);
  adc_configure_timing(ADC, 0, ADC_SETTLING_TIME_3, 1);
  adc_configure_trigger(ADC, ADC_TRIG_SW, 0); // Disable hardware trigger.
  adc_disable_interrupt(ADC, 0xFFFFFFFF); // Disable all ADC interrupts.
  adc_disable_all_channel(ADC);

  // Initialize analogOutput module
  analogOutputInit();
}

#ifdef __cplusplus
}
#endif

