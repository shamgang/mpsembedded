/*
 	main.c

	Author: Shamik Ganguly

 	Main file for Music Performance Augmenter external control module.
 	Digitizes potentiometer signals and sends these
	conversions as well	as digital switch signals 
	in a sequence of I2C messages to the I2C master.
*/

// TI device-specific header
#include <msp430g2553.h>

// Number of messages in send sequence
#define numBytes 7

// Initialize I2C
void init_I2C(void);
// ADC set-up function
void adc_Setup(void);
// ADC sample conversion function
void adc_Sam10();

// Enable selected input pin on ADC
void select_pot(int);

// Own slave address
char itgAddress = 0x69;
// Array of adc outputs
int adc[10] = {0};
// Average of adc outputs
int avg_adc = 0;
// Output byte to send
char output;
// Current message index
int	pot_index = 0;

// ADC10 ISR
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR(void)
{
  __bic_SR_register_on_exit(CPUOFF);        // Clear CPUOFF bit from 0(SR)
}

// I2C transmit ISR
#pragma vector = USCIAB0TX_VECTOR
__interrupt void USCIAB0TX_ISR(void)
{
	UCB0TXBUF = output;
	__bic_SR_register_on_exit(CPUOFF);        // Exit LPM0
}

// I2C start condition ISR
#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
	UCB0CTL1 |= UCTXSTP;                    // I2C stop condition
	IFG2 &= ~UCB0TXIFG;                     // Clear USCI_B0 TX int flag
	__bic_SR_register_on_exit(CPUOFF);      // Exit LPM0
}

int main(void)
{
  WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

  // Pin assignments
  // Assign I2C pins to USCI_B0
  P1SEL |= BIT6 + BIT7;
  P1SEL2|= BIT6 + BIT7;
  // Analog ins (pots) left to right
  P1DIR &= ~BIT0; // set P1.0 to input
  P1REN |= BIT0; // enable P1.0 pullup/pulldown resistor
  P1DIR &= ~BIT1; // set P1.1 to input
  P1REN |= BIT1; // enable P1.1 pullup/pulldown resistor
  P1DIR &= ~BIT2; // set P1.2 to input
  P1REN |= BIT2; // enable P1.2 pullup/pulldown resistor
  P1DIR &= ~BIT3; // set P1.3 to input
  P1REN |= BIT3; // enable P1.3 pullup/pulldown resistor
  P1DIR &= ~BIT4; // set P1.4 to input
  P1REN |= BIT4; // enable P1.4 pullup/pulldown resistor
  P1DIR &= ~BIT5; // set P1.5 to input
  P1REN |= BIT5; // enable P1.5 pullup/pulldown resistor
  // Digital ins (switches) left to right
  P2DIR &= ~BIT0; // set P2.0 to input
  P2REN |= BIT0; // enable P2.0 pullup/pulldown resistor
  P2DIR &= ~BIT1; // set P2.1 to input
  P2REN |= BIT1; // enable P2.1 pullup/pulldown resistor
  P2DIR &= ~BIT2; // set P2.2 to input
  P2REN |= BIT2; // enable P2.2 pullup/pulldown resistor
  P2DIR &= ~BIT3; // set P2.3 to input
  P2REN |= BIT3; // enable P2.3 pullup/pulldown resistor
  P2DIR &= ~BIT4; // set P2.4 to input
  P2REN |= BIT4; // enable P2.4 pullup/pulldown resistor
  P2DIR &= ~BIT5; // set P2.5 to input
  P2REN |= BIT5; // enable P2.5 pullup/pulldown resistor

  // Initialize
  init_I2C();
  adc_Setup();

  while(1) {
	  // Increment message index
	  pot_index = (pot_index + 1) % numBytes;
	  // If last message, set bit 7 (sync bit) high
	  // and send digital in values as bits 6-0
	  // Else, assign ADC to analog ins sequentially,
	  // average conversions, and send value as bits 6-0
	  // with bit 7 low
	  if(pot_index == numBytes - 1) {
		  output = (1 << 7) | ((P2IN & BIT5) << 1) | ((P2IN & BIT4) << 1)
						  | ((P2IN & BIT3) << 1) | ((P2IN & BIT2) << 1)
						  | ((P2IN & BIT1) << 1) | ((P2IN & BIT0) << 1);
	  }
	  else {
		  // Select which analog in to convert
		  select_pot(pot_index);
		  // Perform conversions
		  adc_Sam10();
		  // Average conversions
		  avg_adc = ((adc[0]+adc[1]+adc[2]+adc[3]+adc[4]+adc[5]+adc[6]+adc[7]+adc[8]+adc[9]) / 10);
		  // Convert 10 bit ADC output to 7 bit
		  output = (avg_adc >> 3);
	  }
	  __bis_SR_register(CPUOFF + GIE);
  }
}

void init_I2C(void) {
      UCB0CTL1 |= UCSWRST; 			// Enable SW reset
      UCB0CTL0 = UCMODE_3 + UCSYNC; // I2C Slave, synchronous mode
      UCB0I2COA = itgAddress;       // Own Address is 069h
      UCB0CTL1 &= ~UCSWRST;         // Clear SW reset, resume operation
      IE2 |= UCB0TXIE;              // Enable RX and TX interrupt
      //UCB0I2CIE |= UCSTTIE;       // Enable STT interrupt
}

void adc_Setup() {
	ADC10CTL1 = CONSEQ_2 + INCH_4;						// Repeat single channel, A0
	ADC10CTL0 = ADC10SHT_2 + MSC + ADC10ON + ADC10IE;	// Sample & Hold Time + ADC10 ON + Interrupt Enable
	ADC10DTC1 = 0x0A;									// 10 conversions
	ADC10AE0 |= 0x10;									// P1.4 ADC option select
}

void adc_Sam10()
{
    ADC10CTL0 &= ~ENC;				 // Disable Conversion
    while (ADC10CTL1 & BUSY);		 // Wait if ADC10 busy
    ADC10SA = (int)adc;				 // Transfers data to next array (DTC auto increments address)
    ADC10CTL0 |= ENC + ADC10SC;		 // Enable Conversion and conversion start
    __bis_SR_register(CPUOFF + GIE); // Low Power Mode 0, ADC10_ISR
}

void select_pot(int pin_select) {
	ADC10CTL0 &= ~ENC;
	ADC10CTL1 &= ~INCH_15;
	if(pin_select == 0) {
		ADC10CTL1 |= INCH_0;
		ADC10AE0 |= 0x01;
	}
	else if(pin_select == 1) {
		ADC10CTL1 |= INCH_1;
		ADC10AE0 |= 0x02;
	}
	else if(pin_select == 2) {
		ADC10CTL1 |= INCH_2;
		ADC10AE0 |= 0x04;
	}
	else if(pin_select == 3) {
		ADC10CTL1 |= INCH_3;
		ADC10AE0 |= 0x08;
	}
	else if(pin_select == 4) {
		ADC10CTL1 |= INCH_4;
		ADC10AE0 |= 0x10;
	}
	else {
		ADC10CTL1 |= INCH_5;
		ADC10AE0 |= 0x20;
	}
	ADC10CTL0 |= ENC;
}
