#include <msp430g2553.h>
#include <stdint.h>
#include <math.h>

/**
 * main.c
 */
void uartInit();
uint16_t eight2sixteen(uint8_t MSB, uint8_t LSB);
const int numData=4;
uint8_t opCode[3];
uint8_t result[2];
uint16_t temp1;
uint16_t temp2;
uint16_t temp3;
uint16_t xbar;
uint8_t data[numData];
uint8_t doCalcs;
int i;
uint8_t rxCount;
int txCount;

int main(void)
{
WDTCTL = WDTPW | WDTHOLD;	// stop watchdog timer

	BCSCTL1 = CALBC1_16MHZ;     // configure clock to 16 MHz
	DCOCTL = CALDCO_16MHZ;
	uartInit();
	rxCount=0;
	txCount=0;
	doCalcs=0;
	opCode[0]=0;
	opCode[1]=0;
	opCode[2]=0;
  __bis_SR_register(GIE); //enable GIE

//	__bis_SR_register(LPM0_bits + GIE); //swap to low power mode 0, enable GIE
	while(1){
	    //do processing here
	   if(doCalcs==1){
	   //convert from 8bit to two bit
	   if(opCode[0]=='a'){ //add
	       temp2=0;
	       //iterate through the whole data array and convert everything to a 16 bit int
	       for(i=0; i<numData; i=i+2 ){
	           temp1=eight2sixteen(data[i],data[i+1]);
	           temp2=temp1+temp2; //summation
	       }
	       result[0]=(uint8_t)(temp2>>8);//shift msb since tx buffer is only 8 bits wide
	       result[1]=(uint8_t)(temp2); //grabs the LSB
	       UCA0TXBUF=result[0];
 	       UC0IE |= UCA0TXIE; //transmit MSB
 	       //1txCount++;
           //UC0IE |= UCA0TXIE; //transmit MSB

	   }
	  /* else if(opCode[0]=='s'){//sum of (x-xbar)^2
	       xbar=opCode[2]+(opCode[1]<<8); //convert xbar to 16 bit from opcode
	       temp2=0;
	       temp3=0;
	       for(i=0; i<numData; i=i+2 ){
	                     temp1=eight2sixteen(data[i],data[i+1]);
	                     temp2=temp1-xbar; //x-xbar
	                     temp2=temp2*temp2;//squared
	                     temp3=temp2+temp3;
	       }
	       result[0]=(uint8_t)(temp3>>8);//shift msb since tx buffer is only 8 bits wide
	       result[1]=(uint8_t)(temp3); //grabs the LSB
	       UCA0TXBUF=result[0];
	       UC0IE |= UCA0TXIE; //transmit
	   }

	   else if(opCode[0]=='r'){//square root of single val
	         temp1=opCode[2]+(opCode[1]<<8); //convert xbar to 16 bit from opcode
	         temp2=sqrt(temp1);
	         result[0]=(uint8_t)(temp2>>8);//shift msb since tx buffer is only 8 bits wide
	         result[1]=(uint8_t)(temp2); //grabs the LSB
	         UC0IE |= UCA0TXIE; //transmit MSB

	   }
	   else{
	       doCalcs=0; //invalid state, dont do anything
	   }

*/
	 }

	}
}
void uartInit(){

    P1SEL |= BIT1 + BIT2 ; //enable rx and tx
    P1SEL2 |= BIT1 + BIT2 ;
    //Divider = 138.88888888888889;
    //BRDIV = 138
    //Derived from MSP430 Baud Calculator by TI
    UCA0CTL1 |= UCSSEL_2; // SMCLK
    UCA0BR0 = 138; // 1MHz 115200
    UCA0BR1 = 0; // 1MHz 115200
    UCA0MCTL = UCBRS2+UCBRS1+UCBRS0; // Modulation UCBRSx = 7
    UCA0CTL1 &= ~UCSWRST; // **Initialize USCI state machine**
    UC0IE |= UCA0RXIE; // Enable USCI_A0 RX  (if you set tx you transmit garbage)
}

uint16_t eight2sixteen(uint8_t MSB, uint8_t LSB){
uint16_t val=(MSB<<8)+LSB;
return val;
}
//take in data and save for processing

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    txCount=0;
    if(rxCount<=2){
               opCode[rxCount]=UCA0RXBUF;
    }
    if(rxCount>=3){
    data[rxCount-3]=UCA0RXBUF;
    }
    rxCount++;
    if(rxCount==numData+3){
    doCalcs=1;
    rxCount=0;
 //   __bic_SR_register(LPM0_bits); //no low power till calcs done
    }

}
//transmit processed data
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    txCount++;
    UCA0TXBUF=result[txCount];
    if (txCount == 1){ // TX over?
       doCalcs=0;
       opCode[0]=0;
       txCount=0;
       UC0IE &= ~UCA0TXIE; // Disable USCI_A0 TX interrupt
       // if(txCount>=1){
   //         txCount=0;
   //         doCalcs=0;
   //         opCode[0]=0; //stops from constantly transmitting the same result over and over
           // __bis_SR_register(LPM0_bits + GIE); //swap to low power mode 0, enable GIE
            //calcs done enable low power
    }
}
