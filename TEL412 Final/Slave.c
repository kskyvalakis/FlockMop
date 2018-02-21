/********   Authors  ***********
	Demertzis Rafail-Athanasios
	Giannelos Evagelos
	Mpountrogiannis Konstantinos
	Psarakis Kyriakos
	Sdoukopoulou Glykeria
	Sgourakis Andreas
	Skyvalakis Konstantinos
	Spiliotis Giorgos
	Stratigi Eirini
	Tsetis Ioannis
	Tzagkarakis Eleftherios
	Vestakis Marios
********************************/
//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------

#include "compiler_defs.h"
#include "C8051F320_defs.h"            // SFR declarations
#include "common.h"
#include <stdio.h>
#include "cc2500.h"
#include <stdlib.h>

//-------------------------------------------------------------------------------------------------------
// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS {
	BYTE FSCTRL1;   // Frequency synthesizer control.
	BYTE FSCTRL0;   // Frequency synthesizer control.
	BYTE FREQ2;     // Frequency control word, high byte.
	BYTE FREQ1;     // Frequency control word, middle byte.
	BYTE FREQ0;     // Frequency control word, low byte.
	BYTE MDMCFG4;   // Modem configuration.
	BYTE MDMCFG3;   // Modem configuration.
	BYTE MDMCFG2;   // Modem configuration.
	BYTE MDMCFG1;   // Modem configuration.
	BYTE MDMCFG0;   // Modem configuration.
	BYTE CHANNR;    // Channel number.
	BYTE DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
	BYTE FREND1;    // Front end RX configuration.
	BYTE FREND0;    // Front end RX configuration.
	BYTE MCSM0;     // Main Radio Control State Machine configuration.
	BYTE FOCCFG;    // Frequency Offset Compensation Configuration.
	BYTE BSCFG;     // Bit synchronization Configuration.
	BYTE AGCCTRL2;  // AGC control.
	BYTE AGCCTRL1;  // AGC control.
	BYTE AGCCTRL0;  // AGC control.
	BYTE FSCAL3;    // Frequency synthesizer calibration.
	BYTE FSCAL2;    // Frequency synthesizer calibration.
	BYTE FSCAL1;    // Frequency synthesizer calibration.
	BYTE FSCAL0;    // Frequency synthesizer calibration.
	BYTE FSTEST;    // Frequency synthesizer calibration control
	BYTE TEST2;     // Various test settings.
	BYTE TEST1;     // Various test settings.
	BYTE TEST0;     // Various test settings.
	BYTE FIFOTHR;   // RXFIFO and TXFIFO thresholds.
	BYTE IOCFG2;    // GDO2 output pin configuration
	BYTE IOCFG0;    // GDO0 output pin configuration
	BYTE PKTCTRL1;  // Packet automation control.
	BYTE PKTCTRL0;  // Packet automation control.
	BYTE ADDR;      // Device address.
	BYTE PKTLEN;    // Packet length.
} RF_SETTINGS;

//-----------------------------------------------------------------------------
// Global CONSTANTS
//-----------------------------------------------------------------------------
// Defines
#define Fwd (39)//+1)
#define Back (10)//+1)
#define Left (63)//+1)
#define Right (57)//+1)
#define FLeft (45)//+1)
#define FRight (51)//+1)
#define BLeft (34)//+1)
#define BRight (28)//+1)
#define RRight 15.0
#define RLeft 22.0
#define Ffwd 14
#define Bbwd 9
#define Lbot (((RLeft-RRight)/2)/RLeft)
#define Ltop (((RLeft+RLeft-RRight)/2)/RLeft)
#define Rbot (((RRight+RRight-RLeft)/2)/RRight)
#define Rtop (((RRight+RRight+RRight-RLeft)/2)/RRight)

#define CRC_OK              0x80
#define LQI                 1
#define BYTES_IN_RXFIFO     0x7F
#define BAUDRATE        9600           // Baud rate of UART in bps

#define MODE_NOT_SET    0
#define TX              1
#define RX              2

//#define SYSCLK     24500000
#define BM_SPI0E        0x02    // SPI I/O Enable
#define BM_XBAR         0x40    // Crossbar Enable
#define BM_PCA0ME_1     0x01    //      001: CEX0 routed to Port pin.
#define BM_WEAKPUD      0x80    // Port I/O Weak Pull-up Disable:
//      0: Weak Pull-ups enabled (except for Ports whose I/O are
//         configured as analog input or push-pull output).
//      1: Weak Pull-ups disabled.
#define BM_SPIEN    0x01    // SPI0 Enable: Active high
#define BM_MSTEN    0x40    // Master Mode Enable: Active High
#define BM_NSSMD1   0x08    // Slave Select Mode

// Port 0
#define SCLK_           0x01    // P0.0     SPI Serial clock                            	Output
#define SO_             0x02    // P0.1     SPI MISO signal, xx00 SO/GDO1               	Output
#define GDO1_           0x02    // P0.1
#define SI_             0x04    // P0.2     SPI MOSI signal, xx00 SI                    	Input
#define CSn_            0x08    // P0.3     SPI slave select signal                     	Output
#define GDO0_           0x40    // P0.6     xx00 GDO0                                   	Input
#define GDO2_           0x80    // P0.7     xx00 GDO2                                   	Input
SBIT(P0_1, SFR_P0, 1);
SBIT(GDO0_PIN, SFR_P0, 6);

//Port 2
#define LED1_           0x40    // P2.2     LED1, (Green)                               	Output
#define LED2_           0x80    // P2.3     LED2, (Green)                               	Output
SBIT(LED, SFR_P2, 2);                  // LED='1' means ON
SBIT(LED2, SFR_P2, 3);                 // LED='1' means ON
SBIT(SW2, SFR_P2, 0);                  // SW2='0' means switch pressed

//-------------------------------------------------------------------------------------------------------
#define INT_OSC             0x00
#define FOUR_X_CLK_MULT     0x02
#define INT_OSC_DIV_1       0x83
#define BM_MULRDY   0x20    // Clock Multiplier Ready. This read-only bit indicates the status of the
// Clock Multiplier.
//      0: Clock Multiplier not ready.
//      1: Clock Multiplier ready (locked).
// Select the Internal Oscillator as Multiplier input source and disable the watchdog timer
// SYSCLK = 4X Clock Multiplier / 2
/*#define CLOCK_INIT() \
    do { \
        UINT8 i; \
        PCA0MD &= ~0x40; \
        CLKMUL = INT_OSC; \
        CLKMUL |= 0x80; \
        for (i = 0; i < 20; i++); \
        CLKMUL |= 0xC0; \
        while (!(CLKMUL & BM_MULRDY)); \
        CLKSEL |= FOUR_X_CLK_MULT; \
        OSCICN = INT_OSC_DIV_1; \
    } while (0)*/
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// SPI enable/disable macros:

// Definitions to support burst/single access:
#define WRITE_BURST     0x40
#define READ_SINGLE     0x80
#define READ_BURST      0xC0
//-------------------------------------------------------------------------------------------------------

#define SPI_ENABLE()        (SPI0CN |= BM_SPIEN)
#define SPI_DISABLE()       (SPI0CN &= ~BM_SPIEN)
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// SPI Initialization
// Enable SPI (4-wire Single Master Mode, data centered on first edge of SCK period.
// SCK low in Idle State
#define SPI_INIT(freq) \
    do { \
        SPI0CFG = BM_MSTEN; \
        SPI0CN = BM_NSSMD1; \
        SPI0CKR = freq; \
        SPI_ENABLE(); \
    } while (0)

// where freq is one of:
#define SCLK_6_MHZ      1
#define SCLK_4_MHZ      2
#define SCLK_3_MHZ      3
#define SCLK_2_4_MHZ    4
#define SCLK_2_MHZ      5
#define SCLK_1_5_MHZ    7
#define SCLK_1_2_MHZ    9
#define SCLK_1_MHZ      11
//-------------------------------------------------------------------------------------------------------

SBIT(wave, SFR_P1, 4);                  // wave output

//-----------------------------------------------------------------------------
// Function PROTOTYPES (Declarations)
//-----------------------------------------------------------------------------
void spi_wait(void);
void halSpiWriteReg(BYTE addr, BYTE value);
void halRfWriteRfSettings(RF_SETTINGS *pRfSettings);
void halRfSendPacket(BYTE *txBuffer, UINT8 size);
void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count);
void halSpiStrobe(BYTE strobe);
BYTE halSpiReadStatus(BYTE addr);
BYTE halSpiReadReg(BYTE addr);
void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count);
BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length);

void halWait(UINT16 timeout);
void intToAscii(UINT32 value);
void SYSCLK_Init(void);
//void UART0_Init (void);
void PORT_Init(void);
void Timer2_Init(int counts);
void wavegen(int ont, int offt, int times);
void move(int movementFlag);
void movementTime( int move,int time);
void halWait1(int timeout1);
void decision(float cm,float degrees);

//INTERRUPT_PROTO(Timer2_ISR, INTERRUPT_TIMER2);

//-------------------------------------------------------------------------------------------------------
//  Global Variables

BYTE xdata txBuffer[4];
BYTE xdata rxBuffer[3]; //Length byte  + 2 status bytes are not stored in this buffer
UINT8 xdata mode = MODE_NOT_SET;
BYTE xdata asciiString[11];
int k=(INT16)(2);
int myid=1;
int response_id=5;
unsigned int deg,cm,id;
//radio settings
// Chipcon
// Product = CC2500
// Chip version = E   (VERSION = 0x03)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 541.666667 kHz
// Phase = 1
// Datarate = 249.938965 kBaud
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 2432.999908 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (3) 30/32 sync word bits detected
// Format of RX/TX data = (0) Normal mode, use FIFOs for RX and TX
// CRC operation = (1) CRC calculation in TX and CRC check in RX enabled
// Forward Error Correction = (0) FEC disabled
// Length configuration = (1) Variable length packets, packet length configured by the first received byte after sync word.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 1
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = ( 6) Asserts when sync word has been sent / received, and de-asserts at the end of the packet
// GDO2 signal selection = (41) CHIP_RDY
RF_SETTINGS code rfSettings = {
		  0x0A,   // FSCTRL1   Frequency synthesizer control.
		    0x00,   // FSCTRL0   Frequency synthesizer control.
		    0x5D,   // FREQ2     Frequency control word, high byte.
		    0x93,   // FREQ1     Frequency control word, middle byte.
		    0xB1,   // FREQ0     Frequency control word, low byte.
		    0x2D,   // MDMCFG4   Modem configuration.
		    0x3B,   // MDMCFG3   Modem configuration.
		    0x73,   // MDMCFG2   Modem configuration.
		    0x22,   // MDMCFG1   Modem configuration.
		    0xF8,   // MDMCFG0   Modem configuration.
		    0x00,   // CHANNR    Channel number.
		    0x01,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
		    0xB6,   // FREND1    Front end RX configuration.
		    0x10,   // FREND0    Front end TX configuration.
		    0x18,   // MCSM0     Main Radio Control State Machine configuration.
		    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
		    0x1C,   // BSCFG     Bit synchronization Configuration.
		    0xC7,   // AGCCTRL2  AGC control.
		    0x00,   // AGCCTRL1  AGC control.
		    0xB0,   // AGCCTRL0  AGC control.
		    0xEA,   // FSCAL3    Frequency synthesizer calibration.
		    0x0A,   // FSCAL2    Frequency synthesizer calibration.
		    0x00,   // FSCAL1    Frequency synthesizer calibration.
		    0x11,   // FSCAL0    Frequency synthesizer calibration.
		    0x59,   // FSTEST    Frequency synthesizer calibration.
		    0x88,   // TEST2     Various test settings.
		    0x31,   // TEST1     Various test settings.
		    0x0B,   // TEST0     Various test settings.
		    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
		    0x29,   // IOCFG2    GDO2 output pin configuration.
		    0x06,   // IOCFG0D   GDO0 output pin configuration.
		    0x04,   // PKTCTRL1  Packet automation control.
		    0x05,   // PKTCTRL0  Packet automation control.
		    0x00,   // ADDR      Device address.
		    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0xFE;

//-------------------------------------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// MAIN Routine
//-----------------------------------------------------------------------------
void main (void) {

UINT32 packetsReceived = 0;
UINT32 packetsSent = 0;
UINT8 length;
BYTE mytestbyte;
UINT8 rssi_dec;
INT16 rssi_dBm;
UINT8 rssi_offset = 72;


   PCA0MD &= ~0x40;                    // Disable watchdog timer
   SYSCLK_Init ();                     // Initialize system clock to
                                       // 24.5MHz


   //CLOCK_INIT();                       // Initialize clock
   PORT_Init ();                       // Initialize crossbar and GPIO
   SPI_INIT(SCLK_6_MHZ);               // Initialize SPI
   //UART0_Init();



   NSSMD0 = 1; 						   // you need to wait ~41 usecs, before CC2500 responds
   halWait(1);
   NSSMD0 = 0;
   halWait(1);
   NSSMD0 = 1;
   halWait(41);

   //now reset the radio;
	do {
    	    NSSMD0 = 0;
        	while (P0_1);
        	SPI0DAT = CCxxx0_SRES;
        	spi_wait();
			NSSMD0 = 1;
    	} while (0);

   //_nop_();

   halRfWriteRfSettings(&rfSettings);
   halSpiWriteReg(CCxxx0_PATABLE, paTable);


   //Timer2_Init (SYSCLK / 12 / 10);     // Init Timer2 to generate
                                         // interrupts at a 10Hz rate.

   EA = 0;                             // Enable global interrupts

   while (1){

	   //decision(50,50);
//wave=1;
	   //LED = ~LED;
	 /*
	 LED = ~LED;
	 //TX mode
	 halRfSendPacket(txBuffer, sizeof(txBuffer));
	 intToAscii(++packetsSent);
	 //endof TX mode
     halWait(30000);
	 halWait(30000);
	 //halWait(30000);
	 //halWait(30000);
	 */
//halWait1(7000);

//halWait1(10000);

	 ////RX mode
	 length = sizeof(rxBuffer);
     if (halRfReceivePacket(rxBuffer, &length)) {
         intToAscii(++packetsReceived);


	 		txBuffer[0]=3;
	 		txBuffer[1]=rxBuffer[0];
	 		txBuffer[2]=rxBuffer[1];
	 		txBuffer[3]=rxBuffer[2];
         //rxBuffer[0]=(INT16)(rxBuffer[0]);
		 	//printf("%d\n%d\n%d\n", (INT16)rxBuffer[0], (INT16)rxBuffer[1], (INT16)rxBuffer[2]);
		 	if(rxBuffer[0]<=8){
		 		id=rxBuffer[0];
		 	    deg=rxBuffer[1];
		 	    cm=rxBuffer[2];
		 	}
		 	else if(rxBuffer[0]<=72 ){
		 		id=rxBuffer[0]-64;
				deg=rxBuffer[1];
				cm=rxBuffer[2]+256;
			}
		 	else if(rxBuffer[0]<=136){
		 		id=rxBuffer[0]-128;
		 	    deg=rxBuffer[1]+256;
		 		cm=rxBuffer[2];
		 	}
		 	else{
		 		id=rxBuffer[0]-192;
		 		deg=rxBuffer[1]+256;
		 	    cm=rxBuffer[2]+256;
		 	}
		 	if (id==response_id){
		 		int i=0;
		 		halWait(30000);
		 				 		halWait(30000);
		 				 		halWait(30000);
		 				 		halWait(30000);
		 					 	halWait(30000);
		 					 	halWait(30000);
		 					 	for(i=0;i<10;i++){
		 				 		 halRfSendPacket(txBuffer, sizeof(txBuffer));
		 				 		 intToAscii(++packetsSent);
		 					 	}
		 					 	halWait(30000);
		 					 	halWait(30000);

		 					 	if (packetsSent>0)
		 				 			LED = !LED;
		 				 		 //endof TX mode
		 	}
		 	if(id==myid) {
		 	LED = !LED;
			 	//halWait(30000);
			 	//halWait(30000);
			      EA = 0;

			      halWait1(3000);

			      decision(cm,deg);
			      halWait1(2000);
			      EA=1;
			      halWait1(7000);
			      cm=0;
			      deg=0;

		 	}





   }
}

}
//-----------------------------------------------------------------------------
// SYSCLK_Init
//-----------------------------------------------------------------------------
//
// Return Value: none
// Parameters:   none
//
// This routine initializes the system clock to use the internal 24.5MHz / 8
// oscillator as its clock source.  Also enables missing clock detector reset.
//
//-----------------------------------------------------------------------------
void SYSCLK_Init(void) {
	 	 OSCICN = 0x81;
	 	 //CLKSEL = 0x20;
	 	 //RSTSRC  = 0x04;                     // Enable missing clock detector
}

//-----------------------------------------------------------------------------
// PORT_Init
//-----------------------------------------------------------------------------
//
// Return Value: none
// Parameters:   none
//
// Configure the Crossbar and GPIO ports.
//
// P2.2 - LED (push-pull)
//
// All other port pins unused
//
//-----------------------------------------------------------------------------
void PORT_Init(void) {
	//P2MDOUT = LED1_ | LED2_;         // Enable two LEDs as a push-pull output at port 2
	P2MDOUT = 0x04 | 0x08;
	P1MDOUT |= 0x20;
	//P0MDOUT = SCLK_ | SO_ | SI_ | CSn_;// Enable push-pull output at port 0
	P0MDOUT = 0x01 | 0x02 | 0x04 | 0x08;

	P2MDIN = (~0x01);                  //set P2.0 as analog input

	//P0SKIP = GDO0_ | GDO2_;             //Crossbar skips the selected bits, used as analog input/output
	P0SKIP = 0x40 | 0x80;

	//XBR0 = BM_SPI0E;                        // SPI peripheral selected
	XBR0 = 0x02 | 0x01 | 0x08;

	//XBR1 = BM_XBAR | BM_PCA0ME_1  | BM_WEAKPUD;// Enable crossbar, CEX0 routed to port pin and disable weak pull-ups
	XBR1 = 0x40 | 0x01 | 0x80;

	P0MDOUT |= 0x10;                    // enable UTX as push-pull output

	 P1MDOUT |= 0x20;
	 P0MDOUT |= 0x01;


	 //XBR0 = 0x08;                        // SYSCLK routed to pin - No digital peripherals selected
	 //XBR1 = 0x40;                        // Enable crossbar and weak pull-ups


}

//-----------------------------------------------------------------------------
// Timer2_Init
//-----------------------------------------------------------------------------
//
// Return Value: none
// Parameters:   none
//
// Configure Timer2 to 16-bit auto-reload and generate an interrupt at
// interval specified by <counts> using SYSCLK/48 as its time base.
//
//-----------------------------------------------------------------------------
void Timer2_Init(int counts) {
	TMR2CN = 0x00;                      // Stop Timer2; Clear TF2;
										// use SYSCLK/12 as timebase
	CKCON &= ~0x30;                     // Timer2 clocked based on T2XCLK;

	TMR2RL = -counts;                   // Init reload values
	TMR2 = 0xffff;                      // Set to reload immediately
	ET2 = 1;                            // Enable Timer2 interrupts
	TR2 = 1;                            // Start Timer2
}

//-----------------------------------------------------------------------------
// Interrupt Service Routines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Timer2_ISR
//-----------------------------------------------------------------------------
//
// This routine changes the state of the LED whenever Timer2 overflows.
//
//-----------------------------------------------------------------------------
INTERRUPT(Timer2_ISR, INTERRUPT_TIMER2) {
	TF2H = 0;                           // Clear Timer2 interrupt flag
	//LED = !LED;                         // Change state of LED
	LED2 = !LED2;                         // Change state of LED

}

//-------------------------------------------------------------------------------------------------------
//  void intToAscii(UINT32 value)
//
//  DESCRIPTION:
//		Takes a 32 bits interger as input and converts it to ascii. Puts the result in the global
//      variable asciiString[]
//
//	ARGUMENTS:
//		UINT32 value
//			The value to be converted
//-------------------------------------------------------------------------------------------------------
void intToAscii(UINT32 value) {
	UINT8 i;
	UINT8 j = 0;
	UINT8 digit_start = 0;
	UINT16 digit = 0;
	UINT32 denom = 1000000000;

	if (value == 0) {
		asciiString[0] = '0';
		asciiString[1] = NULL;
	} else {
		for (i = 10; i > 0; i--) {
			digit = value / denom;
			if ((digit_start == 1) || (digit != 0)) {
				digit_start = 1;
				value %= denom;
				asciiString[j++] = (digit + '0');
			}
			denom /= 10;
		}
		asciiString[j++] = NULL;
	}
}                         // intToAscii

//stops for approximatelly timeout usecs
void halWait(UINT16 timeout) {
	do {
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
		_nop_();
	} while (--timeout);
}                         // halWait

void spi_wait(void) {
	do {
		while (!SPIF)
			;
		SPIF = 0;
	} while (0);

}

void halSpiWriteReg(BYTE addr, BYTE value) {
	NSSMD0 = 0;
	while (P0_1)
		;
	SPI0DAT = addr;
	spi_wait();
	SPI0DAT = value;
	spi_wait();
	NSSMD0 = 1;
}                         // halSpiWriteReg

void halRfWriteRfSettings(RF_SETTINGS *pRfSettings) {

	// Write register settings
	halSpiWriteReg(CCxxx0_FSCTRL1, pRfSettings->FSCTRL1);
	halSpiWriteReg(CCxxx0_FSCTRL0, pRfSettings->FSCTRL0);
	halSpiWriteReg(CCxxx0_FREQ2, pRfSettings->FREQ2);
	halSpiWriteReg(CCxxx0_FREQ1, pRfSettings->FREQ1);
	halSpiWriteReg(CCxxx0_FREQ0, pRfSettings->FREQ0);
	halSpiWriteReg(CCxxx0_MDMCFG4, pRfSettings->MDMCFG4);
	halSpiWriteReg(CCxxx0_MDMCFG3, pRfSettings->MDMCFG3);
	halSpiWriteReg(CCxxx0_MDMCFG2, pRfSettings->MDMCFG2);
	halSpiWriteReg(CCxxx0_MDMCFG1, pRfSettings->MDMCFG1);
	halSpiWriteReg(CCxxx0_MDMCFG0, pRfSettings->MDMCFG0);
	halSpiWriteReg(CCxxx0_CHANNR, pRfSettings->CHANNR);
	halSpiWriteReg(CCxxx0_DEVIATN, pRfSettings->DEVIATN);
	halSpiWriteReg(CCxxx0_FREND1, pRfSettings->FREND1);
	halSpiWriteReg(CCxxx0_FREND0, pRfSettings->FREND0);
	halSpiWriteReg(CCxxx0_MCSM0, pRfSettings->MCSM0);
	halSpiWriteReg(CCxxx0_FOCCFG, pRfSettings->FOCCFG);
	halSpiWriteReg(CCxxx0_BSCFG, pRfSettings->BSCFG);
	halSpiWriteReg(CCxxx0_AGCCTRL2, pRfSettings->AGCCTRL2);
	halSpiWriteReg(CCxxx0_AGCCTRL1, pRfSettings->AGCCTRL1);
	halSpiWriteReg(CCxxx0_AGCCTRL0, pRfSettings->AGCCTRL0);
	halSpiWriteReg(CCxxx0_FSCAL3, pRfSettings->FSCAL3);
	halSpiWriteReg(CCxxx0_FSCAL2, pRfSettings->FSCAL2);
	halSpiWriteReg(CCxxx0_FSCAL1, pRfSettings->FSCAL1);
	halSpiWriteReg(CCxxx0_FSCAL0, pRfSettings->FSCAL0);
	halSpiWriteReg(CCxxx0_FSTEST, pRfSettings->FSTEST);
	halSpiWriteReg(CCxxx0_TEST2, pRfSettings->TEST2);
	halSpiWriteReg(CCxxx0_TEST1, pRfSettings->TEST1);
	halSpiWriteReg(CCxxx0_TEST0, pRfSettings->TEST0);
	halSpiWriteReg(CCxxx0_FIFOTHR, pRfSettings->FIFOTHR);
	halSpiWriteReg(CCxxx0_IOCFG2, pRfSettings->IOCFG2);
	halSpiWriteReg(CCxxx0_IOCFG0, pRfSettings->IOCFG0);
	halSpiWriteReg(CCxxx0_PKTCTRL1, pRfSettings->PKTCTRL1);
	halSpiWriteReg(CCxxx0_PKTCTRL0, pRfSettings->PKTCTRL0);
	halSpiWriteReg(CCxxx0_ADDR, pRfSettings->ADDR);
	halSpiWriteReg(CCxxx0_PKTLEN, pRfSettings->PKTLEN);
}                         // halRfWriteRfSettings

//-------------------------------------------------------------------------------------------------------
//  void halRfSendPacket(BYTE *txBuffer, UINT8 size)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with packet length up to 63 bytes.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.
//
//  ARGUMENTS:
//      BYTE *txBuffer
//          Pointer to a buffer containing the data that are going to be transmitted
//
//      UINT8 size
//          The size of the txBuffer
//-------------------------------------------------------------------------------------------------------
void halRfSendPacket(BYTE *txBuffer, UINT8 size) {

	halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);

	halSpiStrobe(CCxxx0_STX);
	//LED = !LED;

	// Wait for GDO0 to be set -> sync transmitted
	while (!GDO0_PIN)
		;

	// Wait for GDO0 to be cleared -> end of packet
	while (GDO0_PIN)
		;

}                         // halRfSendPacket

//-------------------------------------------------------------------------------------------------------
//  void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//
//  DESCRIPTION:
//      This function writes to multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      BYTE *buffer
//          Array of bytes to be written into a corresponding range of
//          CCxx00 registers, starting by the address specified in _addr_.
//      BYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
//-------------------------------------------------------------------------------------------------------
void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count) {
	UINT8 i;
	NSSMD0 = 0;
	while (P0_1)
		;
	SPI0DAT = addr | WRITE_BURST;
	spi_wait();
	for (i = 0; i < count; i++) {
		SPI0DAT = buffer[i];
		spi_wait();
	}
	NSSMD0 = 1;
}                         // halSpiWriteBurstReg

//-------------------------------------------------------------------------------------------------------
//  void halSpiStrobe(BYTE strobe)
//
//  DESCRIPTION:
//      Function for writing a strobe command to the CCxxx0
//
//  ARGUMENTS:
//      BYTE strobe
//          Strobe command
//-------------------------------------------------------------------------------------------------------
void halSpiStrobe(BYTE strobe) {
	NSSMD0 = 0;
	while (P0_1)
		;
	SPI0DAT = strobe;
	spi_wait();
	NSSMD0 = 1;
}                         // halSpiStrobe

//-------------------------------------------------------------------------------------------------------
//  BYTE halSpiReadStatus(BYTE addr)
//
//  DESCRIPTION:
//      This function reads a CCxxx0 status register.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the CCxxx0 status register to be accessed.
//
//  RETURN VALUE:
//      BYTE
//          Value of the accessed CCxxx0 status register.
//-------------------------------------------------------------------------------------------------------
BYTE halSpiReadStatus(BYTE addr) {
	UINT8 x;
	NSSMD0 = 0;
	while (P0_1)
		;
	SPI0DAT = (addr | READ_BURST);
	spi_wait();
	SPI0DAT = 0;
	spi_wait();
	x = SPI0DAT;
	NSSMD0 = 1;
	return x;
}                         // halSpiReadStatus

//-------------------------------------------------------------------------------------------------------
//  BYTE halSpiReadReg(BYTE addr)
//
//  DESCRIPTION:
//      This function gets the value of a single specified CCxxx0 register.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the CCxxx0 register to be accessed.
//
//  RETURN VALUE:
//      BYTE
//          Value of the accessed CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
BYTE halSpiReadReg(BYTE addr) {
	UINT8 x;
	NSSMD0 = 0;
	while (P0_1)
		;
	SPI0DAT = (addr | READ_SINGLE);
	spi_wait();
	SPI0DAT = 0;
	spi_wait();
	x = SPI0DAT;
	NSSMD0 = 1;
	return x;
}
// halSpiReadReg




//-------------------------------------------------------------------------------------------------------
//  void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//
//  DESCRIPTION:
//      This function reads multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      BYTE *buffer
//          Pointer to a byte array which stores the values read from a
//          corresponding range of CCxxx0 registers.
//      BYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
//-------------------------------------------------------------------------------------------------------
void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count) {
	UINT8 i;
	NSSMD0 = 0;
	while (P0_1)
		;
	SPI0DAT = (addr | READ_BURST);
	spi_wait();
	for (i = 0; i < count; i++) {
		SPI0DAT = 0;
		spi_wait();
		buffer[i] = SPI0DAT;
	}
	NSSMD0 = 1;
}                         // halSpiReadBurstReg

//-------------------------------------------------------------------------------------------------------
//  BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length)
//
//  DESCRIPTION:
//      This function can be used to receive a packet of variable packet length (first byte in the packet
//      must be the length byte). The packet length should not exceed the RX FIFO size.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      Also, APPEND_STATUS in the PKTCTRL1 register must be enabled.
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.
//      After the GDO0 pin has been de-asserted, the RXBYTES register is read to make sure that there
//      are bytes in the FIFO. This is because the GDO signal will indicate sync received even if the
//      FIFO is flushed due to address filtering, CRC filtering, or packet length filtering.
//
//  ARGUMENTS:
//      BYTE *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      UINT8 *length
//          Pointer to a variable containing the size of the buffer where the incoming data should be
//          stored. After this function returns, that variable holds the packet length.
//
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK (or no packet was put in the RX FIFO due to filtering)
//-------------------------------------------------------------------------------------------------------
BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length) {
	BYTE status[2];
	UINT8 packetLength;

	halSpiStrobe(CCxxx0_SRX);

	// Wait for GDO0 to be set -> sync received
	while (!GDO0_PIN)
		;

	// Wait for GDO0 to be cleared -> end of packet
	while (GDO0_PIN)
		;

	// This status register is safe to read since it will not be updated after
	// the packet has been received (See the CC1100 and 2500 Errata Note)
	if ((halSpiReadStatus(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)) {

		// Read length byte
		packetLength = halSpiReadReg(CCxxx0_RXFIFO);

		// Read data from RX FIFO and store in rxBuffer
		if (packetLength <= *length) {
			halSpiReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength);
			*length = packetLength;

			// Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
			halSpiReadBurstReg(CCxxx0_RXFIFO, status, 2);
			//printf("%d",status[0]);
			// MSB of LQI is the CRC_OK bit
			return (status[LQI] & CRC_OK);
		} else {
			*length = packetLength;

			// Make sure that the radio is in IDLE state before flushing the FIFO
			// (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point)
			halSpiStrobe(CCxxx0_SIDLE);

			// Flush RX FIFO
			halSpiStrobe(CCxxx0_SFRX);
			return FALSE;
		}
	} else
		return FALSE;
}            // halRfReceivePacket
/*
void UART0_Init (void)
{
   SCON0 = 0x10;                       // SCON0: 8-bit variable bit rate
                                       //        level of STOP bit is ignored
                                       //        RX enabled
                                       //        ninth bits are zeros
                                       //        clear RI0 and TI0 bits
   if (SYSCLK/BAUDRATE/2/256 < 1) {
      TH1 = -(SYSCLK/BAUDRATE/2);
      CKCON &= ~0x0B;                  // T1M = 1; SCA1:0 = xx
      CKCON |=  0x08;
   } else if (SYSCLK/BAUDRATE/2/256 < 4) {
      TH1 = -(SYSCLK/BAUDRATE/2/4);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 01
      CKCON |=  0x01;
   } else if (SYSCLK/BAUDRATE/2/256 < 12) {
      TH1 = -(SYSCLK/BAUDRATE/2/12);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 00
   } else {
      TH1 = -(SYSCLK/BAUDRATE/2/48);
      CKCON &= ~0x0B;                  // T1M = 0; SCA1:0 = 10
      CKCON |=  0x02;
   }

   TL1 = TH1;                          // init Timer1
   TMOD &= ~0xf0;                      // TMOD: timer 1 in 8-bit autoreload
   TMOD |=  0x20;
   TR1 = 1;                            // START Timer1
   TI0 = 1;                            // Indicate TX0 ready
}
*/

void decision(float cm,float degrees){
	float xy1,xy3,y1,y3;
	int i,x,y,xy2,y2;
	int Bflag;


	Bflag = 0;
	x=0;
	xy1=0;
	xy2=0;
	xy3=0;
	y=0;
	y1=0;
	y2=0;
	y3=0;
    if(degrees > 180 && degrees <270){
    	Bflag=1;
    	degrees=degrees-180;
    }else if(degrees > 90 && degrees <=180){
    	Bflag=1;
    	degrees=degrees+180;
    }
	if(degrees <= 90){
		xy1=degrees/RLeft;
		xy2=xy1;
		xy3=xy1-xy2;
		if(xy3 < Lbot){//xy3 < 0.159
			x=xy2;
			y=0;
		}
		else if(Lbot <= xy3 && xy3 <= Ltop){
			x=xy2+1;
			y=1;
		}
		else if(Ltop < xy3 ){//0.659
			x=xy2+1;
			y=0;
		}else{
			x=0;
			y=0;
		}
	}else if(degrees >= 270){
		xy1=(360-degrees)/RRight;
		xy2=xy1;
		xy3=xy1-xy2;
		if(xy3 < Rbot){// 0.267
			x=0;
			y=xy2;
		}
		else if(Rbot <= xy3 && xy3 <= Rtop){
			x=1;
			y=xy2+2;
		}
		else if(Rtop < xy3 ){//0.767
			x=0;
			y=xy2+1;
		}else{
			x=0;
			y=0;
		}

	}else{
		x=0;
		y=0;
	}

	for(i=0;i<x;i++){
		halWait1(7000);
		movementTime(Left,4);
		movementTime(FLeft,5);
		halWait1(7000);
		movementTime(Right,4);
		movementTime(BRight,5);  //4 -> 90
	}
	for(i=0;i<y;i++){
		halWait1(7000);
		movementTime(Right,4);
		movementTime(FRight,4);
		halWait1(7000);
		movementTime(Left,4);
		movementTime(BLeft,6);
	}
	x=0;
	y=0;
	if(Bflag==0){
		y1=cm/Ffwd;
		y2=y1;
		y3=y1-y2;
		if(0.5<y3){
			y2=y2+1;
		}
		for(i=0;i<y2;i++){//y1-1 --> y1
				halWait1(5000);
				movementTime(Fwd,3);
		}
	}else if(Bflag==1){
		y1=cm/Bbwd;
		y2=y1;
		y3=y1-y2;
		if(0.5<y3){
			y2=y2+1;
		}
		for(i=0;i<y2;i++){//y1-1 --> y1
				halWait1(5000);
				movementTime(Back,3);
		}
	}
 }





void movementTime(int move1, int time){
	int i;
	for(i=0;i<time;i++){
		move(move1);
	}

}
void move(int move)
{
	wavegen(67,199,2);
	wavegen(67,67,move);
	wavegen(67,199,2);
}

void wavegen(int ont, int offt, int times)
{
	int i;
	int j;
	for(i=0; i<times; i++){
		wave=1;
		for(j=0;j<ont;j++){
			NOP();
		}
		wave=0;
		for(j=0;j<offt;j++){
			NOP();
		}
	}
}

void halWait1(int timeout1) {
    double i;
    for(i=0;i<timeout1;i++){
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
        nop_();
    }

}
//-----------------------------------------------------------------------------
// End Of File
//-----------------------------------------------------------------------------
