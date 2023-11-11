// Listen for Medtronic packets and print them over the UART
#include "ioCC1111.h"
#include "ioCCxx10_bitdef.h"
#include "aes.h"

#define FREQUENCY 868
#define RFIF_IM_DONE	(1 << 4)
#define BOARD 1


#define RFTX do { RFST = RFST_STX; while ((MARCSTATE) != MARC_STATE_TX); } while (0)
#define RFOFF do { RFST = RFST_SIDLE; while ((MARCSTATE) != MARC_STATE_IDLE); } while (0)


void delay(int msec) {
	while (msec--) {
		int n = 825;	// determined empirically
		while (n--) {
			asm("NOP");
                }
	}
}


void led_init(void) {
	P1DIR |= (1 << 1);
	P1_1 = 0;
}

void led_toggle(void) {
	P1_1 ^= 1;
}

void led_blink(int i) {
	P1_1 = 0;
        delay(500);
	while (i--) {
          led_toggle();
          delay(100);
          led_toggle();
          delay(100);
        }
        delay(500);
}




void crystal_init() {
  SLEEP &= ~SLEEP_OSC_PD;                        
while (!(SLEEP & SLEEP_XOSC_S));         
CLKCON &= ~CLKCON_OSC;                    
while (CLKCON & SLEEP_OSC_PD); 
SLEEP |= SLEEP_OSC_PD; 
}


void main(void)
{
  crystal_init();
  led_init();
  delay(1000);
  led_blink(3);

  RFOFF;

 
  SYNC1 = 0xD3;       //Sync Word, High Byte 
  SYNC0 = 0x91;       //Sync Word, Low Byte 
  PKTLEN = 0xFF;      //Packet Length 
  PKTCTRL1 = 0x04;    //Packet Automation Control 
  PKTCTRL0 = 0x22;    //Packet Automation Control 
  ADDR = 0x00;        //Device Address 
  CHANNR = 0x00;      //Channel Number 
  FSCTRL1 = 0x06;     //Frequency Synthesizer Control 
  FSCTRL0 = 0x00;     //Frequency Synthesizer Control 
  FREQ2 = 0x24;       //Frequency Control Word, High Byte 
  FREQ1 = 0x2D;       //Frequency Control Word, Middle Byte 
  FREQ0 = 0xDD;       //Frequency Control Word, Low Byte 
  MDMCFG4 = 0xE5;     //Modem configuration 
  MDMCFG3 = 0xA3;     //Modem Configuration 
  MDMCFG2 = 0x30;     //Modem Configuration 
  MDMCFG1 = 0x23;     //Modem Configuration 
  MDMCFG0 = 0x11;     //Modem Configuration 
  DEVIATN = 0x16;     //Modem Deviation Setting 
  MCSM2 = 0x07;       //Main Radio Control State Machine Configuration 
  MCSM1 = MCSM1_TXOFF_MODE_TX; //0x30;       //Main Radio Control State Machine Configuration 
  MCSM0 = 0x18;       //Main Radio Control State Machine Configuration 
  FOCCFG = 0x17;      //Frequency Offset Compensation Configuration 
  BSCFG = 0x6C;       //Bit Synchronization Configuration 
  AGCCTRL2 = 0x03;    //AGC Control 
  AGCCTRL1 = 0x40;    //AGC Control 
  AGCCTRL0 = 0x91;    //AGC Control 
  FREND1 = 0x56;      //Front End RX Configuration 
  FREND0 = 0x11;      //Front End TX Configuration 
  FSCAL3 = 0xE9;      //Frequency Synthesizer Calibration 
  FSCAL2 = 0x2A;      //Frequency Synthesizer Calibration 
  FSCAL1 = 0x00;      //Frequency Synthesizer Calibration 
  FSCAL0 = 0x1F;      //Frequency Synthesizer Calibration 
  TEST2 = 0x88;       //Various Test Settings 
  TEST1 = 0x31;       //Various Test Settings 
  TEST0 = 0x09;       //Various Test Settings 
  PA_TABLE7 = 0x00;   //PA Power Setting 7 
  PA_TABLE6 = 0x00;   //PA Power Setting 6 
  PA_TABLE5 = 0x00;   //PA Power Setting 5 
  PA_TABLE4 = 0x00;   //PA Power Setting 4 
  PA_TABLE3 = 0x00;   //PA Power Setting 3 
  PA_TABLE2 = 0x00;   //PA Power Setting 2 
  PA_TABLE1 = 0xC2;   //PA Power Setting 1 
  PA_TABLE0 = 0xC2;   //PA Power Setting 0 
  IOCFG2 = 0x00;      //Radio Test Signal Configuration (P1_7) 
  IOCFG1 = 0x00;      //Radio Test Signal Configuration (P1_6) 
  IOCFG0 = 0x00;      //Radio Test Signal Configuration (P1_5) 
  PARTNUM = 0x11;     //Chip ID[15:8] 
  VERSION = 0x04;     //Chip ID[7:0] 
  FREQEST = 0x00;     //Frequency Offset Estimate from Demodulator 
  LQI = 0x02;         //Demodulator Estimate for Link Quality 
  RSSI = 0x80;        //Received Signal Strength Indication 
  MARCSTATE = 0x01;   //Main Radio Control State Machine State 
  PKTSTATUS = 0x00;   //Packet Status 
  VCO_VC_DAC = 0x94;  //Current Setting from PLL Calibration Module 
 
  RFTX;
  
    const unsigned char key[16] = {0};
    unsigned char data[16] = {0};


    
  for (;;) {
    led_toggle();
    delay(1000);
    led_toggle();
    
    struct AES_ctx aesContext;
    AES_init_ctx(&aesContext, key);
    
    for (int i = 0; i< 100; i++) {
      AES_ECB_encrypt(&aesContext, data);
    }
        
  }
          
		
}
