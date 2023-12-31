/**************************************************************************************************
  Revised:        $Date: 2007-07-06 11:19:00 -0700 (Fri, 06 Jul 2007) $
  Revision:       $Revision: 13579 $

  Copyright 2007-2009 Texas Instruments Incorporated.  All rights reserved.

  IMPORTANT: Your use of this Software is limited to those specific rights granted under
  the terms of a software license agreement between the user who downloaded the software,
  his/her employer (which must be your employer) and Texas Instruments Incorporated (the
  "License"). You may not use this Software unless you agree to abide by the terms of the
  License. The License limits your use, and you acknowledge, that the Software may not be
  modified, copied or distributed unless embedded on a Texas Instruments microcontroller
  or used solely and exclusively in conjunction with a Texas Instruments radio frequency
  transceiver, which is integrated into your product. Other than for the foregoing purpose,
  you may not use, reproduce, copy, prepare derivative works of, modify, distribute,
  perform, display or sell this Software and/or its documentation for any purpose.

  YOU FURTHER ACKNOWLEDGE AND AGREE THAT THE SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS�
  WITHOUT WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY
  WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
  IN NO EVENT SHALL TEXAS INSTRUMENTS OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER CONTRACT,
  NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE
  THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO ANY
  INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST
  DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY
  THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.

  Should you have any questions regarding your right to use this Software,
  contact Texas Instruments Incorporated at www.TI.com.
**************************************************************************************************/

/* ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 *   MRFI (Minimal RF Interface)
 *   Radios: CC2530
 *   Primary code file for supported radios.
 * ~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=~=
 */

/* ------------------------------------------------------------------------------------------------
 *                                          Includes
 * ------------------------------------------------------------------------------------------------
 */
#include MCU_H
#include <string.h>
#include "mrfi.h"
#include "bsp.h"
#include "bsp_macros.h"
#include "bsp_external/mrfi_board_defs.h"


/* ------------------------------------------------------------------------------------------------
 *                                          Defines
 * ------------------------------------------------------------------------------------------------
 */

#define MRFI_RSSI_OFFSET    -76   /* no units */

/* Wait for RSSI to be valid. */
#define MRFI_RSSI_VALID_WAIT()  while(!(RSSISTAT & BV(0)));

/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *    Values imported from mrfi_defs.h
 *   - - - - - - - - - - - - - - - - - -
 */
#define MRFI_LENGTH_FIELD_OFFSET    __mrfi_LENGTH_FIELD_OFS__
#define MRFI_LENGTH_FIELD_SIZE      __mrfi_LENGTH_FIELD_SIZE__
#define MRFI_HEADER_SIZE            __mrfi_HEADER_SIZE__

#define MRFI_DSN_OFFSET             __mrfi_DSN_OFS__
#define MRFI_FCF_OFFSET             __mrfi_FCF_OFS__

#define MRFI_BACKOFF_PERIOD_USECS   __mrfi_BACKOFF_PERIOD_USECS__

#define MRFI_FCF_0_7              (0x01)
#define MRFI_FCF_8_15             (0x88)
#define MRFI_MIN_SMPL_FRAME_SIZE  (MRFI_HEADER_SIZE + NWK_HDR_SIZE)


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *    Radio Definitions
 *   - - - - - - - - - -
 */

/* immediate strobe processor command instructions */
#define ISRXON        0xE3
#define ISTXON        0xE9
#define ISTXONCCA     0xEA
#define ISRFOFF       0xEF
#define ISFLUSHRX     0xED
#define ISFLUSHTX     0xEE



#if defined(MRFI_CC2430)
#define MRFI_RADIO_PARTNUM        0x85
#define MRFI_RADIO_MIN_VERSION    3      /* minimum version is Rev D */
#elif defined(MRFI_CC2431)
#define MRFI_RADIO_PARTNUM        0x89
#define MRFI_RADIO_MIN_VERSION    4
#elif defined(MRFI_CC2530)
#define MRFI_RADIO_PARTNUM        0xA5
#define MRFI_RADIO_MIN_VERSION    1
#elif defined(MRFI_CC2531)
#define MRFI_RADIO_PARTNUM        0xB5
#define MRFI_RADIO_MIN_VERSION    1
#else
#error ERROR: No part number or radio version defined for radio
#endif

/* rx metrics definitions, known as appended "packet status bytes" in datasheet parlance */
#define MRFI_RX_METRICS_CRC_OK_MASK       __mrfi_RX_METRICS_CRC_OK_MASK__
#define MRFI_RX_METRICS_LQI_MASK          __mrfi_RX_METRICS_LQI_MASK__


/* FSCTRLL */
#define FREQ_2405MHZ    0x0B

/* MDMCTRL0H */
#define ADDR_DECODE     BV(3)

/* FSMSTAT1 */
#define FIFO            BV(7)
#define FIFOP           BV(6)
#define SAMPLED_CCA     BV(3)


/* RFIM */
#define IM_TXDONE       BV(1)

/* IEN2 */
#define RFIE            BV(0)

/* SLEEP */
#define XOSC_STB        BV(6)
#define OSC_PD          BV(2)

/* CLKCON */
#define OSC32K          BV(7)
#define OSC             BV(6)

/* RFPWR */
#define ADI_RADIO_PD    BV(4)
#define RREG_RADIO_PD   BV(3)

/* RFIRQF1 */
#define IRQ_TXDONE      BV(1)

/* RFIRQF0 */
#define IRQ_FIFOP       BV(2)
#define IRQ_SFD         BV(1)

/* RFIRQM1 */
#define IM_TXDONE       BV(1)

/* RFIRQM0 */
#define IM_FIFOP        BV(2)
#define IM_SFD          BV(1)


/* FRMFILT1 */
#define ACCEPT_BEACON   BV(3)
#define ACCEPT_DATA     BV(4)
#define ACCEPT_ACK      BV(5)
#define ACCEPT_CMD      BV(6)


/* FRMCTRL0 */
#define RX_MODE_MASK        (BV(2) | BV(3))
#define RX_MODE_NORMAL      (0x00 << 2)
#define RX_MODE_INFINITE_RX (0x2 << 2)

/* FRMFILT0 */
#define FRAME_FILTER_EN     BV(0)

/* FSMSTATE */
#define FSM_FFCTRL_STATE_RX_INF       31      /* infinite reception state - not documented in datasheet */

/* FRMCTRL1 */
#define RX_ENABLE_ON_TX     BV(0)

/* ADCCON1 */
#define RCTRL1                        BV(3)
#define RCTRL0                        BV(2)
#define RCTRL_BITS                    (RCTRL1 | RCTRL0)
#define RCTRL_CLOCK_LFSR              RCTRL0


/* - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 *     IEEE 802.15.4 definitions
 *   - - - - - - - - - - - - - - -
 */
#define IEEE_PHY_PACKET_SIZE_MASK   0x7F
#define IEEE_USECS_PER_SYMBOL       16

/* Max time we can be in a critical section within the delay function.
 */
#define MRFI_MAX_DELAY_US 16 /* usec */

/* -------------------------------------------------------------------  */
/*  TBD these need to move to board def file */

/* 32 kHz clock source select in CLKCON */
#if !defined (OSC32K_CRYSTAL_INSTALLED) || (defined (OSC32K_CRYSTAL_INSTALLED) && (OSC32K_CRYSTAL_INSTALLED == TRUE))
#define OSC_32KHZ  0x00 /* external 32 KHz xosc */
#else
#define OSC_32KHZ  0x80 /* internal 32 KHz rcosc */
#endif

  /* The SW timer is calibrated by adjusting the call to the microsecond delay
   * routine. This allows maximum calibration control with repects to the longer
   * times requested by applicationsd and decouples internal from external calls
   * to the microsecond routine which can be calibrated independently.
   */
#if defined(SW_TIMER)
#define APP_USEC_VALUE    250
#else
#define APP_USEC_VALUE    1000
#endif

/* ------------------------------------------------------------------------------------------------
 *                                          Macros
 * ------------------------------------------------------------------------------------------------
 */

 /* Flush must be done twice - datasheet. */
#define MRFI_RADIO_FLUSH_RX_BUFFER() {RFST = ISFLUSHRX; RFST = ISFLUSHRX;}

/* ------------------------------------------------------------------------------------------------
 *                                    Global Constants
 * ------------------------------------------------------------------------------------------------
 */
const uint8_t mrfiBroadcastAddr[] = { 0xFF, 0xFF, 0xFF, 0xFF };

/*
 *  Verify number of table entries matches the corresponding #define.
 *  If this assert is hit, most likely the number of initializers in the
 *  above array must be adjusted.
 */
BSP_STATIC_ASSERT(MRFI_ADDR_SIZE == ((sizeof(mrfiBroadcastAddr)/sizeof(mrfiBroadcastAddr[0])) * sizeof(mrfiBroadcastAddr[0])));


/* ------------------------------------------------------------------------------------------------
 *                                    Local Prototypes
 * ------------------------------------------------------------------------------------------------
 */
static void Mrfi_RxModeOn(void);
static void Mrfi_RxModeOff(void);
static void Mrfi_RandomBackoffDelay(void);
static void Mrfi_DelayUsec(uint16_t);
static void Mrfi_DelayUsecSem(uint16_t);
static void Mrfi_RadioRegConfig(void);

/* ------------------------------------------------------------------------------------------------
 *                                    Local Constants
 * ------------------------------------------------------------------------------------------------
 */
/*
 *  Logical channel table - this table translates logical channel into
 *  actual radio channel number.  Channel 0, the default channel, is
 *  determined by the channel exported from SmartRF Studio.  The other
 *  table entries are derived from that default.  Each derived channel is
 *  masked with 0xFF to prevent generation of an illegal channel number.
 *
 *  This table is easily customized.  Just replace or add entries as needed.
 *  If the number of entries changes, the corresponding #define must also
 *  be adjusted.  It is located in mrfi_defs.h and is called __mrfi_NUM_LOGICAL_CHANS__.
 *  The static assert below ensures that there is no mismatch.
 */
static const uint8_t mrfiLogicalChanTable[] =
{
  15,
  20,
  25,
  26
};

/* verify number of table entries matches the corresponding #define */
BSP_STATIC_ASSERT(MRFI_NUM_LOGICAL_CHANS == ((sizeof(mrfiLogicalChanTable)/sizeof(mrfiLogicalChanTable[0])) * sizeof(mrfiLogicalChanTable[0])));

/*
 *  RF Power setting table - this table translates logical power value
 *  to radio register setting.  The logical power value is used directly
 *  as an index into the power setting table. The values in the table are
 *  from low to high. The default settings set 3 values: -20 dBm, -10 dBm,
 *  and 0 dBm. The default at startup is the highest value. Note that these
 *  are approximate depending on the radio. Information is taken from the
 *  data sheet.
 *
 *  This table is easily customized.  Just replace or add entries as needed.
 *  If the number of entries changes, the corresponding #define must also
 *  be adjusted.  It is located in mrfi_defs.h and is called __mrfi_NUM_POWER_SETTINGS__.
 *  The static assert below ensures that there is no mismatch.
 */
static const uint8_t mrfiRFPowerTable[] =
{
  0x25,
  0x75,
  0xD5
};

/* verify number of table entries matches the corresponding #define */
BSP_STATIC_ASSERT(MRFI_NUM_POWER_SETTINGS == ((sizeof(mrfiRFPowerTable)/sizeof(mrfiRFPowerTable[0])) * sizeof(mrfiRFPowerTable[0])));

/* ------------------------------------------------------------------------------------------------
 *                                       Local Variables
 * ------------------------------------------------------------------------------------------------
 */
static uint8_t mrfiRadioState  = MRFI_RADIO_STATE_UNKNOWN;
static mrfiPacket_t mrfiIncomingPacket;

/* reply delay support */
static volatile uint8_t  sReplyDelayContext = 0;
static volatile uint8_t  sKillSem = 0;
static          uint16_t sReplyDelayScalar = 0;

#define CORR_THR                      0x14
#define CCA_THR                       0xF8
#define TXFILTCFG                     XREG( 0x61FA )
#define TXFILTCFG_RESET_VALUE         0x09
/**************************************************************************************************
 * @fn          Mrfi_RadioRegConfig
 *
 * @brief       Initialize Radio registers that should be different from their
 *              reset values.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void Mrfi_RadioRegConfig(void)
{
  /* This CORR_THR value should be changed to 0x14 before attempting RX. Testing has shown that
   * too many false frames are received if the reset value is used. Make it more likely to detect
   * sync by removing the requirement that both symbols in the SFD must have a correlation value
   * above the correlation threshold, and make sync word detection less likely by raising the
   * correlation threshold.
   */
  MDMCTRL1 = CORR_THR;

  /* tuning adjustments for optimal radio performance; details available in datasheet */
  RXCTRL = 0x3F;

  /* Raises the CCA threshold from about -108dBm to about -80 dBm input level.
   */
  CCACTRL0 = CCA_THR;

  /* Makes sync word detection less likely by requiring two zero symbols before the sync word.
   * details available in datasheet.
   */
  MDMCTRL0 = 0x85;

  /* Adjust currents in synthesizer; details available in datasheet. */
  FSCTRL = 0x5A;

  /* Adjust target value for AGC control loop; details available in datasheet. */
  AGCCTRL1 = 0x15;

  /* Disable source address matching */
  SRCMATCH = 0;

  /* Tune ADC performance, details available in datasheet. */
  ADCTEST0 = 0x10;
  ADCTEST1 = 0x0E;
  ADCTEST2 = 0x03;

 /* Sets TX anti-aliasing filter to appropriate bandwidth.
  * Reduces spurious emissions close to signal.
  */
  TXFILTCFG = TXFILTCFG_RESET_VALUE;

  /* disable the CSPT register compare function */
  CSPT = 0xFF;
}

/**************************************************************************************************
 * @fn          MRFI_Init
 *
 * @brief       Initialize MRFI.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_Init(void)
{
  /* ------------------------------------------------------------------
   *    Run-time integrity checks
   *   ---------------------------
   */
  memset(&mrfiIncomingPacket, 0x0, sizeof(mrfiIncomingPacket));

  /* verify the correct radio is installed */
  MRFI_ASSERT( CHIPID == MRFI_RADIO_PARTNUM );      /* wrong radio */
  MRFI_ASSERT( CHVER  >= MRFI_RADIO_MIN_VERSION );  /* obsolete radio version */

  /* ------------------------------------------------------------------
   *    Configure IO ports
   *   ---------------------------
   */

#if defined(MRFI_PA_LNA_ENABLED) && defined(BSP_BOARD_SRF04EB)
  MRFI_BOARD_PA_LNA_CONFIG_PORTS();
  MRFI_BOARD_PA_LNA_HGM();
#endif


  /* ------------------------------------------------------------------
   *    Configure clock to use XOSC
   *   -----------------------------
   */
  SLEEPCMD &= ~OSC_PD;                       /* turn on 16MHz RC and 32MHz XOSC */
  while (!(SLEEPSTA & XOSC_STB));            /* wait for 32MHz XOSC stable */
  asm("NOP");                             /* chip bug workaround */
  {
    uint16_t i;

    /* Require 63us delay for all revs */
    for (i=0; i<504; i++)
    {
      asm("NOP");
    }
  }
  CLKCONCMD = (0x00 | OSC_32KHZ);            /* 32MHz XOSC */
  while (CLKCONSTA != (0x00 | OSC_32KHZ));
  SLEEPCMD |= OSC_PD;                        /* turn off 16MHz RC */


  /* Configure radio registers that should be different from reset values. */
  Mrfi_RadioRegConfig();

  /* ------------------------------------------------------------------
   *    Variable Initialization
   *   -------------------------
   */

#ifdef MRFI_ASSERTS_ARE_ON
  PAN_ID0 = 0xFF;
  PAN_ID1 = 0xFF;
#endif



  /* ------------------------------------------------------------------
   *    Initialize Random Seed Value
   *   -------------------------------
   */


  /*
   *  Set radio for infinite reception.  Once radio reaches this state,
   *  it will stay in receive mode regardless RF activity.
   */
  FRMCTRL0 = (FRMCTRL0 & ~RX_MODE_MASK) | RX_MODE_INFINITE_RX;

  /* turn on the receiver */
  RFST = ISRXON;

  /* Wait for RSSI to be valid. Once valid, radio is stable and random bits
   * can be read.
   */
  MRFI_RSSI_VALID_WAIT();

  /* put 16 random bits into the seed value */
  {
    uint16_t rndSeed;
    uint8_t  i;

    rndSeed = 0;

    for(i=0; i<16; i++)
    {
      /* read random bit to populate the random seed */
      rndSeed = (rndSeed << 1) | (RFRND & 0x01);
    }

    /*
     *  The seed value must not be zero.  If it is, the pseudo random sequence will be always be zero.
     *  There is an extremely small chance this seed could randomly be zero (more likely some type of
     *  hardware problem would cause this).  To solve this, a single bit is forced to be one.  This
     *  slightly reduces the randomness but guarantees a good seed value.
     */
    rndSeed |= 0x0080;

    /*
     *  Two writes to RNDL will set the random seed.  A write to RNDL copies current contents
     *  of RNDL to RNDH before writing new the value to RNDL.
     */
    RNDL = rndSeed & 0xFF;
    RNDL = rndSeed >> 8;
  }

  /* turn off the receiver, flush RX FIFO just in case something got in there */
  RFST = ISRFOFF;

  /* flush the rx buffer */
  MRFI_RADIO_FLUSH_RX_BUFFER();

  /* take receiver out of infinite reception mode; set back to normal operation */
  FRMCTRL0 = (FRMCTRL0 & ~RX_MODE_MASK) | RX_MODE_NORMAL;


  /* Initial radio state is OFF state */
  mrfiRadioState = MRFI_RADIO_STATE_OFF;

  /* ------------------------------------------------------------------
   *    Configure Radio Registers
   *   ---------------------------
   */

  /* disable address filtering */
  FRMFILT0 &= ~FRAME_FILTER_EN;

  /* reject beacon/ack/cmd frames and accept only data frames,
   * when filtering is enabled.
   */
  FRMFILT1 &= ~(ACCEPT_BEACON | ACCEPT_ACK | ACCEPT_CMD);

  /* don't enable rx after tx is done. */
  FRMCTRL1 &= ~RX_ENABLE_ON_TX;

  /* set FIFOP threshold to maximum */
  FIFOPCTRL = 127;

  /* set default channel */
  MRFI_SetLogicalChannel( 0 );

  /* set default output power level */
  MRFI_SetRFPwr(MRFI_NUM_POWER_SETTINGS - 1);

  /* enable general RF interrupts */
  IEN2 |= RFIE;


  /* ------------------------------------------------------------------
   *    Final Initialization
   *   -----------------------
   */


  /**********************************************************************************
   *                            Compute reply delay scalar
   *
   * The IEEE radio has a fixed data rate of 250 Kbps. Data rate inference
   * from radio regsiters is not necessary for this radio.
   *
   * The maximum delay needed depends on the MAX_APP_PAYLOAD parameter. Figure
   * out how many bits that will be when overhead is included. Bits/bits-per-second
   * is seconds to transmit (or receive) the maximum frame. We multiply this number
   * by 1000 to find the time in milliseconds. We then additionally multiply by
   * 10 so we can add 5 and divide by 10 later, thus rounding up to the number of
   * milliseconds. This last won't matter for slow transmissions but for faster ones
   * we want to err on the side of being conservative and making sure the radio is on
   * to receive the reply. The semaphore monitor will shut it down. The delay adds in
   * a platform fudge factor that includes processing time on peer plus lags in Rx and
   * processing time on receiver's side. Also includes round trip delays from CCA
   * retries. This portion is included in PLATFORM_FACTOR_CONSTANT defined in mrfi.h.
   *
   * **********************************************************************************
   */

#define   PHY_PREAMBLE_SYNC_BYTES    8

  {
    uint32_t bits, dataRate = 250000;

    bits = ((uint32_t)((PHY_PREAMBLE_SYNC_BYTES + MRFI_MAX_FRAME_SIZE)*8))*10000;

    /* processing on the peer + the Tx/Rx time plus more */
    sReplyDelayScalar = PLATFORM_FACTOR_CONSTANT + (((bits/dataRate)+5)/10);
  }

  /*
   *  Random delay - This prevents devices on the same power source from repeated
   *  transmit collisions on power up.
   */
  Mrfi_RandomBackoffDelay();

  /* enable global interrupts */
  BSP_ENABLE_INTERRUPTS();
}


/**************************************************************************************************
 * @fn          MRFI_Transmit
 *
 * @brief       Transmit a packet using CCA algorithm.
 *
 * @param       pPacket - pointer to packet to transmit
 *
 * @return      Return code indicates success or failure of transmit:
 *                  MRFI_TX_RESULT_SUCCESS - transmit succeeded
 *                  MRFI_TX_RESULT_FAILED  - transmit failed because CCA failed
 **************************************************************************************************
 */
uint8_t MRFI_Transmit(mrfiPacket_t * pPacket, uint8_t txType)
{
  static uint8_t dsn = 0;
  uint8_t txResult = MRFI_TX_RESULT_SUCCESS;

  /* radio must be awake to transmit */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );


  /* ------------------------------------------------------------------
   *    Initialize hardware for transmit
   *   -----------------------------------
   */

  /* turn off reciever */
  Mrfi_RxModeOff();

  /* clear 'transmit done' interrupt flag, this bit is tested to see when transmit completes */
  RFIRQF1 &= ~IRQ_TXDONE;


  /* ------------------------------------------------------------------
   *    Populate the IEEE fields in frame
   *   ------------------------------------
   */

  /* set the sequence number, also known as DSN (Data Sequence Number) */
  pPacket->frame[MRFI_DSN_OFFSET] = dsn;

  /* increment the sequence number, value is retained (static variable) for use in next transmit */
  dsn++;

  /*
   *  Populate the FCF (Frame Control Field) with the following settings.
   *
   *    bits    description                         setting
   *  --------------------------------------------------------------------------------------
   *      0-2   Frame Type                          001 - data frame
   *        3   Security Enabled                      0 - security disabled
   *        4   Frame Pending                         0 - no pending data
   *        5   Ack Request                           0 - no Ack request
   *        6   PAN ID Compression                    0 - no PAN ID compression
   *        7   Reserved                              0 - reserved
   *  - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
   *      8-9   Reserved                             00 - reserved
   *    10-11   Destination Addressing Mode          10 - PAN ID + 16-bit short address
   *    12-13   Frame Version                        00 - IEEE Std 802.15.4-2003
   *    14-15   Source Addressing Mode               10 - PAN ID + 16-bit short address
   *
   */
  pPacket->frame[MRFI_FCF_OFFSET]   = MRFI_FCF_0_7;
  pPacket->frame[MRFI_FCF_OFFSET+1] = MRFI_FCF_8_15;


  /* ------------------------------------------------------------------
   *    Write packet to transmit FIFO
   *   --------------------------------
   */
  {
    uint8_t txBufLen;
    uint8_t * p;
    uint8_t i;

    /* flush FIFO of any previous transmit that did not go out */
    RFST = ISFLUSHTX;

    /* set point at beginning of outgoing frame */
    p = &pPacket->frame[MRFI_LENGTH_FIELD_OFFSET];

    /* get number of bytes in the packet (does not include the length byte) */
    txBufLen = *p;

    /*
     *  Write the length byte to the FIFO.  This length does *not* include the length field
     *  itself but does include the size of the FCS (generically known as RX metrics) which
     *  is generated automatically by the radio.
     */
    RFD = txBufLen + MRFI_RX_METRICS_SIZE;

    /* write packet bytes to FIFO */
    for (i=0; i<txBufLen; i++)
    {
      p++;
      RFD = *p;
    }
  }

  /* ------------------------------------------------------------------
   *    Immediate transmit
   *   ---------------------
   */
  if (txType == MRFI_TX_TYPE_FORCED)
  {
    /* strobe transmit */
    RFST = ISTXON;

    /* wait for transmit to complete */
    while (!(RFIRQF1 & IRQ_TXDONE));

    /* transmit is done */
  }
  else
  {
    /* ------------------------------------------------------------------
     *    CCA transmit
     *   ---------------
     */
    MRFI_ASSERT( txType == MRFI_TX_TYPE_CCA );

    {
      bspIState_t s;
      uint8_t txActive;
      uint8_t ccaRetries;

      /* set number of CCA retries */
      ccaRetries = MRFI_CCA_RETRIES;


      /* ===============================================================================
       *    CCA Algorithm Loop
       *   ====================
       */
      for (;;)
      {
        /* Turn ON the receiver to perform CCA. Can not call Mrfi_RxModeOn(),
         * since that will enable the rx interrupt, which we do not want.
         */
        RFST = ISRXON;

        /*
         *  Wait for CCA to be valid.
         */
        MRFI_RSSI_VALID_WAIT();

        /*
         *  Initiate transmit with CCA.  Command is strobed and then status is
         *  immediately checked.  If status shows transmit is active, this means
         *  that CCA passed and the transmit has gone out.  A critical section
         *  guarantees timing status check happens immediately after strobe.
         */
        BSP_ENTER_CRITICAL_SECTION(s);
        RFST = ISTXONCCA;
        txActive = FSMSTAT1 & SAMPLED_CCA;
        BSP_EXIT_CRITICAL_SECTION(s);

        /* see transmit went out */
        if (txActive)
        {
          /* ----------|  CCA Passed |---------- */

          /* wait for transmit to complete */
          while (!(RFIRQF1 & IRQ_TXDONE));

          /* transmit is done. break out of CCA algorithm loop */
          break;
        }
        else
        {
          /* ----------|  CCA Failed |---------- */

          /* if no CCA retries are left, transmit failed so abort */
          if (ccaRetries == 0)
          {
            /* set return value for failed transmit */
            txResult = MRFI_TX_RESULT_FAILED;

            /* break out of CCA algorithm loop */
            break;
          }

          /* decrement CCA retries before loop continues */
          ccaRetries--;

          /* turn off reciever to conserve power during backoff */
          Mrfi_RxModeOff();

          /* delay for a random number of backoffs */
          Mrfi_RandomBackoffDelay();
        }
      }
      /*
       *  ---  end CCA Algorithm Loop ---
       * =============================================================================== */
    }
  }


  /* turn radio back off to put it in a known state */
  Mrfi_RxModeOff();

  /* If the radio was in RX state when transmit was attempted,
   * put it back in RX state.
   */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }

  /* return the result of the transmit */
  return( txResult );
}


/**************************************************************************************************
 * @fn          MRFI_Receive
 *
 * @brief       Copies last packet received to the location specified.
 *              This function is meant to be called after the ISR informs
 *              higher level code that there is a newly received packet.
 *
 * @param       pPacket - pointer to location of where to copy received packet
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_Receive(mrfiPacket_t * pPacket)
{
  /* copy last received packed into indicated memory */
  *pPacket = mrfiIncomingPacket;
}


/**************************************************************************************************
 * @fn          MRFI_RxOn
 *
 * @brief       Turn on the receiver.  No harm is done if this function is called when
 *              receiver is already on.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_RxOn(void)
{
  /* radio must be awake before we can move it to RX state */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* Put the radio in RX state, if not already */
  if(mrfiRadioState != MRFI_RADIO_STATE_RX)
  {
    mrfiRadioState = MRFI_RADIO_STATE_RX;
    Mrfi_RxModeOn();
  }
}


/**************************************************************************************************
 * @fn          MRFI_RxIdle
 *
 * @brief       Put radio in idle mode (receiver if off).  No harm is done this function is
 *              called when radio is already idle.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_RxIdle(void)
{
  /* radio must be awake to move it to idle mode */
  MRFI_ASSERT( mrfiRadioState != MRFI_RADIO_STATE_OFF );

  /* if radio is on, turn it off */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOff();
    mrfiRadioState = MRFI_RADIO_STATE_IDLE;
  }
}


/**************************************************************************************************
 * @fn          MRFI_RxIsr
 *
 * @brief       Receive interrupt.  Reads incoming packet from radio FIFO.  If CRC passes the
 *              external function MRFI_RxCompleteISR() is called.
 *
 *              Note : All RF interrupts use this same interrupt vector.  Normally, the interrupt
 *              enable bits and interrupt flag bits are examined to see which interrupts need to
 *              be serviced.  In this implementation, only the FIFOP interrupt is used.  This
 8              function is optimized to take advantage of that fact.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
BSP_ISR_FUNCTION( MRFI_RxIsr, RF_VECTOR )
{
  uint8_t numBytes;
  uint8_t i, crcOK;

  /* We should receive this interrupt only in RX state
   * Should never receive it if RX was turned On only for
   * some internal mrfi processing like - during CCA.
   * Otherwise something is terribly wrong.
   */
  MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );

  /* While there is stuff in the Rx FIFO... */
  do {
    /*
     * Pend on frame completion. First timne through this always passes.
     * Later, though, it is possible that the Rx FIFO has bytes but we
     * havn't received a complete frame yet.
     */
    while (!(RFIRQF0 & IRQ_FIFOP)) ;

    /* Check for Rx overflow. Checking here means we may flush a valid frame */
    if ((FSMSTAT1 & FIFOP) && (!(FSMSTAT1 & FIFO)))
    {
      /* flush receive FIFO to recover from overflow (per datasheet, flush must be done twice) */
      MRFI_RADIO_FLUSH_RX_BUFFER();
      break;
    }

    /* clear interrupt flag so we can detect another frame later. */
    RFIRQF0 &= ~IRQ_FIFOP;

    /* ------------------------------------------------------------------
     *    Read packet from FIFO
     *   -----------------------
     */

    /*
     *  Determine number of bytes to be read from receive FIFO.  The first byte
     *  has the number of bytes in the packet.  A mask must be applied though
     *  to strip off unused bits.  The number of bytes in the packet does not
     *  include the length byte itself but does include the FCS (generically known
     *  as RX metrics).
     */
    numBytes = RFD & IEEE_PHY_PACKET_SIZE_MASK;

    /* see if frame will fit in maximum available buffer or is too small */
    if (((numBytes + MRFI_LENGTH_FIELD_SIZE - MRFI_RX_METRICS_SIZE) > MRFI_MAX_FRAME_SIZE) ||
         (numBytes < MRFI_MIN_SMPL_FRAME_SIZE))
    {
      /* packet is too big or too small. remove it from FIFO */
      for (i=0; i<numBytes; i++)
      {
        /* read and discard bytes from FIFO */
        RFD;
      }
    }
    else
    {
      uint8_t *p, *p1;

      /* set pointer at first byte of frame storage */
      p  = &mrfiIncomingPacket.frame[MRFI_LENGTH_FIELD_OFFSET];
      p1 = mrfiIncomingPacket.frame;

      /* Clear out my buffer to remove leftovers in case a bogus packet gets through */
      memset(p1, 0x0, sizeof(mrfiIncomingPacket.frame));

      /*
       *  Store frame length into the incoming packet memory.  Size of rx metrics
       *  is subtracted to get the MRFI frame length which separates rx metrics from
       *  the frame length.
       */
      *p = numBytes - MRFI_RX_METRICS_SIZE;

      /* read frame bytes from receive FIFO and store into incoming packet memory */
      for (i=0; i<numBytes-MRFI_RX_METRICS_SIZE; i++)
      {
        p++;
        *p = RFD;
      }

      /* read rx metrics and store to incoming packet */

      /* Add the RSSI offset to get the proper RSSI value. */
      mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_RSSI_OFS] = RFD + MRFI_RSSI_OFFSET;

      /* The second byte has 7 bits of Correlation value and 1 bit of
       * CRC pass/fail info. Remove the CRC pass/fail bit info.
       * Also note that for CC2430 radio this is the correlation value and not
       * the LQI value. Some convertion is needed to extract the LQI value.
       * This convertion is left to the application at this time.
       */
      crcOK = RFD;   /* get CRC/LQI byte */
      if (!(crcOK & (~MRFI_RX_METRICS_LQI_MASK)))
      {
        /* bad CRC. Move on... */
        continue;
      }

      /* CRC OK. Save LQI info */
      mrfiIncomingPacket.rxMetrics[MRFI_RX_METRICS_CRC_LQI_OFS] = (crcOK & MRFI_RX_METRICS_LQI_MASK);

      /* Eliminate frames that are the correct size but we can tell are bogus
       * by their frame control fields.
       */
      if ((p1[MRFI_FCF_OFFSET] == MRFI_FCF_0_7) &&
          (p1[MRFI_FCF_OFFSET+1] == MRFI_FCF_8_15))
      {
        /* call external, higher level "receive complete" */
        MRFI_RxCompleteISR();
      }
    }
  } while (RXFIFOCNT);


  /* ------------------------------------------------------------------
   *     Clean up on exit
   *   --------------------
   */

  /* Clear FIFOP interrupt:
   * This is an edge triggered interrupt. The interrupt must be first cleared
   * at the MCU before re-enabling the interrupt at the source.
   */
  S1CON = 0x00; /* Clear the interrupt at MCU. */

  RFIRQF0 &= ~IRQ_FIFOP; /* Clear the interrupt source flag. */
}


/**************************************************************************************************
 * @fn          MRFI_SetLogicalChannel
 *
 * @brief       Set logical channel.
 *
 * @param       chan - logical channel number
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_SetLogicalChannel(uint8_t chan)
{
  uint8_t phyChannel;

  /* logical channel is not valid */
  MRFI_ASSERT( chan < MRFI_NUM_LOGICAL_CHANS );

  /* make sure radio is off before changing channels */
  Mrfi_RxModeOff();

  /* convert logical channel number into physical channel number */
  phyChannel = mrfiLogicalChanTable[chan];

  /* write frequency value of new channel */
  FREQCTRL  = FREQ_2405MHZ + (5 * (phyChannel - 11));

  /* Put the radio back in RX state, if it was in RX before channel change */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }
}

/**************************************************************************************************
 * @fn          MRFI_SetRFPwr
 *
 * @brief       Set RF output power.
 *
 * @param       level - Power level
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_SetRFPwr(uint8_t level)
{

  /* make sure radio is off before changing power level */
  Mrfi_RxModeOff();

  TXPOWER = mrfiRFPowerTable[level];

  /* Put the radio back in RX state, if it was in RX before channel change */
  if(mrfiRadioState == MRFI_RADIO_STATE_RX)
  {
    Mrfi_RxModeOn();
  }
}
/**************************************************************************************************
 * @fn          MRFI_WakeUp
 *
 * @brief       Wake up radio from sleep state.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_WakeUp(void)
{
  /* if radio is asleep, wake it up */
  if(mrfiRadioState == MRFI_RADIO_STATE_OFF)
  {
    /* enter idle mode */
    mrfiRadioState = MRFI_RADIO_STATE_IDLE;

    /* Nothing to be done on radio. */
  }
}


/**************************************************************************************************
 * @fn          MRFI_Sleep
 *
 * @brief       Request radio go to sleep.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_Sleep(void)
{
  /* If radio is not asleep, put it to sleep */
  if(mrfiRadioState != MRFI_RADIO_STATE_OFF)
  {
    /* go to idle so radio is in a known state before sleeping */
    MRFI_RxIdle();

    /* Our new state is OFF */
    mrfiRadioState = MRFI_RADIO_STATE_OFF;
  }
}


/**************************************************************************************************
 * @fn          MRFI_SetRxAddrFilter
 *
 * @brief       Set the address used for filtering received packets.
 *
 * @param       pAddr - pointer to address to use for filtering
 *
 * @return      none
 **************************************************************************************************
 */
uint8_t MRFI_SetRxAddrFilter(uint8_t * pAddr)
{
  /*
   *  Determine if filter address is valid.  Cannot be a reserved value.
   *   -Reserved PAN ID's of 0xFFFF and 0xFFFE.
   *   -Reserved short address of 0xFFFF.
   */
  if ((((pAddr[0] == 0xFF) || (pAddr[0] == 0xFE)) && (pAddr[1] == 0xFF))  ||
        (pAddr[2] == 0xFF) && ((pAddr[3] == 0xFF)))
  {
    /* unable to set filter address */
    return( 1 );
  }

  /* set the hardware address registers */
  PAN_ID0     = pAddr[0];
  PAN_ID1     = pAddr[1];
  SHORT_ADDR0 = pAddr[2];
  SHORT_ADDR1 = pAddr[3];

  /* successfully set filter address */
  return( 0 );
}


/**************************************************************************************************
 * @fn          MRFI_EnableRxAddrFilter
 *
 * @brief       Enable received packet filtering.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_EnableRxAddrFilter(void)
{
  MRFI_ASSERT( (PAN_ID0 != 0xFF) && (PAN_ID1 != 0xFF) ); /* filter address not set */

  /* enable hardware filtering on the radio */
  FRMFILT0 |= FRAME_FILTER_EN;
}


/**************************************************************************************************
 * @fn          MRFI_DisableRxAddrFilter
 *
 * @brief       Disable received packet filtering.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_DisableRxAddrFilter(void)
{
  /* disable hardware filtering on the radio */
  FRMFILT0 &= ~FRAME_FILTER_EN;
}


/**************************************************************************************************
 * @fn          MRFI_Rssi
 *
 * @brief       Returns "live" RSSI value
 *
 * @param       none
 *
 * @return      RSSI value in units of dBm.
 **************************************************************************************************
 */
int8_t MRFI_Rssi(void)
{
  int8_t rssi;

  /* Radio must be in RX state to measure rssi. */
  MRFI_ASSERT( mrfiRadioState == MRFI_RADIO_STATE_RX );

  /*
   *  Assuming that the Radio was just turned on, we must wait for
   *  RSSI to be valid.
   */
  MRFI_RSSI_VALID_WAIT();

  /* read RSSI value from hardware */
  rssi = RSSI;

  /* apply offset given in datasheet */
  rssi = rssi + MRFI_RSSI_OFFSET;

  /* return RSSI value */
  return( rssi );
}


/**************************************************************************************************
 * @fn          MRFI_RandomByte
 *
 * @brief       Returns a random byte using a special hardware feature that generates new
 *              random values based on the truly random seed set earlier.
 *
 * @param       none
 *
 * @return      a random byte
 **************************************************************************************************
 */
uint8_t MRFI_RandomByte(void)
{
  /* clock the random generator to get a new random value */
  ADCCON1 = (ADCCON1 & ~RCTRL_BITS) | RCTRL_CLOCK_LFSR;

  /* return newly randomized value from hardware */
  return(RNDH);
}


/**************************************************************************************************
 * @fn          MRFI_DelayMs
 *
 * @brief       Delay the specified number of milliseconds.
 *
 * @param       milliseconds - delay time
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_DelayMs(uint16_t milliseconds)
{
  while (milliseconds)
  {
    Mrfi_DelayUsec( APP_USEC_VALUE );
    milliseconds--;
  }
}

/**************************************************************************************************
 * @fn          MRFI_ReplyDelay
 *
 * @brief       Delay number of milliseconds scaled by data rate. Check semaphore for
 *              early-out. Run in a separate thread when the reply delay is
 *              invoked. Cleaner then trying to make MRFI_DelayMs() thread-safe
 *              and reentrant.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_ReplyDelay()
{
  bspIState_t s;
  uint16_t    milliseconds = sReplyDelayScalar;

  BSP_ENTER_CRITICAL_SECTION(s);
  sReplyDelayContext = 1;
  BSP_EXIT_CRITICAL_SECTION(s);

  while (milliseconds)
  {
    Mrfi_DelayUsecSem( APP_USEC_VALUE );
    if (sKillSem)
    {
      break;
    }
    milliseconds--;
  }

  BSP_ENTER_CRITICAL_SECTION(s);
  sKillSem           = 0;
  sReplyDelayContext = 0;
  BSP_EXIT_CRITICAL_SECTION(s);
}

/**************************************************************************************************
 * @fn          MRFI_PostKillSem
 *
 * @brief       Post to the loop-kill semaphore that will be checked by the iteration loops
 *              that control the delay thread.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
void MRFI_PostKillSem(void)
{
  if (sReplyDelayContext)
  {
    sKillSem = 1;
  }

  return;
}

/**************************************************************************************************
 * @fn          Mrfi_RxModeOn
 *
 * @brief       Put radio into receive mode.
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RxModeOn(void)
{
  /* send strobe to enter receive mode */
  RFST = ISRXON;

  /* enable receive interrupts */
  RFIRQM0 |= IM_FIFOP;
}


/**************************************************************************************************
 * @fn          Mrfi_RxModeOff
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RxModeOff(void)
{
  /*disable receive interrupts */
  RFIRQM0 &= ~IM_FIFOP;

  /* turn off radio */
  RFST = ISRFOFF;

  /* flush the receive FIFO of any residual data */
  MRFI_RADIO_FLUSH_RX_BUFFER();

  /* clear receive interrupt */
  RFIRQF0 = ~IRQ_FIFOP;
}


/**************************************************************************************************
 * @fn          Mrfi_RandomBackoffDelay
 *
 * @brief       -
 *
 * @param       none
 *
 * @return      none
 **************************************************************************************************
 */
static void Mrfi_RandomBackoffDelay(void)
{
  uint8_t backoffs;
  uint8_t i;

  /* calculate random value for backoffs - 1 to 16 */
  backoffs = (MRFI_RandomByte() & 0x0F) + 1;

  /* delay for randomly computed number of backoff periods */
  for (i=0; i<backoffs; i++)
  {
    Mrfi_DelayUsec( MRFI_BACKOFF_PERIOD_USECS );
  }
}

/****************************************************************************************************
 * @fn          Mrfi_DelayUsec
 *
 * @brief       Execute a delay loop using HW timer. The macro actually used to do the delay
 *              is not thread-safe. This routine makes the delay execution thread-safe by breaking
 *              up the requested delay into small chunks and executing each chunk as a critical
 *              section. The chunk size is choosen to be the smallest value used by MRFI. The delay
 *              is only approximate because of the overhead computations. It errs on the side of
 *              being too long.
 *
 * input parameters
 * @param   howLong - number of microseconds to delay
 *
 * @return      none
 ****************************************************************************************************
 */
static void Mrfi_DelayUsec(uint16_t howLong)
{
  bspIState_t intState;
  uint16_t    count = howLong/MRFI_MAX_DELAY_US;

  if (howLong)
  {
    do
    {
      BSP_ENTER_CRITICAL_SECTION(intState);
      BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
      BSP_EXIT_CRITICAL_SECTION(intState);
    } while (count--);
  }

  return;
}

/****************************************************************************************************
 * @fn          Mrfi_DelayUsecSem
 *
 * @brief       Execute a delay loop using a HW timer. See comments for Mrfi_DelayUsec().
 *              Delay specified number of microseconds checking semaphore for
 *              early-out. Run in a separate thread when the reply delay is
 *              invoked. Cleaner then trying to make MRFI_DelayUsec() thread-safe
 *              and reentrant.
 *
 * input parameters
 * @param   howLong - number of microseconds to delay
 *
 * @return      none
 ****************************************************************************************************
 */
static void Mrfi_DelayUsecSem(uint16_t howLong)
{
  bspIState_t s;
  uint16_t count = howLong/MRFI_MAX_DELAY_US;

  if (howLong)
  {
    do
    {
      BSP_ENTER_CRITICAL_SECTION(s);
      BSP_DELAY_USECS(MRFI_MAX_DELAY_US);
      BSP_EXIT_CRITICAL_SECTION(s);
      if (sKillSem)
      {
        break;
      }
    } while (count--);
  }

  return;
}

/**************************************************************************************************
 * @fn          MRFI_GetRadioState
 *
 * @brief       Returns the current radio state.
 *
 * @param       none
 *
 * @return      radio state - off/idle/rx
 **************************************************************************************************
 */
uint8_t MRFI_GetRadioState(void)
{
  return mrfiRadioState;
}

/**************************************************************************************************
 *                                  Compile Time Integrity Checks
 **************************************************************************************************
 */

/*
 *  The current implementation requires an address size of four bytes.  These four bytes are
 *  spread across the PAN ID and the short address.  A larger address size is possible by using
 *  long address instead of short address.  The change is not difficult but it does require
 *  code modification.
 */
#if (MRFI_ADDR_SIZE != 4)
#error "ERROR:  Address size must be four bytes.  A different address size requires code modification."
#endif

 #define MRFI_RADIO_TX_FIFO_SIZE     128  /* from datasheet */

/* verify largest possible packet fits within FIFO buffer */
#if ((MRFI_MAX_FRAME_SIZE + MRFI_RX_METRICS_SIZE) > MRFI_RADIO_TX_FIFO_SIZE)
#error "ERROR:  Maximum possible packet length exceeds FIFO buffer.  Decrease value of maximum application payload."
#endif

/*
 *  These asserts happen if there is extraneous compiler padding of arrays.
 *  Modify compiler settings for no padding, or, if that is not possible,
 *  comment out the offending asserts.
 */
BSP_STATIC_ASSERT(sizeof(mrfiLogicalChanTable) == ((sizeof(mrfiLogicalChanTable)/sizeof(mrfiLogicalChanTable[0])) * sizeof(mrfiLogicalChanTable[0])));
BSP_STATIC_ASSERT(sizeof(mrfiBroadcastAddr) == ((sizeof(mrfiBroadcastAddr)/sizeof(mrfiBroadcastAddr[0])) * sizeof(mrfiBroadcastAddr[0])));


/**************************************************************************************************
*/

