# rfEasyLinkEcho
---

### SysConfig Notice

All examples will soon be supported by SysConfig, a tool that will help you 
graphically configure your software components. A preview is available today in 
the examples/syscfg_preview directory. Starting in 3Q 2019, with SDK version 
3.30, only SysConfig-enabled versions of examples will be provided. For more 
information, click [here](http://www.ti.com/sysconfignotice).

-------------------------

Project Setup using the System Configuration Tool (SysConfig)
-------------------------
The purpose of SysConfig is to provide an easy to use interface for configuring 
drivers, RF stacks, and more. The .syscfg file provided with each example 
project has been configured and tested for that project. Changes to the .syscfg 
file may alter the behavior of the example away from default. Some parameters 
configured in SysConfig may require the use of specific APIs or additional 
modifications in the application source code. More information can be found in 
SysConfig by hovering over a configurable and clicking the question mark (?) 
next to it's name.

### EasyLink Stack Configuration
Many parameters of the EasyLink stack can be configured using SysConfig 
including RX, TX, Radio, and Advanced settings. More information can be found in 
SysConfig by hovering over a configurable and clicking the question mark (?) 
next to it's name. Alternatively, refer to the System Configuration Tool 
(SysConfig) section of the Proprietary RF User's guide found in 
&lt;SDK_INSTALL_DIR&gt;/docs/proprietary-rf/proprietary-rf-users-guide.html. 

Example Summary
---------------
This example demonstrates the use of the EasyLink API in doing bi-directional 
communication. It will require the use of two boards, one running 
the rfEasyLinkEchoTx project that will originate the packets, while another 
board running the rfEasyLinkEchoRx project will re-transmit (echo) them back 
to the originator.

For more information on the EasyLink API and usage refer to the [Proprietary RF User's guide](http://dev.ti.com/tirex/#/?link=Software%2FSimpleLink%20CC13x2%2026x2%20SDK%2FDocuments%2FProprietary%20RF%2FProprietary%20RF%20User's%20Guide)

Peripherals Exercised
---------------------
For the rfEasyLinkEchoTx (`Board_1`) project,
* `Board_PIN_LED1` - Blinking indicates a successful transmission and reception
  of a packet (echo)
* `Board_PIN_LED2` - Indicates an abort occurred in packet reception (waiting 
  for the echo)
* `Board_PIN_LED1` & `Board_PIN_LED2` indicate an error condition

For the rfEasyLinkEchoRx (`Board_2`) project,
* `Board_PIN_LED2` - Blinking indicates a successful reception and 
  re-transmission of a packet (echo)
* `Board_PIN_LED1` - Indicates an error in either reception of a packet or 
  in its re-transmission

Resources & Jumper Settings
---------------------------
> If you're using an IDE (such as CCS or IAR), please refer to Board.html in 
your project directory for resources used and board-specific jumper settings. 
Otherwise, you can find Board.html in the directory 
&lt;SDK_INSTALL_DIR&gt;/source/ti/boards/&lt;BOARD&gt;.

Example Usage
-------------
The user will require two launchpads, one running rfEasyLinkEchoTx (`Board_1`), 
another running rfEasyLinkEchoRx (`Board_2`). Run Board_2 first, followed by 
Board_1. Board_1 is set to transmit a packet every second while Board_2 is 
set to receive the packet and then turnaround and transmit it after a delay of
100ms. Board_PIN_LED1 on Board_1 will toggle when it's able to successfully 
transmits a packet, and when it receives its echo. Board_PIN_LED2 on Board_2 
will toggle when it receives a packet, and then when its able to re-transmit 
it (see [figure 1]).

![perfect_echo_ref][figure 1]

If the receiver (`Board_2`) is turned off and the rfEchoTx (`Board_1`) begins 
transmitting, Board_1 switches over to the receiver mode waiting for an echo 
that will never come; in this situation a timeout timer is started and if no 
packet is received within 300ms the receiver operation is aborted. This 
condition is indicated by Board_PIN_LED1 being cleared and Board_PIN_LED2 
being set (see [figure 2]).

![missed_first_ref][figure 2]

If the receiver continues to stay turned off then the rfEchoTx example will 
alternate between transmitting and aborted receiver operations. Board_PIN_LED1
and Board_PIN_LED2 will start alternating, as seen in [figure 3].

![missed_first_few_ref][figure 3]

An error in transmission of a packet, or the reception of its echo, is 
indicated by both LEDs going high on Board_1, while an error on Board_2 will 
cause it to set Board_PIN_LED1 high and Board_PIN_LED2 low (see [figure 4]).

![echo_error_ref][figure 4].


Application Design Details
--------------------------
This example shows how to use the EasyLink API to access the RF driver, set the
frequency and transmit packets. The RFEASYLINKTX_ASYNC define is used to select
between the Blocking or Async TX/RX API.

The rfEasyLinkEchoTx example will transmit a packet every second while the 
rfEasyLinkEchoRx will echo all received packets after a delay of 100ms. If 
Board_1 aborts a packet reception Board_PIN_LED2 is set while Board_PIN_LED1 is
cleared. An error condition on Board_1 is indicated by both LEDs going high.
An error condition on Board_2 is indicated by Board_LED_1 going high while 
Board_LED_2 is cleared.

A single task, "rfEasyLinkEchoFnx", configures the RF driver through the 
EasyLink API and transmits and receives messages.

EasyLink API
-------------------------
### Overview
The EasyLink API should be used in application code. The EasyLink API is
intended to abstract the RF Driver in order to give a simple API for
customers to use as is or extend to suit their application[Use Cases]
(@ref USE_CASES).

### General Behavior
Before using the EasyLink API:

  - The EasyLink Layer is initialized by calling EasyLink_init(). This
    initializes and opens the RF driver and configures a modulation scheme
    passed to EasyLink_init.
  - The RX and TX can operate independently of each other.

The following is true for receive operation:

  - RX is enabled by calling EasyLink_receive() or EasyLink_receiveAsync()
  - Entering RX can be immediate or scheduled
  - EasyLink_receive() is blocking and EasyLink_receiveAsync() is non-blocking
  - The EasyLink API does not queue messages so calling another API function
    while in EasyLink_receiveAsync() will return EasyLink_Status_Busy_Error
  - An Async operation can be cancelled with EasyLink_abort()

The following apply for transmit operation:

  - TX is enabled by calling EasyLink_transmit(), EasyLink_transmitAsync()
    or EasyLink_transmitCcaAsync()
  - TX can be immediate or scheduled
  - EasyLink_transmit() is blocking and EasyLink_transmitAsync(), 
    EasyLink_transmitCcaAsync() are non-blocking
  - EasyLink_transmit() for a scheduled command, or if TX cannot start
  - The EasyLink API does not queue messages so calling another API function
    while in either EasyLink_transmitAsync() or EasyLink_transmitCcaAsync() 
    will return EasyLink_Status_Busy_Error
  - An Async operation can be cancelled with EasyLink_abort()

### Error Handling
The EasyLink API will return EasyLink_Status containing success or error
  code. The EasyLink_Status codes are:

   - EasyLink_Status_Success
   - EasyLink_Status_Config_Error
   - EasyLink_Status_Param_Error
   - EasyLink_Status_Mem_Error
   - EasyLink_Status_Cmd_Error
   - EasyLink_Status_Tx_Error
   - EasyLink_Status_Rx_Error
   - EasyLink_Status_Rx_Timeout
   - EasyLink_Status_Busy_Error
   - EasyLink_Status_Aborted

### Power Management
The power management framework will try to put the device into the most
power efficient mode whenever possible. Please see the technical reference
manual for further details on each power mode.

The EasyLink Layer uses the power management offered by the RF driver Refer to the RF
Driver documentation for more details.

### Supported Functions
    | Generic API function          | Description                                        |
    |-------------------------------|----------------------------------------------------|
    | EasyLink_init()               | Init's and opens the RF driver and configures the  |
    |                               | specified settings based on EasyLink_Params struct |
    | EasyLink_transmit()           | Blocking Transmit                                  |
    | EasyLink_transmitAsync()      | Non-blocking Transmit                              |
    | EasyLink_transmitCcaAsync()   | Non-blocking Transmit with Clear Channel Assessment|
    | EasyLink_receive()            | Blocking Receive                                   |
    | EasyLink_receiveAsync()       | Non-blocking Receive                                |
    | EasyLink_abort()              | Aborts a non blocking call                         |
    | EasyLink_enableRxAddrFilter() | Enables/Disables RX filtering on the Addr          |
    | EasyLink_getIeeeAddr()        | Gets the IEEE Address                              |
    | EasyLink_setFrequency()       | Sets the frequency                                 |
    | EasyLink_getFrequency()       | Gets the frequency                                 |
    | EasyLink_setRfPower()         | Sets the Tx Power                                  |
    | EasyLink_getRfPower()         | Gets the Tx Power                                  |
    | EasyLink_getRssi()            | Gets the RSSI                                      |
    | EasyLink_getAbsTime()         | Gets the absolute time in RAT ticks                |
    | EasyLink_setCtrl()            | Set RF parameters, test modes or EasyLink options  |
    | EasyLink_getCtrl()            | Get RF parameters or EasyLink options              |
    | EasyLink_getIeeeAddr()        | Gets the IEEE address                              |

### Frame Structure
The EasyLink implements a basic header for transmitting and receiving data. This header supports
addressing for a star or point-to-point network with acknowledgements.

Packet structure:

     _________________________________________________________
    |           |                   |                         |
    | 1B Length | 1-64b Dst Address |         Payload         |
    |___________|___________________|_________________________|


Note for IAR users: When using the CC1310DK, the TI XDS110v3 USB Emulator must
be selected. For the CC1310_LAUNCHXL, select TI XDS110 Emulator. In both cases,
select the cJTAG interface.

[figure 1]:EasyLinkEcho_PerfectEcho.png "Perfect Echo"
[figure 2]:EasyLinkEcho_MissedFirstPacket.png "Missed First Packet"
[figure 3]:EasyLinkEcho_MissingFirstCouplePackets.png "Missing First Couple of Packets"
[figure 4]:EasyLinkEcho_ErrorTxRx.png "Echo Error"
