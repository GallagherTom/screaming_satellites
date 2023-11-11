# Screaming Satellites 
_An expansion of the research done in [Screaming Channels](https://eurecom-s3.github.io/screaming_channels/) with a focus on small satellites_

This repository provides source code and artifacts to support the three article series exploring the potential data leaks across broadcasts of System-on-a-Chip transceivers used by (or potentially used by) small satellites. The articles in this series build on each other to ultimately provide a framework and process for identifying and measuring _Screaming Channels_ data leaks. The articles in this series are:
1. (Article #1) [_Understanding How System-on-a-Chip Data can Leak over Radio Transmissions_](https://www.ijatl.org/)
2. (Article #2) [_Identifying System-on-a-Chip Data Leaks over Radio Transmissions of Small Satellites_](https://www.ijatl.org/)
3. (Article #3) [_Measuring System-on-a-Chip Data Leaks over Radio Transmissions of Small Satellites_](https://www.ijatl.org/)

## Reproducing Experiments from Article #2
Article #2 provided a process for identifying potential leaks in SoC transceivers. The process was demonstrated using four real-world devices. The source code and binaries used in those demonstrations is available here and the steps below outline the process. 

1. Load firmware on the device
   1. **Nordic Semiconductor nRF52832** [source](https://github.com/eurecom-s3/screaming_channels) | [script](https://github.com/GallagherTom/screaming_satellites/blob/main/scripts/alternate-sleep-active/nRF52832.py)
      + The nRF52832 firmware was developed for the 2018 Screaming Channels research. An additional script is used to exercise the firmware according to the process described in Article #2
   3. **Texas Instruments CC1111** [source](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware/cc1111) | [binary](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware/cc1111/binary)
   4. **Texas Instruments CC1310** [source](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware/cc1310) | [binary](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware/cc1310/binary) | [script](https://github.com/GallagherTom/screaming_satellites/blob/main/scripts/alternate-sleep-active/cc1310.py)
      + The CC1310 firmware mirrors the firmware used in the 2018 Screaming Channels research and thus requires an additional script to exercise it in accordance with the process described in Article #2
   6. **Seeed LoRa Wio-E5** [source](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware) | [binary](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware)
   7. **Other devices** the above four devices were included in the research of this series. If you are evaluating a different device, you will need to follow the process outlined in Article #2 
2. Listen for Transmissions
   1. Setup [Gnuradio and GnuRadio Companion (GRC)](https://www.gnuradio.org/)
   2. Download the [.GRC script](https://github.com/GallagherTom/screaming_satellites/tree/main/frequency-time-analysis) developed for this experiment
      + If needed, alter the file to meet your requirements. For instance, the script includes acceptable ranges for frequency and signal gain. These may need to be adjusted to meet the requirements for your specific requirements
3. Exercise the Firmware
   + The CC1111 and Wio-E5 firmware automatically begins iterating through patterns of sleeping and processing
   + The nRF52832 and CC1310 firmware provides more robust functionality and is exercised according to the process described in Article #2 using an external Python script
4. Adjust Parameters and Observe the results
   + The Receiver gain, Intermediate gain, Baseband gain, and receiving frequency can all be adjusted through slider controls in the user interface created by the GRC script. Adjusting these values can make a signal easier or harder to identify within the noise. Finding the right balance for these parameters will depend on the hardware and environment in question. Iterating through combinations of values is a time-consuming but unavoidable step.
   + Compare the GRC output to the results provided in Article #2

## Reproducing Experiments from Article #3
Coming soon!
  
