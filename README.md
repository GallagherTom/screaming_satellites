# Screaming Satellites 
_An expansion of the research done in [Screaming Channels](https://eurecom-s3.github.io/screaming_channels/) with a focus on small satellites_

This repository provides source code and artifacts to support the three article series exploring the potential data leaks across broadcasts of System-on-a-Chip transceivers used by (or potentially used by) small satellites. The articles in this series build on each other to ultimately provide a framework and process for identifying and measuring _Screaming Channels_ data leaks. The articles in this series are:
1. [_Understanding How System-on-a-Chip Data can Leak over Radio Transmissions_ (Article #1)](https://www.ijatl.org/)
2. [_Identifying System-on-a-Chip Data Leaks over Radio Transmissions of Small Satellites_ (Article #2)](https://www.ijatl.org/)
3. [_Measuring System-on-a-Chip Data Leaks over Radio Transmissions of Small Satellites_ (Article #3)](https://www.ijatl.org/)

## Reproducing Experiments from Article #2
Article #2 provided a process for identifying potential leaks in SoC transceivers. The process was demonstrated using four real-world devices. The source code and binaries used in those demonstrations is available here and the steps below outline the process. 

1. Load firmware on the device
   1. Nordic Semiconductor nRF52832: [source](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware) | [binary](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware)
   3. Texas Instruments CC1111 [source](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware) | [binary](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware)
   4. Texas Instruments CC1310 [source](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware) | [binary](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware)
   5. Seeed LoRa Wio-E5 [source](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware) | [binary](https://github.com/GallagherTom/screaming_satellites/tree/main/firmware)
   6. Another device: the above four devices were included in the research of this series. If you are evaluating a different device, you will need to follow the process outlined in Article #2 
2. Receive transmissions using GnuRadio
   1. Setup [Gnuradio and GnuRadio Companion](https://www.gnuradio.org/)
   2. Download the [.GRC script](https://github.com/GallagherTom/screaming_satellites/blob/main/spectrum_analysis/spectrum_analzer.grc) developed for this experiment
      + If needed, alter the file to meet your requirements. For instance, the script includes acceptable ranges for frequency and signal gain. These may need to be adjusted to meet the requirements for your specific requirements

## Reproducing Experiments from Article #3
Coming soon!
  
