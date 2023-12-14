from ubuntu:latest
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -yq software-properties-common 
RUN apt-get update && apt-get install -yq \
    gnuradio-dev gr-osmosdr hackrf gqrx-sdr \
    git vim python3-pip minicom  \
    xauth sudo wget unzip libiio0 libiio-utils \
    udiskie gcc-arm-none-eabi bluez
     
RUN pip3 install setuptools gatt==0.2.7 pyzmq==24.0.0 

ENV SC=/screaming_channels
ENV PYTHONPATH=/usr/lib/python3/dist-packages:/usr/lib/python3/site-packages
ENV LD_LIBRARY_PATH=/usr/lib

RUN git clone https://www.github.com/muchlikeacat/screaming_channels
WORKDIR /screaming_channels


RUN cd experiments/src/ && python3 setup.py develop

WORKDIR /screaming_channels/experiments/

RUN mkdir traces

ENV NORDIC_SEMI_SDK=/home/screaming/screaming_channels/firmware/nRF5_SDK_14.2.0_17b948a/
