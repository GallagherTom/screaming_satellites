from ubuntu:latest
ENV DEBIAN_FRONTEND noninteractive

RUN apt-get update && apt-get install -yq software-properties-common 
RUN apt-get update && apt-get install -yq \
    gnuradio-dev gr-osmosdr hackrf gqrx-sdr \
    git vim python3-pip minicom  \
    xauth sudo wget unzip libiio0 libiio-utils \
    udiskie gcc-arm-none-eabi bluez \
    libntl-dev libgmp-dev
RUN pip3 install setuptools gatt==0.2.7 pyzmq==24.0.0 


RUN git clone https://github.com/giocamurati/python_hel
WORKDIR /python_hel/hel_wrapper
RUN make AES_TYPE=aes_ni && \
    make install && ldconfig  
WORKDIR /python_hel/python_hel
RUN python3 setup.py install

ENV SC=/screaming_channels
ENV PYTHONPATH=/usr/lib/python3/dist-packages:/usr/lib/python3/site-packages
ENV LD_LIBRARY_PATH=/usr/lib
ENV NORDIC_SEMI_SDK=/home/screaming/screaming_channels/firmware/nRF5_SDK_14.2.0_17b948a/

WORKDIR /

RUN git clone https://github.com/GallagherTom/screaming_satellites/

WORKDIR /screaming_satellites/screaming_channels/screaming_channels_v2/
RUN cd experiments/src/ && python3 setup.py develop

WORKDIR /screaming_satellites/screaming_channels/screaming_channels_v2/experiments/

ENTRYPOINT ["/bin/sh"]
    
