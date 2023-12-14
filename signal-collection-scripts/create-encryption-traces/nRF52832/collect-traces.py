import os, time 


config_path = "/nRF52832-configs/"
config_name = "fixed-key.json" #change to the name of the configuration for the test



timestr = time.strftime("%Y%m%d-%H%M%S")

os.system("mkdir test-"+timestr)


os.system('xhost si:localuser:root')
os.system('docker build -t sc .')

#collect traces
initial_command = "sc-triage --radio=HackRF --loglevel=DEBUG --device=/dev/ttyACM0 triage /configs/"+config_name+" /output"
os.system('docker run -it --net=host --env="DISPLAY" -v /tmp/.X11-unix:/tmp/.X11-unix -v "$HOME/.Xauthority:/root/.Xauthority:rw" --privileged -v /dev/bus/usb:/dev/bus/usb -v /media/owner/JLINK:/media/root/JLINK:rw -v "$(pwd)'+config_path+':/configs" -v "$(pwd)/test-'+timestr+'/output:/output" sc -c "' + initial_command+ '"')

#Fix permissions on files created by root
os.system('chmod 0777 -R ./test-'+timestr)


print("Capture Complete")
