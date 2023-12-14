#!/usr/bin/env python3
  
import serial, sys, time
port = "/dev/ttyACM0"
baudrate = 115200
ser = serial.Serial(port,baudrate,timeout=1)



def sendChars(stringToSend):
    ser.write(str.encode(stringToSend))
    time.sleep(0.1)

def readAll(waitFor):
    while True:
        data = ""
        try:
            data = ser.readline().decode('ascii').replace('\r\n', '')
            print(data)
            time.sleep(0.01)
        except:
            exit
        if(waitFor==""):
            if(data==""):
                break
        else:
            if(waitFor in data):
                break

def GiveChars(string, waitFor=""):
    sendChars(string)
    readAll(waitFor)

def GiveLine(string, waitFor=""):
    GiveChars(string+"\r\n", waitFor)

GiveChars("h") #Display menu (just lets us know we're connected correctly)

GiveChars("p") #Set a new power Level
GiveChars("0") #Set power to +4 dBm (max for cc1310)

GiveChars("c") #Start radio transmit in continuous mode


GiveChars("n") #Enter AES mode
GiveChars("n") #[From AES mode] Set number of encryptions
GiveLine("2000") #Set number of encryptions to be 5000
x=0
while x<1000:
    print("Sleeping 2 sec")
    time.sleep (2)
    print("Beginning Encryption again")
    start = time.time()
    GiveChars("r", "Done")
    end = time.time()
    print("Encrypted for " + str(end - start) + "s")
    x=x+1
    
GiveChars("q") #Exit AES mode
GiveChars("e") #Disable continuous broadcast

GiveChars("h") #Checking that we're still responding

print("Done")















