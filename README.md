# Arduino DJI 03 RC ARM V2

This project allows an Arduino XIAO Seeeduino to arm a DJI 03 Air Unit automatically once powered up, and as long as the Goggles are powered up. The intent of which is to allow the full power of the DJI 03 Air Unit to activate without having to use a flight controller.

Note that this project no longer uses an RC channel to arm, although that feature may come at a later time.

**Instructions:**
1. Complete the wiring per below.
2. Download this entire repository, including the MSP library files, which have been modified for this application.
3. [Upload the arduino_DJI_03_RC_ARM.ino sketch to the Arduino](https://support.arduino.cc/hc/en-us/articles/4733418441116-Upload-a-sketch-in-Arduino-IDE).
4. Power on your goggles.
5. Power on the aircraft.
6. Two seconds after the Air Unit has powered up, it will arm. 

**Caution:**
Please make sure that no more than 5v regulated is being supplied to the Arduino XIAO Seeeduino.  The below wiring diagram assumes the receiver is powered from a 5v BEC/UBEC.  If the receiver is powered by a different or unregulated voltage then you will need to provide a 5v UBEC between the main battery and the Arduino and remove the connection to the receiver.

**Voltage reading:**  
Add a voltage divider between supply and ground, highest resistance value towards positive (R1), middle going to A0.  
Choose resistors in the tens of kiloohms range, with a ratio appropriate for the intended supply voltage.  
e.g. R1=33k, R2=5.1k, ratio of 6.471, measurement range 21.3V (3.3V Arduino reference * 6.471)  
Set VOLT_DIVIDER to 1024/measurement range e.g. 1024/21.3 = 48, then adjust as necessary if not precise enough.  

![Arduino DJI 03 RC ARM Wiring](https://i.imgur.com/88jpdS7.png)
