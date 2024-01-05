# Arduino DJI 03 RC ARM V2

This project allows an Arduino XIAO Seeeduino to arm a DJI 03 Air Unit automatically once powered up, and as long as the Goggles are powered up. The intent of which is to allow the full power of the DJI 03 Air Unit to activate without having to use a flight controller.

Note that this project no longer uses an RC channel to arm, although that feature may come at a later time.

**Instructions:**
1. Complete the wiring per below.
2. Download this entire repository (click on green Code button and  then Download Zip).
3. Extract all files into a folder called "arduino_DJI_03_RC_ARM" exactly that name, no "-main" or anything else.
4. Open the arduino_DJI_03_RC_ARM.ino file in Arduino IDE.
5. Install the Seeeduino Board and choose/downgrade to board version 1.7.9. They changed something with SerialUSB that breaks the code after that, I will fix in a future release.
6. [Upload the arduino_DJI_03_RC_ARM.ino sketch to the Arduino](https://support.arduino.cc/hc/en-us/articles/4733418441116-Upload-a-sketch-in-Arduino-IDE).
7. Power on your goggles.
8. Power on the aircraft.
9. Two seconds after the Air Unit has powered up, it will arm. 

**Caution:**
Please make sure that no more than 5v regulated is being supplied to the Arduino XIAO Seeeduino.  The below wiring diagram assumes the receiver is powered from a 5v BEC/UBEC.  If the receiver is powered by a different or unregulated voltage then you will need to provide a 5v UBEC between the main battery and the Arduino and remove the connection to the receiver.

**Voltage reading (optional):**  
Add a voltage divider between supply and ground, highest resistance value towards positive (R1), middle going to A0.  
Choose resistors in the tens of kiloohms range, with a ratio appropriate for the intended supply voltage.  
e.g. R1=33k, R2=5.1k, ratio of 6.471, measurement range 21.3V (3.3V Arduino reference * 6.471)  
Set VOLT_DIVIDER to 1024/measurement range e.g. 1024/21.3 = 48, then adjust as necessary if not precise enough.  

![Arduino DJI 03 RC ARM Wiring](https://i.imgur.com/88jpdS7.png)
