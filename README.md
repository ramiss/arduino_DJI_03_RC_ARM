# Arduino DJI 03 RC ARM

This project allows an Arduino Nano to arm a DJI 03 Air Unit based on the output of a standard PWM RC Channel. The intent of which is to allow the full power of the DJI 03 Air Unit to activate without having to use a flight controller.

**Instructions:**
1. Complete the wiring per below.
2. [Install the following Libraries in Arduino IDE](https://support.arduino.cc/hc/en-us/articles/5145457742236-Add-libraries-to-Arduino-IDE):
   - AltSoftSerial by Paul Stroffregen V1.4
   - ReefwingMSP by David Such V2.0.0
   - PulseInput by RCMags V1.0.0 (also install any dependencies)
3. [Upload the arduino_DJI_03_RC_ARM.ino sketch to the Arduino](https://support.arduino.cc/hc/en-us/articles/4733418441116-Upload-a-sketch-in-Arduino-IDE).
4. Plug the Arm Channel into whatever channel you want to arm with on the receiver.
5. Power on the system.
6. When the Arm Channel goes above 1500 (above 50%) the DJI air unit will arm.

*Please note that the DJI Air Unit must be fully powered on and connected to the Goggles before you give it the arm signal. 

**Caution:**
Please make sure that only 5v regulated is being supplied to the Arduino Nano.  The below wiring diagram assumes the receiver is powered from a 5v BEC/UBEC.  If the receiver is powered by a different or unregulated voltage then you will need to provide a 5v UBEC between the main battery and the Arduino Nano, or provide between 7v to 12v via the Arduino's VIN pin.

![Arduino DJI 03 RC ARM Wiring](https://i.imgur.com/l8pAnhi.jpg)
