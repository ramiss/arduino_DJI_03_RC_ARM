# Arduino DJI 03 RC ARM

This project allows an Arduino Nano to arm a DJI 03 Air Unit based on the output of a standard PWM RC Channel. The intent of which is to allow the full power of the DJI 03 Air Unit to activate without having to use a flight controller.

**Instructions:**
1. Complete the wiring per below.
2. [Upload the arduino_DJI_03_RC_ARM.ino sketch to the Arduino](https://support.arduino.cc/hc/en-us/articles/4733418441116-Upload-a-sketch-in-Arduino-IDE).
3. Plug the Arm Channel into whatever channel you want to arm with.
4. Power on the system.
5. When the Arm Channel goes above 1500 (above 50%) the DJI air unit will arm.

*Please note that the DJI Air Unit must be fully powered on and connected to the Goggles before you give it the arm signal. 
 

![Arduino DJI 03 RC ARM Wiring](https://i.imgur.com/l8pAnhi.jpg)
