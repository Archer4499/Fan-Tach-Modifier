# Fan Tach Modifier

Allows an Arduino to change the reported speed of a fan by reading the signal from the tachometer wire and creating a new modified output signal.
Optionally supports using a potentiometer to set the modification factor between fan speed and output speed.

The following boards have a preconfigured configuration:
  - Digispark Pro    (One fan)
  - Arduino Uno/Nano (One fan)
  - Teensy 2.0       (Two fans)
  - Arduino Mega     (Four fans)

And easy custom configuration to allow any other board with one 16-bit timer and free hardware interrupt pin per fan configured to be used.

## Some Specifics

Uses Timer1/3/4/5 for each fan respectively as these are usually the 16-bit timers if the board has them.

#### For each fan x (0-3), the following pins need to be configured:
  - IN_PIN_x
    - A hardware interrupt pin. Connected to the fan to read its actual RPM.
  - OUT_PIN_x
    - Any free non-restricted pin. Connected to the end device to output the modified signal to.
  - POT_PIN_x (Optional)
    - An analog input pin. To use a potentiometer to set the modification factor between fan speed and output speed.
    - Can also define the same pin to control multiple fans from the same potentiometer.


#### Example fan math: 
- In (Noctua NF S12A):
  - Fan speed 300-1200 RPM
  - 600-2400 half_revolutions/min
  - 10-40 Hz
  - Input interval received: **100-25 ms**
- Out (Emulating: Nidec V12E12BS1B570):
  - Fan speed 1600?-6400 RPM
  - 3200-12800 half_revolutions/min
  - 53.33-213.33 Hz
  - 18.75-4.69 ms
  - Output interval resolution count sent: **4687-1172 counts**
- Factor of 5.333, in this case calculated from the max fan speeds.
