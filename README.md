# Fan Tach Modifier

Allows an arduino to change the reported speed of a fan by reading the signal from the tachometer wire and creating a new modified output signal.
Currently set up to change a 300-1200 RPM fan (a Noctua NF S12A fan) into a 1600-6400 RPM (simulating a high speed server fan).

Designed for a Digispark Pro or Arduino Uno, but should be able to be modified to work on any arduino with at least one 16-bit Timer (in addition to the inturrupt timer). Support for multiple fans can be added with additional 16-bit Timers.
