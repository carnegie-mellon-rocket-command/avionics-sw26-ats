# avionics-26-ats

## datalogger2026
The servo position is absolute (0 degrees is always the same every time you turn it on). 

The Servo library write() function takes in [0,180]. Our servo goes 0 to 270. You must map your actual max constraint (for 2026 subscale, 254 degrees) to some value within 180.

e.g. If our max is 254, ATS_MAX is 254/270*180.

Servo attach() also will write a 93 degree turn when it is called, so just write(0) immediately after.
