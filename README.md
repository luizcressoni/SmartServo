# SmartServo
Smart Servo class that implements a simple S-curve (constant acceleration) for servo control

cSmartServo class implements s-curve soft movement control for servos and runs on Arduino IDE and PlatformIO.
<br>Servo control is still done by a Servo (#include <Servo.h>) object.
<br>
From initial position (i) to final position (f) motion starts with initial speed defined by VMIN and starts accelerating until
<br>(a) - maximun speed (VMAX) is reached or
<br>(b) - middle point is reached.
<br>If VMAX is reached before the middle point, the distance from the middle point is captured and servo keeps moving with constant speed until it reaches point (c) when it starts deaccelerating down to VMIN and keeps moving until the final point is reached.
<br>If the middle point is reached before VMAX, then it will start deaccelerating immediately. 

<p><img src="https://github.com/luizcressoni/SmartServo/blob/main/pictures/scurve.png?raw=true" alt="" width="556" height="351" /></p>
<br>
Servo position limits are defined my the Servo object and are: minimum 0, maximum 180. You can trim those limits and the code will interpolate input values in the full range (0-180) to your trimmed values.
<br><br><b>How to use:</b>
<br><i>init(uint8_t _index, uint8_t _iopin, uint8_t _min, uint8_t _max, uint8_t _initpos)</i>
<br> _index: Servo Index. Used by the servo objeto to define the PWM channel (0 to 15)
<br> _iopin: GPIO used to control the servo. Must be a PWM capable pin
<br> _min / _max: minimum and maximum position values
<br> _initpos: initial position
<br>
<br><i>Scurve(uint8_t _endpos, uint8_t _vmax)</i>
<br>Call this to set the end position
<br>_endpos: end position
<br>_vmax: maximum speed in degrees / SERVO_TASKDELAY ms
<br>If the distance to the end point is less than MIN_DISTANCE, S-curve is disabled and the servo will run in constant speed.
<br><br>
After this, just call loop() as frequent as you can 
