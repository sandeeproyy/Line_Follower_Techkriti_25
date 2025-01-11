# Line_Follower_Bot_techkriti_25
Let me explain how the algorithm works:

Initialization Phase:


Initialize all sensors (IR array, obstacle sensor)
Setup OLED display
Configure motor driver pins
Set initial state to LINE_FOLLOWING


Main Line Following:


Continuously read the 7-channel IR sensor array
Calculate position error from center line
Apply PID control to maintain smooth line following
Adjust motor speeds based on PID output


Node Detection & Processing:


When multiple center sensors detect black line, identify as potential node
Read the 2×2 matrix pattern using IR sensors
Convert pattern to 4-bit binary number (ABCD format)
Display value on OLED screen
Process based on node number:
First Node:

Convert binary to decimal (x)
Calculate speed limit = x/5 m/s
Adjust motor speeds accordingly

Second Node (0000):

Reset speed to normal
Continue line following

Third Node:

Convert binary to decimal (x)
Calculate angle = x*10 degrees
Enter circle following mode




Obstacle Avoidance:


Continuously check front IR sensor
If obstacle detected:

Stop immediately
Turn 180 degrees
Resume line following




Circle Following:


After third node, enter circle following state
Adjust motor speeds for circular motion
Use encoders to measure arc length
Turn at calculated angle (clockwise or counterclockwise)
Return to line following once angle reached


Speed Control:


Maintain speed below calculated limit from first node
Use encoder feedback for precise speed control
PID control for smooth acceleration/deceleration


Final Phase:


Upon reaching finish area
Display "FINISH" on OLED
Stop motors

Key Features:

Real-time node value display
Precise speed control using encoders
Obstacle detection and avoidance
Accurate angle calculation and turning
PID-based line following for smooth movement

The algorithm uses multiple states to handle different situations, with the main states being:

LINE_FOLLOWING
NODE_READING
SPEED_CONTROL
ANGLE_DETECTION
CIRCLE_FOLLOWING
FINISHED

Each state has its own control logic while maintaining the ability to transition between states based on sensor inputs and progression through the course.
