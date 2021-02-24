# List of Arm Commands

## e: Emergency stop

### Synopsis
e

### Description
This command stops the arm from moving. No command will resume arm movement until a ‘c’ command is issued.

***

## c: Resume from emergency stop

### Synopsis
c

### Description
This command allows the arm to resume moving after an emergency stop. 

***

## p: Move to absolute position within limits

### Synopsis
p [angle 0] [angle 1] [angle 2] [angle 3] [angle 4] [angle 5] [angle 6] [angle 7]

### Description
This command is the most useful command. It tells the arm to move to a set of joint coordinates. These coordinates are directly from the IK model.

### Example
p 50 -200 0 100000 582 20 5401
This command would move the arm to the specified position. Angle 4 is out of range, so it automatically gets limited to the maximum specified in the Arduino code.

***

## f: Move to position with NO LIMITS

### Synopsis
f [angle 0] [angle 1] [angle 2] [angle 3] [angle 4] [angle 5] [angle 6] [angle 7]

### Description
This command is just like the ‘p’ command, but ignores angle constraints
Example
p 50 -200 0 100000 582 20 5401
This command would move the arm to the specified position. Angle 4 is out of normal range, but the ‘f’ command does not care and destroys the arm.

***

## r: Relative move

### Synopsis
r [angle 0] [angle 1] [angle 2] [angle 3] [angle 4] [angle 5] [angle 6] [angle 7]

### Description
This command adds the argument angles to the current goal positions. It essentially moves the arm by a relative amount. It does not have limits, so care must be taken.

### Example
r 50 0 0 0 0 0 0 0
This command would rotate the shoulder by ~5 degrees and would not affect anything else. 

***

## r: Relative move

### Synopsis
r [angle 0] [angle 1] [angle 2] [angle 3] [angle 4] [angle 5] [angle 6] [angle 7]

### Description
This command adds the argument angles to the current goal positions. It essentially moves the arm by a relative amount. It does not have limits, so care must be taken.

## Example
r 50 0 0 0 0 0 0 0
This command would rotate the shoulder by ~5 degrees and would not affect anything else. 

***

## s: Starting position calibration

### Synopsis
s

### Description
Once the arm is in the starting position, running this command will calibrate the encoders to know they are in that position. The starting position is everything centered, and the elbow and shoulder as far up as possible.

***

## m: Direct manual motor control

### Synopsis
m [velocity 0] [velocity 1] [velocity 2] [velocity 3] [velocity 4] [velocity 5] [velocity 6] [velocity 7]

### Description
This command allows for direct control of joint velocities. An ‘m’ command must be sent at least 10 times per second, or all the motors will pause until the next command to avoid damage. When an ‘m’ command is sent, the arm enters “manual mode”. When in “manual mode”, PID control is disabled. To get out of “manual mode”, send a ‘p’, ‘f’ or ‘r’ command. 
Example
m 0 0 0 0 -200 0 0
This command would move the wrist downwards at about 78% power, and all the other joints would be stationary. If another ‘m’ command is not received within 100ms, all the motors would shut off until the next ‘m’ command.
