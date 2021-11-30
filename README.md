# openDogV3
CAD and Code for openDog V3: https://www.youtube.com/playlist?list=PLpwJoq86vov8uTgd8_WNgBHFpDYemO-OJ

I've included a Bill of Materials this time, BOM.ods, which is probably complete, I'll be adding to it if I remember anything else.

I used the AS5047 encoders in absolute position mode in this build. Check out the ODrive documentation for more info: https://docs.odriverobotics.com/encoders

Also the ODrive vel_limit and vel_limit_tolerance are set to math.infinite within the ODrive tool which stops the axis disarming under certain circumstanses. (You will have to do 'import math' first within the ODrive tool).

Menu options on the Dog/LCD are as follows:

0) nothing / default at power up
1) put motors into closed loop control
2) move legs outwards so they just clear the stand stirrups by 1-2 mm
3) move legs so both shouler and knee joints are at 45'
4) turn up motor position, velocity and integrator gains
5) inverse kinematics demo mode for 6 axis of translation and rotation
6) walking mode
10) put the feet back into position so they rest on the stand stirrups
