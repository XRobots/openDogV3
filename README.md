# openDogV3
CAD and Code for openDog V3: https://www.youtube.com/playlist?list=PLpwJoq86vov8uTgd8_WNgBHFpDYemO-OJ

I've included a Bill of Materials this time, BOM.ods, which is probably complete, I'll be adding to it if I remember anything else.

I used the AS5047 encoders in absolute position mode in this build. Check out the ODrive documentation for more info: https://docs.odriverobotics.com/encoders You will have to configure the encoder parameters and then run the offset calibration as per the ODrive docmentation. Default offsets are set in the code in the variable declaration section which will need calibrating to move the joints to the default positons in mode 3 below.

ODrive vel_limit and vel_limit_tolerance are set to math.inf within the ODrive tool which stops the motors disarming under certain circumstanses. (You will have to do 'import math' first within the ODrive tool).

Menu options on the Dog/LCD are as follows:

0) nothing / default at power up
1) put motors into closed loop control
2) move legs outwards so they just clear the stand stirrups by 1-2 mm
3) move legs so both shoulder and knee joints are at 45' (the default positions shown in the CAD)
4) turn up motor position, velocity and integrator gains
5) inverse kinematics demo mode for 6 axis of translation and rotation (also makes the legs slightly straighterCancel changes)
6) walking mode (same leg position as 5)
10) put the feet back into position so they rest on the stand stirrups

The remote now has a 'reverse' switch which reverses four axis of the remote so the dog walks backwards. This happens on the remote rather than in the dog's kinematics. There is also a motor enable switch which must be on for the dog to work.

Foot mould CAD is included for silicone rubber feet. I used a 25A Shore hardness Platinum cure silicone with pigment. Note that the Carbon Fibre foot tube is glued into the lower leg and foot insert to stop it rotating.

The parts are all printed in PLA. The larger parts are around 15% infill with 3 perimeters at 0.3mm layer height. The smaller parts such as the Cycloidal drive internals are 4 perimeters and up to 30-40% infill.
