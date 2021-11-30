      void kinematics (int leg, float xIn, float yIn, float zIn, float roll, float pitch, float yawIn, int interOn, int dur) {
      
      // leg 1  : front left
      // leg 2  : front right
      // leg 3  : back left
      // leg 4  : back right
      
      // moving the foot sideways on the end plane
      float hipOffset = 108;     // distance from the hip pivot to the centre of the leg
      float lengthY;
      float hipAngle1a;
      float hipAngle1b;
      float hipAngle1;
      float hipAngle1Degrees;
      float hipHyp;
      
      // moving the foot forwards or backwardes in the side plane
      float shoulderAngle2;
      float shoulderAngle2Degrees;
      float z2;
      
      // side plane of individual leg only
      #define shinLength 200     
      #define thighLength 200
      float z3;
      float shoulderAngle1;
      float shoulderAngle1Degrees;
      float shoulderAngle1a;   
      float shoulderAngle1b;
      float shoulderAngle1c;
      float shoulderAngle1d;
      float kneeAngle;  
      float kneeAngleDegrees; 
      
      // *** ROTATION AXIS
      
      // roll axis
      #define bodyWidth 59         // half the distance between the hip  pivots (the front)
      float legDiffRoll;            // differnece in height for each leg
      float bodyDiffRoll;           // how much shorter the 'virtual body' gets
      float footDisplacementRoll;   // where the foot actually is
      float footDisplacementAngleRoll; // smaller angle
      float footWholeAngleRoll;     // whole leg angle
      float hipRollAngle;       // angle for hip when roll axis is in use
      float rollAngle;          // angle in RADIANS that the body rolls
      float zz1a;               // hypotenuse of final triangle
      float zz1;                // new height for leg to pass onto the next bit of code
      float yy1;                // new position for leg to move sideways
      
      // pitch axis
      #define bodyLength 272          // half the distance between shoulder pivots  (the side)
      float legDiffPitch;            // differnece in height for each leg
      float bodyDiffPitch;           // how much shorter the 'virtual body' gets
      float footDisplacementPitch;   // where the foot actually is
      float footDisplacementAnglePitch; // smaller angle
      float footWholeAnglePitch;     // whole leg angle
      float shoulderPitchAngle;      // angle for hip when roll axis is in use
      float pitchAngle;              // angle in RADIANS that the body rolls
      float zz2a;                    // hypotenuse of final triangle
      float zz2;                     // new height for the leg to pass onto the next bit of code
      float xx1;                     // new position to move the leg fowwards/backwards
      
      // yaw axis 
      
      float yawAngle;                 // angle in RADIANs for rotation in yaw
      float existingAngle;            // existing angle of leg from centre
      float radius;                   // radius of leg from centre of robot based on x and y from sticks
      float demandYaw;                // demand yaw postion - existing yaw plus the stick yaw 
      float xx3;                      // new X coordinate based on demand angle 
      float yy3;                      // new Y coordinate based on demand angle
      
      float x;
      float y;
      float z;
      float yaw;
      
      
      // ** INTERPOLATION **
      // use Interpolated values if Interpolation is on
      if (interOn == 1) {     
      
            if (leg == 1) {        // front right
                z = interpFRZ.go(zIn,dur);
                x = interpFRX.go(xIn,dur);
                y = interpFRY.go(yIn,dur);
                yaw = interpFRT.go(yawIn,dur);
            }
          
            else if (leg == 2) {    // front left
                z = interpFLZ.go(zIn,dur);
                x = interpFLX.go(xIn,dur);
                y = interpFLY.go(yIn,dur); 
                yaw = interpFLT.go(yawIn,dur);
             
            }
          
            else if (leg == 4) {   // back right
                z = interpBRZ.go(zIn,dur);
                x = interpBRX.go(xIn,dur);
                y = interpBRY.go(yIn,dur); 
                yaw = interpBRT.go(yawIn,dur); 
            }
          
            else if (leg == 3) {    // back left
                z = interpBLZ.go(zIn,dur);
                x = interpBLX.go(xIn,dur);
                y = interpBLY.go(yIn,dur); 
                yaw = interpBLT.go(yawIn,dur);
            }  
      } 
      
      
      // wait for filters to settle before using Interpolated values
      // set a timer for filter to settle    
      if (interpFlag == 0) {           
                z = zIn;        // in the meantime use raw values
                x = xIn;
                y = yIn;
                yaw = yawIn;
            if (currentMillis - previousInterpMillis >= 300) {
                interpFlag = 1;
            } 
       }
       // once time has settled and Interpolation is off then use the original values
       else if (interpFlag == 1 && interOn == 0 ) {
                z = zIn;
                x = xIn;
                y = yIn;
                yaw = yawIn;
       }
      
       // **** START INVERSE KINEMATICS CALCS ****
      
       //yy3 = y;
       //zz2 = z;
       //xx3 = x;

           // ** YAW AXIS **

      // convert degrees to radians for the calcs
      yawAngle = (PI/180) * yaw;
      
      // put in offsets from robot's parameters so we can work out the radius of the foot from the robot's centre
      if (leg == 1) {         // front left leg
         y = y - (bodyWidth+hipOffset); 
         x = x - bodyLength;      
      }
      else if (leg == 2) {    // front right leg
         y = y + (bodyWidth+hipOffset);
         x = x - bodyLength; 
      }
      else if (leg == 3) {    // back left leg
         y = y - (bodyWidth+hipOffset); 
         x = x + bodyLength;
      }
      else if (leg == 4) {    // back left leg
         y = y + (bodyWidth+hipOffset); 
         x = x + bodyLength;
      }
      
      //calc existing angle of leg from cetre
      existingAngle = atan(y/x);   
      
      // calc radius from centre
      radius = y/sin(existingAngle);
      
      //calc demand yaw angle
      demandYaw = existingAngle + yawAngle;
      
      // calc new X and Y based on demand yaw angle
      xx3 = radius * cos(demandYaw);           // calc new X and Y based on new yaw angle
      yy3 = radius * sin(demandYaw);
      
      // remove the offsets so we pivot around 0/0 x/y
      if (leg == 1) {         // front left leg
         yy3 = yy3 + (bodyWidth+hipOffset); 
         xx3 = xx3 + bodyLength;      
      }
      else if (leg == 2) {    // front right leg
         yy3 = yy3 - (bodyWidth+hipOffset);
         xx3 = xx3 + bodyLength; 
      }
      else if (leg == 3) {    // back left leg
         yy3 = yy3 + (bodyWidth+hipOffset); 
         xx3 = xx3 - bodyLength;
      }
      else if (leg == 4) {    // back left leg
         yy3 = yy3 - (bodyWidth+hipOffset); 
         xx3 = xx3 - bodyLength;
      }       

      // ** PITCH AXIS ***   
      
      if (leg == 1 || leg == 2) {
      pitch = pitch *-1;
      xx3 = xx3*-1;
      }
      
      // convert pitch to degrees
      pitchAngle = (PI/180) * pitch;
      
      //calc top triangle sides
      legDiffPitch = sin(pitchAngle) * bodyLength;
      bodyDiffPitch = cos(pitchAngle) * bodyLength;
      
      // calc actual height from the ground for each side
      legDiffPitch = z - legDiffPitch;
      
      // calc foot displacement
      footDisplacementPitch = ((bodyDiffPitch - bodyLength)*-1)+xx3;
      
      //calc smaller displacement angle
      footDisplacementAnglePitch = atan(footDisplacementPitch/legDiffPitch);
      
      //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
      zz2a = legDiffPitch/cos(footDisplacementAnglePitch);
      
      // calc the whole angle for the leg
      footWholeAnglePitch = footDisplacementAnglePitch + pitchAngle;
      
      //calc actual leg length - the new Z to pass on
      zz2 = cos(footWholeAnglePitch) * zz2a;
      
      //calc new Z to pass on
      xx1 = sin(footWholeAnglePitch) * zz2a;  
      
      if (leg == 1 || leg == 2) {
      xx1 = xx1*-1;
      }         

      // *** ROLL AXIS *** 
      
      //turn around roll angle for each side of the robot
      if (leg == 2 || leg == 3) {
        roll = 0-roll;
        yy3 = yy3*-1;
       
      }
      else if (leg == 1 || leg == 4) {
        roll = 0+roll;       
      }
      
      // convert roll angle to radians
      rollAngle = (PI/180) * roll;    //covert degrees from the stick to radians
      
      // calc the top triangle sides
      legDiffRoll = sin(rollAngle) * bodyWidth;
      bodyDiffRoll = cos(rollAngle) * bodyWidth;  
      
      // calc actual height from the ground for each side
      legDiffRoll = zz2 - legDiffRoll; 
      
      // calc foot displacement
      footDisplacementRoll = (((bodyDiffRoll - bodyWidth)*-1)+hipOffset)-yy3;
      
      //calc smaller displacement angle
      footDisplacementAngleRoll = atan(footDisplacementRoll/legDiffRoll); 
      
      //calc distance from the ground at the displacement angle (the hypotenuse of the final triangle)
      zz1a = legDiffRoll/cos(footDisplacementAngleRoll);
      
      // calc the whole angle for the leg
      footWholeAngleRoll = footDisplacementAngleRoll + rollAngle;
      
      //calc actual leg length - the new Z to pass on
      zz1 = cos(footWholeAngleRoll) * zz1a;
      
      //calc new Y to pass on
      yy1 = (sin(footWholeAngleRoll) * zz1a)-hipOffset;       // take away the offset so we can pivot around zero  
  
  
      // *** TRANSLATION AXIS ***
  
      // calculate the hip joint and new leg length based on how far the robot moves sideways
      // Y axis - side to side
      // first triangle
  
      if (leg == 1 || leg == 4) {   // reverse the calcs for each side of the robot
        hipOffset = hipOffset * -1; 
        yy1 = yy1*-1;     
      }
  
      yy1 = yy1 +  hipOffset;          // add on hip offset because there is default distance in Y
      hipAngle1a = atan(yy1/zz1);    
      hipAngle1Degrees = ((hipAngle1a * (180/PI)));   // convert to degrees
      hipHyp = zz1/cos(hipAngle1a);         // this is the hypotenuse of the first triangle
  
      // second triangle
      
      hipAngle1b = asin(hipOffset/hipHyp) ;     // calc 'the other angle' in the triangle
      hipAngle1 = (PI - (PI/2) - hipAngle1b) + hipAngle1a;     // calc total hip angle
      hipAngle1 = hipAngle1 - 1.5708;           // take away offset for rest position
      hipAngle1Degrees = ((hipAngle1 * (180/PI)));   // convert to degrees 
  
      // calc new leg length to give to the code  below
      z2 = hipOffset/tan(hipAngle1b);           // new leg length
  
      // ****************
      
      // X axis - front to back
      // calculate the shoulder joint offset and new leg length based on now far the foot moves forward/backwards
      shoulderAngle2 = atan(xx1/z2);     // calc how much extra to add to the shoulder joint
      shoulderAngle2Degrees = shoulderAngle2 * (180/PI);
      z3 = z2/cos(shoulderAngle2);     // calc new leg length to feed to the next bit of code below
  
      // ****************
  
      // Z axis - up and down
      // calculate leg length based on shin/thigh length and knee and shoulder angle
      z3 = constrain(z3,200,390);                 // constrain leg length to stop it turning inside out and breaking the trig
      shoulderAngle1a = sq(thighLength) + sq(z3) - sq(shinLength);
      shoulderAngle1b = 2 * thighLength * z3;
      shoulderAngle1c = shoulderAngle1a / shoulderAngle1b;
      shoulderAngle1 = acos(shoulderAngle1c);     // radians
      kneeAngle = PI - (shoulderAngle1 *2);       // radians
  
      //calc degrees from angles
      shoulderAngle1Degrees = shoulderAngle1 * (180/PI);    // degrees
      kneeAngleDegrees = kneeAngle * (180/PI);              // degrees 
  


      // write to joints
      
      float conversion;
      conversion = 0.02777777777777777777777777777778;    // factor for converting degrees to motor turns used by the ODrive
      
      if (leg == 1) {     // front right
          float shoulderAngle1Counts = (shoulderAngle1Degrees-45) * conversion; // convert to encoder counts
          float shoulderAngle2Counts = shoulderAngle2Degrees * conversion; // convert to encoder counts
          float shoulderAngleCounts = shoulderAngle1Counts + shoulderAngle2Counts;
          float kneeAngleCounts = (kneeAngleDegrees-90) * conversion; // convert to encoder counts 
          float hipAngleCounts = hipAngle1Degrees * conversion;    // convert to encoder counts       
          driveJoints (21, shoulderAngleCounts);    // front right shoulder
          driveJoints (20, kneeAngleCounts);    // front right knee  
          driveJoints (10, hipAngleCounts);     // front right hip
      }
      
      else if (leg == 2) {     // front left
          float shoulderAngle1Counts = (shoulderAngle1Degrees-45) * conversion; // convert to encoder counts
          float shoulderAngle2Counts = shoulderAngle2Degrees * conversion; // convert to encoder counts
          float shoulderAngleCounts = shoulderAngle1Counts + shoulderAngle2Counts;
          float kneeAngleCounts = (kneeAngleDegrees-90) * conversion; // convert to encoder counts 
          float hipAngleCounts = hipAngle1Degrees * conversion;    // convert to encoder counts       
          driveJoints (51, shoulderAngleCounts);    // front left shoulder
          driveJoints (50, kneeAngleCounts);    // front left knee
          driveJoints (40, hipAngleCounts);     // front left hip
      }
      
      else if (leg == 3) {     // back left
          float shoulderAngle1Counts = (shoulderAngle1Degrees-45) * conversion; // convert to encoder counts
          float shoulderAngle2Counts = shoulderAngle2Degrees * conversion; // convert to encoder counts
          float shoulderAngleCounts = shoulderAngle1Counts - shoulderAngle2Counts;
          float kneeAngleCounts = (kneeAngleDegrees-90) * conversion; // convert to encoder counts 
          float hipAngleCounts = hipAngle1Degrees * conversion;    // convert to encoder counts        
          driveJoints (61, shoulderAngleCounts);    // back left shoulder
          driveJoints (60, kneeAngleCounts);    // back left knee
          driveJoints (41, hipAngleCounts);     // back left hip
          
      }
      
      else if (leg == 4) {     // back right
          float shoulderAngle1Counts = (shoulderAngle1Degrees-45) * conversion; // convert to encoder counts
          float shoulderAngle2Counts = shoulderAngle2Degrees * conversion; // convert to encoder counts
          float shoulderAngleCounts = shoulderAngle1Counts - shoulderAngle2Counts;
          float kneeAngleCounts = (kneeAngleDegrees-90) * conversion; // convert to encoder counts 
          float hipAngleCounts = hipAngle1Degrees * conversion;    // convert to encoder counts        
          driveJoints (31, shoulderAngleCounts);    // back right shoulder
          driveJoints (30, kneeAngleCounts);    // back right knee
          driveJoints (11, hipAngleCounts);     // back right hip
      }





    
}
