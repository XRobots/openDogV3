// init hips

void OdriveInit1() {

      Serial.println("ODrive 1");

      for (int axis = 0; axis < 2; ++axis) {
          //Serial1 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';          // *** Velocity limits should be set to infinite through the ODrive tool, this stops the motors disarming under certain situations ***
          Serial1 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  
          requested_state = requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive1.run_state(axis, requested_state, false); // don't wait 
      }      

     Serial.println("ODrive 2");

      for (int axis = 0; axis < 2; ++axis) {
          //Serial2 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
          Serial2 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  
          requested_state = requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial2 << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive2.run_state(axis, requested_state, false); // don't wait 
      }   

      Serial.println("ODrive 3");

      for (int axis = 0; axis < 2; ++axis) {
          //Serial3 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
          Serial3 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  
          requested_state = requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial3 << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive3.run_state(axis, requested_state, false); // don't wait 
      } 

      Serial.println("ODrive 4");

      for (int axis = 0; axis < 2; ++axis) {
          //Serial4 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
          Serial4 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  
          requested_state = requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial4 << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive4.run_state(axis, requested_state, false); // don't wait 
      }

      Serial.println("ODrive 5");

      for (int axis = 0; axis < 2; ++axis) {
          //Serial5 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
          Serial5 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  
          requested_state = requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial5 << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive5.run_state(axis, requested_state, false); // don't wait 
      }

      Serial.println("ODrive 6");

      for (int axis = 0; axis < 2; ++axis) {
          //Serial6 << "w axis" << axis << ".controller.config.vel_limit " << 6000.0f << '\n';
          Serial6 << "w axis" << axis << ".motor.config.current_lim " << 20.0f << '\n';
  
          requested_state = requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL;
          Serial6 << "Axis" << axis << ": Requesting state " << requested_state << '\n';
          odrive6.run_state(axis, requested_state, false); // don't wait 
      }
}


void modifyGains() {                               // this function turns up the gains when it is executed (menu option 4 via the remote)

          Serial.println("modfy gains");

          float posGainKnee = 15.0;
          float posGainHips = 70.0;  
          float posGainShoulder = 15.0; 
          float velGain = 0.1;      
          float integrator = 0.1;  

          Serial1 << "w axis" << 0 << ".controller.config.pos_gain " << posGainHips << '\n';
          Serial1 << "w axis" << 1 << ".controller.config.pos_gain " << posGainHips << '\n';

          Serial2 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
          Serial2 << "w axis" << 1 << ".controller.config.pos_gain " << posGainShoulder << '\n';

          Serial3 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
          Serial3 << "w axis" << 1 << ".controller.config.pos_gain " << posGainShoulder << '\n';

          Serial4 << "w axis" << 0 << ".controller.config.pos_gain " << posGainHips << '\n';
          Serial4 << "w axis" << 1 << ".controller.config.pos_gain " << posGainHips << '\n';

          Serial5 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
          Serial5 << "w axis" << 1 << ".controller.config.pos_gain " << posGainShoulder << '\n';

          Serial6 << "w axis" << 0 << ".controller.config.pos_gain " << posGainKnee << '\n';
          Serial6 << "w axis" << 1 << ".controller.config.pos_gain " << posGainShoulder << '\n';

          // ******

          Serial1 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
          Serial1 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

          Serial2 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
          Serial2 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

          Serial3 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
          Serial3 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

          Serial4 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
          Serial4 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

          Serial5 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
          Serial5 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

          Serial6 << "w axis" << 0 << ".controller.config.vel_gain " << velGain << '\n';
          Serial6 << "w axis" << 1 << ".controller.config.vel_gain " << velGain << '\n';

          // ******

          Serial1 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
          Serial1 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

          Serial2 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
          Serial2 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

          Serial3 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
          Serial3 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

          Serial4 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
          Serial4 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

          Serial5 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
          Serial5 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';

          Serial6 << "w axis" << 0 << ".controller.config.vel_integrator_gain " << integrator << '\n';
          Serial6 << "w axis" << 1 << ".controller.config.vel_integrator_gain " << integrator << '\n';        
}

void applyOffsets1() {
          // apply initial offsets to ODrives for hips         
          
          odrive1.SetPosition(0, offSet10);         // hips - right front
          odrive1.SetPosition(1, offSet11);         // hips - right rear
          odrive4.SetPosition(0, offSet40);         // hips - left front
          odrive4.SetPosition(1, offSet41);         // hips - left rear 
}

void applyOffsets2() {
          // apply initial offsets to ODrives for shoulders

          odrive2.SetPosition(1, offSet21);         // shoulder - right front
          odrive3.SetPosition(1, offSet31);         // shoulder - right rear       
          odrive5.SetPosition(1, offSet51);         // shoulder - left front
          odrive6.SetPosition(1, offSet61);         // shoulder - left rear 
          odrive2.SetPosition(0, offSet20);         // knee - right front
          odrive3.SetPosition(0, offSet30);         // knee - right rear        
          odrive5.SetPosition(0, offSet50);         // knee - left front
          odrive6.SetPosition(0, offSet60);         // knee - left front   
              
}




void driveJoints(int joint, float pos) {
          // takes into account the original setup offsets for motor postions, and also turns around directions so they are consistent
          // also constrains the motion limts for each joint    

          if (mydata_remote.toggleTop == 1) {       // ************** only do it if the motor enable is on via teh remote *****************

              // knees

              if (joint == 20) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive2.SetPosition(0, pos + offSet20);    // knee - right front
              }
              else if (joint == 30) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive3.SetPosition(0, (pos*-1) + offSet30);    // knee - right back
              }
              else if (joint == 50) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive5.SetPosition(0, (pos*-1) + offSet50);    // knee - left front
              }
              else if (joint == 60) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive6.SetPosition(0, pos + offSet60);    // knee - left back
              }

              // shoulder

              else if (joint == 21) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive2.SetPosition(1, (pos*-1) + offSet21);    // shoulder - right front
              }        
              else if (joint == 31) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive3.SetPosition(1, pos + offSet31);    // shoulder - right rear
              }        
              else if (joint == 51) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive5.SetPosition(1, pos + offSet51);    // shoulder - left front
              }        
              else if (joint == 61) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive6.SetPosition(1, (pos*-1) + offSet61);    // shoulder - left rear      
              }

              // hips
              else if (joint == 10) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive1.SetPosition(0, pos+offSet10);    // hips - right front
              }
              else if (joint == 11) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive1.SetPosition(1, (pos*-1)+offSet11);    // hips - right rear
              }
              else if (joint == 40) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive4.SetPosition(0, pos+offSet40);    // hips - knee - left front
              }
              else if (joint == 41) {
                  pos = constrain(pos, -2.5,2.5);
                  odrive4.SetPosition(1, (pos*-1)+offSet41);    // hips - left rear
              }
     

          }

          

          
}
