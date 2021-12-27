 /*      
  * WASless IMU + GPS fusieon code for For AgOpenGPS
  * 4 Feb 2021, Brian Tischler
  * 18 Dec 2021, Juho Könnö
  * Like all Arduino code - copied from somewhere else :)
  * So don't claim it as your own
  */

  #include <Wire.h>
  #include "TinyGPS++.h"
  #include "BNO08x_AOG.h"
  #include <Kalman.h>
  using namespace BLA;

  // BNO08x definitions
  // Run filter at 100 Hz raw data, update orientation at 50 Hz
  #define REPORT_INTERVAL 10 //Report interval in ms (same as the delay at the bottom)
  
  #define CONST_180_DIVIDED_BY_PI 57.2957795130823
  #define CONST_PI 3.141592653589793

  // IMU parameters
  #define BIW 0.0001454441043328608   // 0.5 deg / min in rad/s bias instability
  #define ARW 0.00034906585039886593  // 1.2 deg dynamic rotation vector accuracy = 3.0 deg / sqrt(h) ARW in rad / sqrt(s)
  #define BIA 0.014715                // 1.5 mg acceleration bias in m/s/s
  #define VRW 0.002                   // 0.12 m/s/sqrt(h) in m/s/sqrt(s)

  double sigma_dd = 9.064720283654388*BIW*BIW/ARW;   // 2*PI/LN(2)*BIW^2/ARW, process bias covariance for yaw rate
  double sigma_aa = 9.064720283654388*BIA*BIA/VRW;   // 2*PI/LN(2)*BIA^2/VRW, process bias covariance for acceleration
  
  // Setup NMEA parsing stuff with TinyGPS++ library
  TinyGPSPlus gps;
  // Parse heading and speed from VTG
  TinyGPSCustom speedGPS(gps, "GNVTG", 7);
  TinyGPSCustom headingGPS(gps, "GNVTG", 1);
  // Some aux variables
  double old_lat = 0.0;
  double old_lon = 0.0;
  double lat = 0.0;
  double lon = 0.0;
  double dist = 0.0;
  double heading = 0.0;
  double machineYawRate = 0.0;
  double wasAngle = 0.0;
  double wheelBase = 2.76;
  double wasSpeedLimit = 2.0;
  double measuredGPSSpeed = 0.0;
  double stabTimeCounter = 0.0;
  double xAccAccumulated = 0.0;
  double yawRateAccumulated = 0.0;
  double headingBias = 0.0;
  double headingShift = 0.0;
  double headingOld = 0.0;
  double headingKF = 0.0;
  bool imuFlag = false;
  
  // Setup the Kalman filter variables
  #define Nstate 6 // heading, z-rate, z-bias, velocity, x-acc, x-bias
  #define Nobs 2   // heading, speed
  #define Ncom 1 // pitch rate
  BLA::Matrix<Nobs> obs; // observation vector
  BLA::Matrix<Ncom> ctrl; // observation vector
  KALMAN<Nstate,Nobs,Ncom> K; // your Kalman filter
  unsigned long T; // current time
  // Covariance tuning parameters
  double deltaT; // delay between two updates of the filter
  double s2yaw = 0.49; // covariance for yaw, based on process noise 40m/s2 max accel.
  double s2acc = 10000; // covariance for acceleration based on 100 m/s^3 jerk
  double rGPS = 0.0001; //GPS meas covariance
  double rIMU = 0.01; //IMU meas covariance
  int turnSwitch = 0;

  //CMPS PGN - 211
  uint8_t data[] = {0x80,0x81,0x7D,0xD3,8, 0,0,0,0, 0,0,0,0, 15};
  int16_t dataSize = sizeof(data);

  // booleans to see if we are using BNO08x and if we're in stabilize mode
  bool useBNO08x = false;
  bool stabilizeMode = true;

  // BNO08x address variables to check where it is
  const uint8_t bno08xAddresses[] = {0x4A,0x4B};
  const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses)/sizeof(bno08xAddresses[0]);
  uint8_t bno08xAddress;
  BNO080 bno08x;

  double bno08xHeading = 0;
  double yawRate = 0;
  double pitchRate = 0;
  double xAcc = 0;
  double bno08xRoll = 0;
  double bno08xPitch = 0;

  int16_t bno08xHeading10x = 0;
  int16_t bno08xRoll10x = 0;

  bool stringComplete = false;  // whether the string is complete

  void setup()
  {
    Serial.begin(38400);  // Start serial port
    Serial2.begin(38400);
    Wire.begin();

    //test if CMPS working
    uint8_t error;
    for(int16_t i = 0; i < nrBNO08xAdresses; i++)
    {
      bno08xAddress = bno08xAddresses[i];
      
      Serial.print("\r\nChecking for BNO08X on ");
      Serial.println(bno08xAddress, HEX);
      Wire.beginTransmission(bno08xAddress);
      error = Wire.endTransmission();
    
      if (error == 0)
      {
        Serial.println("Error = 0");
        Serial.print("BNO08X ADDRESs: 0x");
        Serial.println(bno08xAddress, HEX);
        Serial.println("BNO08X Ok.");
        Serial.println("");
        Serial.println("Starting logging for following quantities:");
        Serial.println("latitude,longitude,speed (km/h),heading_fix2fix (deg),heading_vtg (deg),heading_bno (deg),yaw_rate (deg/s),pitch_rate (deg/s),bno_roll (deg),bno_pitch (deg),x_acc (m/s2)");
        // Initialize BNO080 lib        
        if (bno08x.begin(bno08xAddress))
        {
          Wire.setClock(400000); //Increase I2C data rate to 400kHz

          // Add bit of delay to make sure things are up...
          delay(100);
            
          // Enable raw gyro data at double rate
          bno08x.enableGyro(REPORT_INTERVAL);
          // Enable gameRotationVector mode
          bno08x.enableGameRotationVector(REPORT_INTERVAL-1); //Send data update every REPORT_INTERVAL in ms for BNO085, looks like this cannot be identical to the other reports for it to work...
          // Enable raw accelerometer
          bno08x.enableAccelerometer(REPORT_INTERVAL+1);
          delay(100);
          // Retrieve the getFeatureResponse report to check if Rotation vector report is corectly enable
          if (bno08x.getFeatureResponseAvailable() == true)
          {
            if (bno08x.checkReportEnable(SENSOR_REPORTID_GAME_ROTATION_VECTOR, REPORT_INTERVAL) == false) bno08x.printGetFeatureResponse();

            // Break out of loop
            useBNO08x = true;
            break;
          }
          else 
          {
            Serial.println("BNO08x init fails!!");
          }
        }
        else
        {
          Serial.println("BNO080 not detected at given I2C address.");
        }
      }
      else 
      {
        Serial.println("Error = 4");
        Serial.println("BNO08X not Connected or Found"); 
      }
    }

    // Set initial time for timestep computations
    T = millis();
  }
  
  void loop()
  {

    // TIME COMPUTATION
    deltaT = (millis()-T)/1000.0;
    T = millis();

    if(useBNO08x)
    {
      if (bno08x.dataAvailable() == true)
      {
        // Stuff here is in the IMU coordinate frame!
        bno08xHeading = bno08x.getYaw();                          // Heading in radian
        bno08xHeading = -bno08xHeading;                           // BNO085 counter clockwise data to clockwise data
        yawRate = bno08x.getGyroZ();                              // Get raw yaw rate in radian
        pitchRate = bno08x.getGyroY();                            // Get raw pitch rate in radian
        xAcc = bno08x.getAccelX();                                // Get x-acceleration for speed fusion
        if (bno08xHeading < 0 && bno08xHeading >= -1.0*CONST_PI)  // Scale BNO085 yaw from [-180°;180°] to [0;360°]
        {
          bno08xHeading = bno08xHeading + 2.0*CONST_PI;
        }
            
        //bno08xRoll = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI; //Convert roll to degrees
        //bno08xPitch = (bno08x.getPitch())* CONST_180_DIVIDED_BY_PI; // Convert pitch to degrees

        // Check for NaN so that the matrices don't get screwed up at start
        if (isnan(bno08xRoll)) bno08xRoll = 0.0;
        if (isnan(bno08xPitch)) bno08xPitch = 0.0;

        // Stuff to be sent to AOG, modify later...

        bno08xHeading10x = (int16_t)(bno08xHeading * 10);
        bno08xRoll10x = (int16_t)(bno08xRoll * 10);
        
        //the heading x10
        data[5] = (uint8_t)bno08xHeading10x;
        data[6] = bno08xHeading10x >> 8;

        //the roll x10
        data[7] = (uint8_t)bno08xRoll10x;
        data[8] = bno08xRoll10x >> 8;

        // Run the Kalman filter starts here:
        
        // Build the Kalman filter matrices, different for each time step
        
        // Process matrix
        K.F = {1.0,-deltaT*cos(bno08xRoll)/cos(bno08xPitch),0.0,0.0,0.0,0.0,
                                            0.0,1.0,0.0,0.0,0.0,0.0,
                                            0.0,0.0,1.0,0.0,0.0,0.0,
                                            0.0,0.0,0.0,1.0,deltaT,0.0,
                                            0.0,0.0,0.0,0.0,1,0.0,
                                            0.0,0.0,0.0,0.0,0.0,1.0};
                                            
        // Control matrix
        K.B = {-deltaT*sin(bno08xRoll)/cos(bno08xPitch),0.0,0.0,0.0,0.0,0.0};
        
        // Model covariance matrix
        K.Q = {s2yaw*pow(deltaT,4),s2yaw*pow(deltaT,3),0.0,0.0,0.0,0.0,
               s2yaw*pow(deltaT,3),s2yaw*pow(deltaT,2),0.0,0.0,0.0,0.0,
               0.0,0.0,pow(deltaT*sigma_dd,2),0.0,0.0,0.0,
               0.0,0.0,0.0,s2acc*pow(deltaT,4),s2acc*pow(deltaT,3),0.0,
               0.0,0.0,0.0,s2acc*pow(deltaT,3),s2acc*pow(deltaT,2),0.0,
               0.0,0.0,0.0,0.0,0.0,pow(deltaT*sigma_aa,2)};

        // Check if we've read a full NMEA sentence into TinyGPS, then we're gonna do a GPS update on the filter 
        // i.e. update on GPS whenever it's availabe
        if (stringComplete) {
          stringComplete = false;
          // If position is updated, calculate fix2fix heading and update lat/lon
          if(gps.location.isUpdated()) {
                Serial.println("GPS UPDATE");
                lat = gps.location.lat();
                lon = gps.location.lng();
                dist = TinyGPSPlus::distanceBetween(lat,lon,old_lat,old_lon);
                if (dist > 0.5) {
                  heading = TinyGPSPlus::courseTo(old_lat,old_lon,lat,lon)/CONST_180_DIVIDED_BY_PI; // in radian as well
                  old_lat = lat;
                  old_lon = lon;
                }

                // GPS speed
                measuredGPSSpeed = atof(speedGPS.value());
      
                // Set measurement matrix to GPS update matrix, unit conversion from km/h to m/s for speed
                K.H = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 3.6, 0.0, 0.0};
                // Set measurement uncertainty for GPS
                K.R = {atan(0.05/(measuredGPSSpeed/3.6+0.1)),   0.0,
                       0.0,   0.025};
                
                // Observations and control variable update from sensors 
                
                // Check for speed over WAS limit
                if (measuredGPSSpeed > wasSpeedLimit && yawRate < 6.0/CONST_180_DIVIDED_BY_PI) {
                  imuFlag = false;
                  // Use GPS heading
                  obs(0) = heading;
                  // Check for direction of travel
                  if (K.x(3) < 0) {
                    // Reverse direction
                    obs(1) = -measuredGPSSpeed;
                  } else {
                    // Forward direction
                    obs(1) = measuredGPSSpeed;
                  }
                } else {
                  //Serial.println("IMU LOW SPEED");
                  
                  if (!imuFlag) {
                    // First time going to IMU heading fix IMU to last GPS heading
                    headingBias = heading - bno08xHeading;
                    imuFlag = true;
                  }
                  if (imuFlag) {
                    obs(0) = bno08xHeading + headingBias;
                  }
                  
                  // Use IMU heading
                  //obs(0) = bno08xHeading;
                  // Below limit set speed to zero
                  obs(1) = 0.0;
                }
                
                // Force heading variable between 0 and 2 Pi
                // Check if the jump is close to 2Pi and assume the angle is going over, this is to prevent jumps in the variable
                if ((obs(0) - headingOld) > CONST_PI) {
                  headingShift -= 2.0*CONST_PI; 
                  //Serial.println("PLUSSFHIFT");
                } else if ((obs(0) - headingOld) < -CONST_PI) {
                  headingShift += 2.0*CONST_PI;
                  //Serial.println("MINUSSHIFT");
                }

                headingOld = obs(0);
                //Serial.println(headingShift);
                obs(0) = obs(0) + headingShift;
                
                // Pitchrate for control vector
                ctrl(0) = pitchRate;     
                
                //Serial << obs << ctrl << '\n';
          }
        } 
        // Here we do the IMU update whenever there's no fresh GPS data
        else {
              //Serial.println("IMU UPDATE");
              // Set measurement matrix to IMU update matrix
              K.H = {0.0, 1.0, -1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, -1.0};
              // Set measurement uncertainty for IMU
              K.R = {ARW*ARW,   0.0,
                       0.0,   VRW*VRW};

              // Observations and control variable update from sensors
              obs(0) = yawRate;
              obs(1) = xAcc;
              ctrl(0) = pitchRate;
        }
        
        // Stabilize for 5 secs at startup
        if (stabilizeMode) {
          stabTimeCounter += deltaT;
          xAccAccumulated += xAcc*deltaT;
          yawRateAccumulated += yawRate*deltaT;
          if (stabTimeCounter > 5.0){
            stabilizeMode = false;
            // Set initial biases
            K.x(2) = -yawRateAccumulated/stabTimeCounter;
            K.x(5) = -xAccAccumulated/stabTimeCounter;
            Serial.print("Stabilization done, yaw bias: ");
            Serial.print(K.x(2));
            Serial.print(", acceleration bias: ");
            Serial.println(K.x(5));
          }
        } else {
          // Run the filter step
          K.update(obs,ctrl);

          headingKF = K.x(0) - headingShift;
          
          // Force heading variable between 0 and 2 Pi (in case the IMU integration runs over the limit)
         /*if (headingKF > 2.0*CONST_PI) {
           headingKF -= 2.0*CONST_PI;  
         } else if (headingKF < 0.0) {
            headingKF += 2.0*CONST_PI;
         }*/

          // Calculate true yaw rate in navigation frame
          machineYawRate = (K.x(1)-K.x(2))*cos(bno08xRoll)/cos(bno08xPitch)+pitchRate*sin(bno08xRoll)/cos(bno08xPitch);

          // Calculate virtual steering angle
          // Check speed to avoid division by zero, set zero for now...
          if (abs(K.x(3)) > wasSpeedLimit/3.6) {
            wasAngle = atan(machineYawRate*wheelBase/K.x(3));
          } else {
            wasAngle = 0.0;
          } 
          Serial.print(headingKF*CONST_180_DIVIDED_BY_PI);
          //Serial.print(heading);
          Serial.print('\t'); 
          //Serial.print(K.x(3)*3.6);
          Serial.print('\t');
          Serial.print(wasAngle*CONST_180_DIVIDED_BY_PI);
          Serial.print('\t');
          if (yawRate > 6.0/CONST_180_DIVIDED_BY_PI) {
            Serial.println("50");
          } else {
            Serial.println("0");
          }
          //Serial.println((bno08xHeading)*CONST_180_DIVIDED_BY_PI);
        }

        /*
        // Dump data to Serial 1 (USB) for testing / saving
        Serial.print(lat,12);
        Serial.print(",");
        Serial.print(lon,12);
        Serial.print(",");
        Serial.print(speedGPS.value());
        Serial.print(",");
        Serial.print(heading,8);
        Serial.print(",");
        Serial.print(headingGPS.value());
        Serial.print(",");
        Serial.print(bno08xHeading,8);
        Serial.print(",");
        Serial.print(yawRate,8);
        Serial.print(",");
        Serial.print(pitchRate,8);
        Serial.print(",");
        Serial.print(bno08xRoll,8);
        Serial.print(",");
        Serial.print(bno08xPitch,8);
        Serial.print(",");
        Serial.println(xAcc,8);
        */

      }
    }

    int16_t CK_A = 0;
    
    for (int16_t i = 2; i < dataSize - 1; i++)
    {
        CK_A = (CK_A + data[i]);
    }
    
    data[dataSize - 1] = CK_A;

    //Serial.write(data, dataSize);
    //Serial.println();
    //Serial.flush();

    //100 hz
    delay(REPORT_INTERVAL);                           
  }


  // Read from Serial2 until full NMEA sentence and feed to TinyGPS
  void serialEvent2() {
  while (Serial2.available()) {
    // get the new byte:
    char inChar = (char)Serial2.read();
    gps.encode(inChar);
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}
