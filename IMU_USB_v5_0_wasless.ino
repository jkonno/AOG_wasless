 /*      
  * USB IMU code for For AgOpenGPS
  * 4 Feb 2021, Brian Tischler
  * Like all Arduino code - copied from somewhere else :)
  * So don't claim it as your own
  */

  #include <Wire.h>
  #include "TinyGPS++.h"
  #include "BNO08x_AOG.h"
  #define CMPS14_ADDRESS 0x60    // Address of CMPS14 shifted right one bit for arduino wire library
  #include <Kalman.h>
  using namespace BLA;

  // BNO08x definitions
  #define REPORT_INTERVAL 20 //Report interval in ms (same as the delay at the bottom)
  
  #define CONST_180_DIVIDED_BY_PI 57.2957795130823

  // Setup NMEA parsing stuff
  TinyGPSPlus gps;
  TinyGPSCustom speedGPS(gps, "GNVTG", 7);
  TinyGPSCustom headingGPS(gps, "GNVTG", 1);
  double old_lat = 0.0;
  double old_lon = 0.0;
  double lat = 0.0;
  double lon = 0.0;
  double dist = 0.0;
  double heading = 0.0;

  // Setup the Kalman filter variables

  #define Nstate 6 // heading, z-rate, z-bias, velocity, x-acc, x-bias
  #define Nobs 2   // heading, speed
  #define Ncom 1 // pitch rate

  BLA::Matrix<Nobs> obs; // observation vector
  KALMAN<Nstate,Nobs,Ncom> K; // your Kalman filter
  unsigned long T; // current time
  double deltaT; // delay between two updates of the filter


  //CMPS PGN - 211
  uint8_t data[] = {0x80,0x81,0x7D,0xD3,8, 0,0,0,0, 0,0,0,0, 15};
  int16_t dataSize = sizeof(data);

  // booleans to see if we are using CMPS or BNO08x
  bool useCMPS = false;
  bool useBNO08x = false;

  // BNO08x address variables to check where it is
  const uint8_t bno08xAddresses[] = {0x4A,0x4B};
  const int16_t nrBNO08xAdresses = sizeof(bno08xAddresses)/sizeof(bno08xAddresses[0]);
  uint8_t bno08xAddress;
  BNO080 bno08x;

  float bno08xHeading = 0;
  float yawRate = 0;
  float pitchRate = 0;
  float xAcc = 0;
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
    Serial.println("Checking for CMPS14");
    Wire.beginTransmission(CMPS14_ADDRESS);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.println("Error = 0");
      Serial.print("CMPS14 ADDRESs: 0x");
      Serial.println(CMPS14_ADDRESS, HEX);
      Serial.println("CMPS14 Ok.");
      useCMPS = true;
    }
    else 
    {
      Serial.println("Error = 4");
      Serial.println("CMPS not Connected or Found"); 
    }

    if(!useCMPS)
    {
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

            delay(100);
            
            // Use gameRotationVector
            bno08x.enableGyro(REPORT_INTERVAL);
            bno08x.enableGameRotationVector(REPORT_INTERVAL-1); //Send data update every REPORT_INTERVAL in ms for BNO085, looks like this cannot be identical to the other reports for it to work...
            bno08x.enableAccelerometer(REPORT_INTERVAL);
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
    }
    T = millis();
  }
  
  void loop()
  {

    // TIME COMPUTATION
    deltaT = (millis()-T)/1000.0;
    T = millis();
    
    if(useCMPS)
    {
        Wire.beginTransmission(CMPS14_ADDRESS);  
        Wire.write(0x02);                     
        Wire.endTransmission();
    
        Wire.requestFrom(CMPS14_ADDRESS, 2); 
        while(Wire.available() < 2);       
  
        //the heading x10
        data[6] = Wire.read();
        data[5] = Wire.read();
   
        Wire.beginTransmission(CMPS14_ADDRESS);  
        Wire.write(0x1C);                    
        Wire.endTransmission();
   
        Wire.requestFrom(CMPS14_ADDRESS, 2);  
        while(Wire.available() < 2);        
  
        //the roll x10
        data[8] = Wire.read();
        data[7] = Wire.read();
    } 
    else if(useBNO08x)
    {
      if (bno08x.dataAvailable() == true)
      {
        bno08xHeading = (bno08x.getYaw()) * CONST_180_DIVIDED_BY_PI; // Convert yaw / heading to degrees
        bno08xHeading = -bno08xHeading; //BNO085 counter clockwise data to clockwise data
        yawRate = (bno08x.getGyroZ()) * CONST_180_DIVIDED_BY_PI; // Get raw yaw rate
        pitchRate = (bno08x.getGyroY()) * CONST_180_DIVIDED_BY_PI; // Get raw pitch rate
        xAcc = (bno08x.getAccelX()); // Get x-acceleration for speed fusion
        if (bno08xHeading < 0 && bno08xHeading >= -180) //Scale BNO085 yaw from [-180°;180°] to [0;360°]
        {
          bno08xHeading = bno08xHeading + 360;
        }
            
        bno08xRoll = (bno08x.getRoll()) * CONST_180_DIVIDED_BY_PI; //Convert roll to degrees
        bno08xPitch = (bno08x.getPitch())* CONST_180_DIVIDED_BY_PI; // Convert pitch to degrees

        bno08xHeading10x = (int16_t)(bno08xHeading * 10);
        bno08xRoll10x = (int16_t)(bno08xRoll * 10);
        
        //the heading x10
        data[5] = (uint8_t)bno08xHeading10x;
        data[6] = bno08xHeading10x >> 8;

        //the roll x10
        data[7] = (uint8_t)bno08xRoll10x;
        data[8] = bno08xRoll10x >> 8;

        // Build the Kalman filter matrices
        // Process matrix
        K.F = {1.0,-deltaT*cos(bno08xRoll)/cos(bno08xPitch),0.0,0.0,0.0,0.0,
                                             0.0,1.0,0.0,0.0,0.0,0.0,
                                            0.0,0.0,1.0,0.0,0.0,0.0,
                                            0.0,0.0,0.0,1.0,deltaT,0.0,
                                            0.0,0.0,0.0,0.0,1,0.0,
                                            0.0,0.0,0.0,0.0,0.0,1.0};
        // Control matrix
        K.B = {-deltaT*sin(bno08xRoll)/cos(bno08xPitch),0.0,0.0,0.0,0.0,0.0};

        // Check if we've read a full NMEA sentence into TinyGPS, then we're gonna do a GPS update on the filter
        if (stringComplete) {
          stringComplete = false;
          // If position is updates, calculate fix2fix heading and update lat/lon
          if(gps.location.isUpdated()) {
                lat = gps.location.lat();
                lon = gps.location.lng();
                dist = TinyGPSPlus::distanceBetween(lat,lon,old_lat,old_lon);
                if (dist > 0.5) {
                  heading = TinyGPSPlus::courseTo(old_lat,old_lon,lat,lon);
                  old_lat = lat;
                  old_lon = lon;
                }
                // GPS update measurement matrix, unit conversion from km/h to m/s
                K.H = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 3.6, 0.0, 0.0};
          }
        } 
        // Here we do the IMU update
        else {
              // IMU measurement matrix
              K.H = {0.0, 1.0, -1.0, 0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0, 0.0, 1.0, -1.0};
     
        }

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

    //10 hz
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
