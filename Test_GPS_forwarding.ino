#include <FlexCAN_T4.h>

FlexCAN_T4<CAN1, RX_SIZE_1024, TX_SIZE_1024> can1;
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> can2;
CAN_message_t msg;

// Vehicle position 65267
CAN_message_t vehiclePos;
// Custom GNSS time 65390
CAN_message_t customGNSSTime;
// Custom GNSS fix 65391
CAN_message_t customGNSSFix;
// Custon GNSS dop 65392
CAN_message_t customGNSSDop;

int incomingByte = 0;
int temp = 0;
bool isHeaderFound = false;
bool isPGNFound = false;
uint8_t tempHeader = 0;
uint8_t pgn = 0;
uint8_t cls = 0;
uint16_t dataLength = 0;
int64_t readI4 = 0;
uint64_t lon = 0;
uint64_t lat = 0;
uint16_t heading = 0;

void setup(void) {
  //pinMode(2, OUTPUT);
  //pinMode(3, OUTPUT);
  //digitalWrite(2,LOW);
  //digitalWrite(3,LOW);
  // Set messages
  vehiclePos.id = 0x18FEF380;
  vehiclePos.flags.extended = 1;
  vehiclePos.seq = 1;

  customGNSSTime.id = 0x18FF6E80;
  customGNSSTime.flags.extended = 1;
  customGNSSTime.seq = 1;

  customGNSSFix.id = 0x18FF6F80;
  customGNSSFix.flags.extended = 1;
  customGNSSFix.seq = 1;

  customGNSSDop.id = 0x18FF7080;
  customGNSSDop.flags.extended = 1;
  customGNSSDop.seq = 1;
  
  Serial.begin(38400);  // Start serial port
  Serial5.begin(115200);
  can1.begin();
  can1.setBaudRate(250000);
  can2.begin();
  can2.setBaudRate(250000);
}

void loop() {
  if (Serial5.available() > 1 && !isHeaderFound && !isPGNFound) 
  {
    uint8_t temp = Serial5.read();
    if (tempHeader == 0xb5 && temp == 0x62) // Check for ublox binary message 
    {
      isHeaderFound = true;
      tempHeader = 0;        
      //Serial.println("Header found!");
    }
    else  
    {
      tempHeader = temp;     //save for next time
      return;    
    }
  }

  //Find Source, PGN, and Length
  if (Serial5.available() > 3 && isHeaderFound && !isPGNFound)
  {
    cls = Serial5.read(); //Message class
    pgn = Serial5.read(); //Message ID
    dataLength = Serial5.read()| Serial5.read() << 8;
    isPGNFound = true;
    //Serial.print("Length of data: ");
    //Serial.println(dataLength);
  } 

  //The data package
  if (Serial5.available() > dataLength && isHeaderFound && isPGNFound)
  {
    if (cls == 0x01 && pgn == 0x07) //UBX-NAV-PVT
    {
      // Data bytes 0-3 iTOW, read in same byte order
      customGNSSTime.buf[0] = Serial5.read();
      customGNSSTime.buf[1] = Serial5.read();
      customGNSSTime.buf[2] = Serial5.read();
      customGNSSTime.buf[3] = Serial5.read();

      // Data bytes 4-10 GPS time
      // Year
      Serial5.read();
      Serial5.read();
      // Month
      Serial5.read();
      // Day
      Serial5.read();
      // Hour
      Serial5.read();
      // Min
      Serial5.read();
      // Sec
      Serial5.read();

      // Data byte 11 validity flags
      customGNSSFix.buf[2] = Serial5.read();
      
      // Data byte 12-15 time accuracy estimate
      Serial5.read();
      Serial5.read();
      Serial5.read();
      Serial5.read();

      // Data byte 16-19 nanoseconds
      Serial5.read();
      Serial5.read();
      Serial5.read();
      Serial5.read();

      // Data byte 20 fix type
      customGNSSFix.buf[3] = Serial5.read();

      // Data byte 21 fix status flags 
      customGNSSFix.buf[1] = Serial5.read();

      // Data byte 22 additional flags
      Serial5.read();

      // Data byte 23 number of satellites
      customGNSSFix.buf[0] = Serial5.read();

      // Data bytes 24-27 longitude, add +210 degree offset for uint
      readI4 = (Serial5.read() | Serial5.read() << 8 | Serial5.read() << 16 | Serial5.read() << 24);
      lon = readI4 + 2100000000;
      //Serial.println((double)lon*1e-7-210.0);
      vehiclePos.buf[0]=(byte)(lon);
      vehiclePos.buf[1]=(byte)(lon >> 8);
      vehiclePos.buf[2]=(byte)(lon >> 16);
      vehiclePos.buf[3]=(byte)(lon >> 24);

      // Data bytes 28-31 latitude, add +210 degree offset for uint
      readI4 = (Serial5.read() | Serial5.read() << 8 | Serial5.read() << 16 | Serial5.read() << 24);
      lat = readI4 + 2100000000;
      //Serial.println((double)lat*1e-7-210.0);
      vehiclePos.buf[4]=(byte)(lat);
      vehiclePos.buf[5]=(byte)(lat >> 8);
      vehiclePos.buf[6]=(byte)(lat >> 16);
      vehiclePos.buf[7]=(byte)(lat >> 24);

      // Data bytes 32-35 height above ellipsoid
      customGNSSFix.buf[4]=Serial5.read();
      customGNSSFix.buf[5]=Serial5.read();
      customGNSSFix.buf[6]=Serial5.read();
      customGNSSFix.buf[7]=Serial5.read();

      // Data bytes 36-39 height above MSL
      Serial5.read();
      Serial5.read();
      Serial5.read();
      Serial5.read();

      // Data bytes 40-43 hAcc, this is 32 bit uint, truncated to 16 bit uint so take LSB
      customGNSSDop.buf[0]=Serial5.read();
      customGNSSDop.buf[1]=Serial5.read();
      Serial5.read();
      Serial5.read();

      // Data bytes 44-47 vAcc, this is 32 bit uint, truncated to 16 bit uint so take LSB
      customGNSSDop.buf[2]=Serial5.read();
      customGNSSDop.buf[3]=Serial5.read();
      Serial5.read();
      Serial5.read();

      // Data bytes 48-51 velN
      Serial5.read();
      Serial5.read();
      Serial5.read();
      Serial5.read();

      // Data bytes 52-55 velE
      Serial5.read();
      Serial5.read();
      Serial5.read();
      Serial5.read();
      
      // Data bytes 56-59 velD
      Serial5.read();
      Serial5.read();
      Serial5.read();
      Serial5.read();

      // Data bytes 60-63 ground speed mm/s
      customGNSSTime.buf[4] = Serial5.read();
      customGNSSTime.buf[5] = Serial5.read();
      customGNSSTime.buf[6] = Serial5.read();
      customGNSSTime.buf[7] = Serial5.read();

      // Data bytes 64-67 heading of motion NEEDS EDITING, CANT JUST TRUNCATE
      readI4 = (Serial5.read() | Serial5.read() << 8 | Serial5.read() << 16 | Serial5.read() << 24);
      heading = readI4/1000;
      //Serial.println(0.01*heading);
      customGNSSDop.buf[6]=(byte)(heading);
      customGNSSDop.buf[7]=(byte)(heading >> 8);

      // Data bytes 68-71 speed accuracy sAcc
      Serial5.read();
      Serial5.read();
      Serial5.read();
      Serial5.read();

      // Data bytes 72-75 heading accuracy
      Serial5.read();
      Serial5.read();
      Serial5.read();
      Serial5.read(); 

      // Data bytes 76-77 position DOP pDOP
      customGNSSDop.buf[4]=Serial5.read();
      customGNSSDop.buf[5]=Serial5.read();

      // Data bytes 78-79 additional flags
      Serial5.read();
      Serial5.read(); 

      // Data bytes 80-83 reserved
      Serial5.read();
      Serial5.read(); 
      Serial5.read();
      Serial5.read(); 

      // Data bytes 84-87 heading of vehicle
      Serial5.read();
      Serial5.read(); 
      Serial5.read();
      Serial5.read(); 

      // Data bytes 88-89 magnetic declination
      Serial5.read();
      Serial5.read(); 

      // Data bytes 90-91 magnetic declination accuracy
      Serial5.read();
      Serial5.read(); 
      
      //reset for next pgn sentence
      isHeaderFound = isPGNFound = false;
      pgn=dataLength=0;      
      //Serial.println("Data ready!");
      msg = customGNSSFix;
      can1.write(msg);
      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
      Serial.print("  EXT: "); Serial.print(msg.flags.extended );
      Serial.print("  LEN: "); Serial.print(msg.len);
      Serial.print(" DATA: ");
      for ( uint8_t i = 0; i < 8; i++ ) {
      Serial.print(msg.buf[i], HEX); Serial.print(" ");
      }
      Serial.println("");
      msg = customGNSSDop;
      //can1.write(msg);
      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
      Serial.print("  EXT: "); Serial.print(msg.flags.extended );
      Serial.print("  LEN: "); Serial.print(msg.len);
      Serial.print(" DATA: ");
      for ( uint8_t i = 0; i < 8; i++ ) {
        Serial.print(msg.buf[i], HEX); Serial.print(" ");
      }
      Serial.println("");
      msg = vehiclePos;
      //can1.write(msg);
      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
      Serial.print("  EXT: "); Serial.print(msg.flags.extended );
      Serial.print("  LEN: "); Serial.print(msg.len);
      Serial.print(" DATA: ");
      for ( uint8_t i = 0; i < 8; i++ ) {
        Serial.print(msg.buf[i], HEX); Serial.print(" ");
      }
      Serial.println("");
      msg = customGNSSTime;
      //can1.write(msg);
      Serial.print("  ID: 0x"); Serial.print(msg.id, HEX );
      Serial.print("  EXT: "); Serial.print(msg.flags.extended );
      Serial.print("  LEN: "); Serial.print(msg.len);
      Serial.print(" DATA: ");
      for ( uint8_t i = 0; i < 8; i++ ) {
        Serial.print(msg.buf[i], HEX); Serial.print(" ");
      }
      Serial.println("");    
      Serial.println("");
    }
  }
}
