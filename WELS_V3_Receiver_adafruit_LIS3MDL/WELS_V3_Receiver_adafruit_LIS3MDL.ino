/*

                         ____         _____                _____       __           ____
            \        /  |      |     |_____       \    /        |     |__|    /\   |     |  /
             \  /\  /   |---   |           |       \  /    -----|     |   \  /__\  |     | /
              \/  \/    |____  |____  _____|        \/     _____|     |___/ /    \ |____ |  \


    DISCLAIMER!!! - I did not write any of the code that is responsible for the MPU-9250.
   The code I wrote for that never really worked
   and this was really long and lot so I just used someone else is.
   I'm not 100% sure were the rights stand. The creater can be found here: https://www.instructables.com/Tilt-Compensated-Compass/
   Every other line of code I wrote, which is not many.

   If it comes down to it, I will rewrite the code that isn't mine but as far a I know it was open sorce. Please don't sue me.
   --Jake Donnini

   This version uses pulse functions and relies on magnetometers and not gyros. It also incorperates the tilt compesation code if that was obvous enough.

      This code is designed to be used with the MPU-9250/6500
*/
                                                       ///////////////////////////////////
                                                       //         Configuration         //
                                                       ///////////////////////////////////

                                                       bool useDirectionSensors = true;
//                             Turn this to false to turn of direction control, aka, it will vibrate in one spot.

                                                         bool Record_data = false;
//                           Turn this to ture to start the mag claibration, only needs to be done once in a while.

                                                           bool vibePulse = false;
//                 If true the vibrators will pulse faster the closer the object, if false intesity of the vibrator will increase

                                                         bool gyroStabization = false;
//                              if false it will turn off gyro stabization of the magentometer output



#include <RF24Network.h>
#include <RF24Network_config.h>
#include <Sync.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>

#include <Adafruit_LIS3MDL.h>
#include <Adafruit_Sensor.h>

Adafruit_LIS3MDL lis3mdl;
#define LIS3MDL_CLK 13
#define LIS3MDL_MISO 12
#define LIS3MDL_MOSI 11
#define LIS3MDL_CS 10

//------------------------------
//     MPU 9600 Varibles
//------------------------------

#define MPU9250_I2C_address 0x68                                        // I2C address for MPU9250 
#define MPU9250_I2C_master_enable 0x6A                                  // USER_CTRL[5] = I2C_MST_EN
#define MPU9250_Interface_bypass_mux_enable 0x37                        // INT_PIN_CFG[1]= BYPASS_EN

#define Frequency 125                                                   // 8mS sample interval 
#define Sensitivity 65.5                                                // Gyro sensitivity (see data sheet)

#define Sensor_to_deg 1/(Sensitivity*Frequency)                         // Convert sensor reading to degrees
#define Sensor_to_rad Sensor_to_deg*DEG_TO_RAD                          // Convert sensor reading to radians

#define Loop_time 1000000/Frequency                                     // Loop time (uS)
long    Loop_start;                                                     // Loop start time (uS)

int     Gyro_x,     Gyro_y,     Gyro_z;
long    Gyro_x_cal, Gyro_y_cal, Gyro_z_cal;
float   Gyro_pitch, Gyro_roll, Gyro_yaw;
float   Gyro_pitch_output, Gyro_roll_output;

// ----- Accelerometer
long    Accel_x,      Accel_y,      Accel_z,    Accel_total_vector;
float   Accel_pitch,  Accel_roll;

// ----- Magnetometer
#define AK8963_I2C_address 0x0C                                             // I2C address for AK8963
#define AK8963_cntrl_reg_1 0x0A                                             // CNTL[4]=#bits, CNTL[3:0]=mode
#define AK8963_status_reg_1 0x02                                            // ST1[0]=data ready
#define AK8963_data_ready_mask 0b00000001                                   // Data ready mask
#define AK8963_overflow_mask 0b00001000                                     // Magnetic sensor overflow mask
#define AK8963_data 0x03                                                    // Start address of XYZ data                                                                
#define AK8963_fuse_ROM 0x10                                                // X,Y,Z fuse ROM

// ----- Compass heading
/*
  The magnetic declination for Lower Hutt, Woodstown -11 degrees
  Obtain your magnetic declination from http://www.magnetic-declination.com/
  Uncomment the declination code within the main loop() if you want True North.
*/
float   Declination = -11;                                             //  Degrees ... replace this declination with yours
int     Heading;

int     Mag_x,                Mag_y,                Mag_z;                  // Raw magnetometer readings
float   Mag_x_dampened,       Mag_y_dampened,       Mag_z_dampened;
float   Mag_x_hor, Mag_y_hor;
float   Mag_pitch, Mag_roll;

// ----- Record compass offsets, scale factors, & ASA values
/*
   These values seldom change ... an occasional check is sufficient
   (1) Open your Arduino "Serial Monitor
   (2) Set "Record_data=true;" then upload & run program.
   (3) Replace the values below with the values that appear on the Serial Monitor.
   (4) Set "Record_data = false;" then upload & rerun program.
*/

int     Mag_x_offset = -8,      Mag_y_offset = 262,     Mag_z_offset = -107;   // Hard-iron offsets
float   Mag_x_scale = 1.01,     Mag_y_scale = 1.00,     Mag_z_scale = 0.99;    // Soft-iron scale factors
float   ASAX = 1.20,            ASAY = 1.21,            ASAZ = 1.16;           // (A)sahi (S)ensitivity (A)djustment fuse ROM values.


// ----- Flags
bool Gyro_synchronised = false;
bool Flag = false;

// ----- Debug
#define Switch A0                       // Connect an SPST switch between A0 and GND to enable/disable tilt stabilazation
long Loop_start_time;
long Debug_start_time;

//----------------------------
//       Radio Varibles
//----------------------------

RF24 radio(7, 8); // CE, CSN for mega
RF24Network network(radio);      // Include the radio in the network
const uint16_t master = 00;   // Address of this node in Octal format ( 04,031, etc)
const uint16_t rightHand01 = 01;      // Address of the other node in Octal format
const uint16_t leftHand012 = 02;
const uint16_t rightFoot04 = 03;      // 0X for child, 0XX for grand child, use only child
const uint16_t leftFoot022 = 04;
int message = 1;

//-----------------------------
//     Vibrator Varibles
//-----------------------------

unsigned long timeAftRH = 0;
unsigned long timeAftLH = 0;
unsigned long timeAftRF = 0;
unsigned long timeAftLF = 0;
int vpRH[] = {2, 3, 4}; //Pwm pins: probly not nessary btw
int vpLH[] = {5, 6, 9};
int vpRF[] = {11, 10, 13};
int vpLF[] = {44, 45, 46};
int pinCount = 3;
int vibAngleRH, vibAngleLH, vibAngleRF, vibAngleLF;
//float vibAngleRawRH, vibAngleRawLH, vibAngleRawRF, vibAngleRawLF;
int vibStateRH = LOW;
int vibStateLH = LOW;
int vibStateRF = LOW;
int vibStateLF = LOW;
int posRH, posLH, posLF, posRF;
int minVal;
int midVal;
int maxVal;


// datapack unload
struct DataPack {
  int distance;
  int magValIn;
};



//-------------------------------------------------------------------------------------------------------------------
//                                               SETUP
//-------------------------------------------------------------------------------------------------------------------


void setup() {

  Serial.begin(9600);
  Serial.println("\n              ____         _____                _____ ");
  Serial.println(" \\        /  |      |     |_____       \\    /        |");
  Serial.println("  \\  /\\  /   |---   |           |       \\  /    -----|");
  Serial.println("   \\/  \\/    |____  |____  _____|        \\/     _____|");
  
  Serial.println("\n\n\nInitializing receiver...");
  

  // ----- Provision to disable tilt stabilization
  /*
     Connect a jumper wire between A0 and GRN to disable the "tilt stabilazation"
  */
  pinMode(Switch, INPUT_PULLUP);                        // Short A0 to GND to disable tilt stabilization

  // ----- Configure the magnetometer
  configure_magnetometer();

  // ----- Calibrate the magnetometer
  /*
     Calibrate only needs to be done occasionally.
     Enter the magnetometer values into the "header"
     then set "Record_data = false".
  */
  if (Record_data == true)
  {
    calibrate_magnetometer();
  }

  // ----- Configure the gyro & magnetometer
  
  

  

  //---------------------------------
  //         Radio Setup
  //---------------------------------
  radio.begin();
  network.begin(90, master);  //(channel, node address)
  radio.setDataRate(RF24_2MBPS);

  //---------------------------------
  //        Vibrator Setup
  //---------------------------------

  pinMode(8, OUTPUT);

  //--- for loops to inizlise each vib pin
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    pinMode(vpRH[thisPin], OUTPUT);
  }
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    pinMode(vpLH[thisPin], OUTPUT);
  }
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    pinMode(vpRF[thisPin], OUTPUT);
  }
  for (int thisPin = 0; thisPin < pinCount; thisPin++) {
    pinMode(vpLF[thisPin], OUTPUT);
  }
  
  Debug_start_time = micros();                          // Controls data display rate to Serial Monitor
  Loop_start_time = micros();                           // Controls the Gyro refresh rate
}

// ---------------------------------------------------------------------------------------------------------------
//                Main Loop
// ---------------------------------------------------------------------------------------------------------------


void loop() {
  network.update(); //update network
  lis3mdl.read();      // get X Y and Z data at once

  //-----------------------------------------------------------------------
  //                                 Mag loop
  //-----------------------------------------------------------------------


  
  ////////////////////////////////////////////
  //        MAGNETOMETER CALCULATIONS       //
  ////////////////////////////////////////////
 
    // ---- Test equations
    Mag_x_hor = Mag_x;
    Mag_y_hor = Mag_y;
 

  // ----- Dampen any data fluctuations
  Mag_x_dampened = Mag_x_dampened * 0.9 + Mag_x_hor * 0.1;
  Mag_y_dampened = Mag_y_dampened * 0.9 + Mag_y_hor * 0.1;

  // ----- Calculate the heading
  Heading = atan2(Mag_x_dampened, Mag_y_dampened) * RAD_TO_DEG;  // Magnetic North

  /*
     By convention, declination is positive when magnetic north
     is east of true north, and negative when it is to the west.
  */

  Heading += Declination;                                   // Geographic North
  if (Heading > 360.0) Heading -= 360.0;
  if (Heading < 0.0) Heading += 360.0;

  // ----- Allow for under/overflow
  if (Heading < 0) Heading += 360;
  if (Heading >= 360) Heading -= 360;


  // ----- Loop control
  /*
     Adjust the loop count for a yaw reading of 360 degrees
     when the MPU-9250 is rotated exactly 360 degrees.
     (Compensates for any 16 MHz Xtal oscillator error)
  */
  while ((micros() - Loop_start_time) < 8000);
  Loop_start_time = micros();

  //end of mag calculation


  //---------------------------------------------
  //        Radio read & write
  //---------------------------------------------

  


  DataPack dataPack; // declare data pack
  RF24NetworkHeader header;
  RF24NetworkHeader ping(rightHand01);

  network.write(ping, &message, sizeof(message)); //send to right hand

  //--- make radio connection
  if (network.available()) {
    //while (network.available()) { // if radio is found

      network.read(header, &dataPack, sizeof(dataPack));   //read radios




      //--- vibration calc
      unsigned long timeElapsedRH = millis();     // start timers
      unsigned long timeElapsedLH = millis();
      unsigned long timeElapsedRF = millis();
      unsigned long timeElapsedLF = millis();

      //-----------------------------------------------------------------------
      //                     Sensor Location Calculation
      //-----------------------------------------------------------------------


      


      //                    *******************************************
      //                    *               Right Hand                *
      //                    *******************************************

      if (header.from_node == 01) { //rh


        posRH = dataPack.magValIn / 30; //split up in to 12 section

        /*                                    N
                                            12| 1
                                         11   |    2
                                    W __10____|_____3__ E
                                        9     |     4
                                         8    |    5
                                            7 | 6
                                              S
             At any givin monent each sensor could be at any one of these 12 points.
             However we want only the infornt of the user to be active
        */

        minVal = Heading / 30;  //create min val
        maxVal = minVal + 2; //create max val
        midVal = minVal + 1; // create mid val
        
        if (maxVal > 11) { maxVal -= 11; midVal -= 1; }  //make the circle go around
        if (minVal < 0) { minVal += 11; midVal += 1; } 


        if (useDirectionSensors == true) {
          if (posRH <= minVal && posRH < (minVal + 1)) { vibAngleRH = 0; }
          else if (midVal == posRH) { vibAngleRH = 1; }  
          else if (posRH >= maxVal) { vibAngleRH = 2; }

          if (vibePulse == true) { // if pulsing and tracking is turned on
            if (dataPack.distance < 100) { // distance limit, increas to increase limit
              if (timeElapsedRH - timeAftRH >= dataPack.distance * 5) { //create pulsing
                timeAftRH = timeElapsedRH;
                if (vibStateRH == LOW) {
                  vibStateRH = HIGH;
                } else {
                  vibStateRH = LOW;
                } // to else
              } // to if time of vib
              digitalWrite(vpRH[vibAngleRH], vibStateRH);  // output vib
            } //if range limit
            else  {
              digitalWrite(vpRH[0], LOW); //make sure off
              digitalWrite(vpRH[1], LOW);
              digitalWrite(vpRH[2], LOW);
            }  // to else
          } else { //vibe pulse
            if (dataPack.distance < 100) {
              analogWrite(vpRH[vibAngleRH], ((100 - dataPack.distance) * 2.55)); 
            }
          }


        } else {

          //if direction is turned off this will just vibrate at one point

          if (vibePulse == true) { // if pulsing is turned on but not tracking
            if (dataPack.distance < 100) { // distance limit, increas to increase limit
              if (timeElapsedRH - timeAftRH >= dataPack.distance * 5) { //create pulsing
                timeAftRH = timeElapsedRH;
                if (vibStateRH == LOW) {
                  vibStateRH = HIGH;
                } else {
                  vibStateRH = LOW;
                } // to else
              } // to if time of vib
              digitalWrite(vpRH[0], vibStateRH);  // output vib
            } //if range limit
            else  {
              digitalWrite(vpRH[0], LOW); //make sure off
              digitalWrite(vpRH[1], LOW);
              digitalWrite(vpRH[2], LOW);
            }  // to else
          } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpRH[vibAngleRH], ((100 - dataPack.distance) * 2.55)); 
            }
          }
          
        }



      }

      //                    *******************************************
      //                    *               Left Hand                 *
      //                    *******************************************


      if (header.from_node == 02) {

       
      }


      //                    *******************************************
      //                    *               Right Foot                *
      //                    *******************************************


      if (header.from_node == 03) {

       
      }


      //                    *******************************************
      //                    *               Left Foot                 *
      //                    *******************************************



      if (header.from_node == 04) {

        
      }

   // } // to while loop
  } else {
    digitalWrite(vpRH[0], LOW); //make sure off
    digitalWrite(vpRH[1], LOW);
    digitalWrite(vpRH[2], LOW);
    digitalWrite(vpLH[0], LOW); //make sure off
    digitalWrite(vpLH[1], LOW);
    digitalWrite(vpLH[2], LOW);
    digitalWrite(vpRF[0], LOW); //make sure off
    digitalWrite(vpRF[1], LOW);
    digitalWrite(vpRF[2], LOW);
    digitalWrite(vpLF[0], LOW); //make sure off
    digitalWrite(vpLF[1], LOW);
    digitalWrite(vpLF[2], LOW);
  } // normally off

  //----------------------------------------------------------------
  //                 Serial write data
  //----------------------------------------------------------------

  Serial.print("Dist (cm): ");
  Serial.print(dataPack.distance);
  //    Serial.print(" Header data: ");
  //   Serial.print(header.from_node);
  Serial.print("\t BM: ");
  Serial.print(Heading);
  Serial.print(" ");
  Serial.print(vibAngleRH); 
  Serial.print("\t Angle: ");
  Serial.print(dataPack.magValIn);
  Serial.print(" ");
  Serial.print(posRH);
  Serial.print("\t Min/Max Val: ");
  Serial.print(minVal);
  Serial.print(" ");
  Serial.print(midVal);
  Serial.print(" ");
  Serial.println(maxVal);
  //   Serial.print(" Freq (Hz): ");
  //   Serial.println(1/(dataPack.distance*.01)); //frequency calc

  delay(10);
}    // end of loop



//-------------------------------------------------------------------------
//                       Config functions
//-------------------------------------------------------------------------







// ----------------------------
//  Configure magnetometer
// ----------------------------
void configure_magnetometer()
{
                                    // Wait for the data
  ASAX = (lis3mdl.x - 128) * 0.5 / 128 + 1;                       // Adjust data
  ASAY = (lis3mdl.y - 128) * 0.5 / 128 + 1;
  ASAZ = (lis3mdl.z - 128) * 0.5 / 128 + 1;

 
  delay(100);                                                       // Wait for mode change
}

// -------------------------------
//  Calibrate magnetometer
// -------------------------------
void calibrate_magnetometer()
{
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;                                               // ST2 status register

  int mag_x_min =  32767;                                         // Raw data extremes
  int mag_y_min =  32767;
  int mag_z_min =  32767;
  int mag_x_max = -32768;
  int mag_y_max = -32768;
  int mag_z_max = -32768;

  float chord_x,  chord_y,  chord_z;                              // Used for calculating scale factors
  float chord_average;


  Serial.print("Rotate Compass");                                    // Print text to screen


  // ----- Record min/max XYZ compass readings
  for (int counter = 0; counter < 16000 ; counter ++)             // Run this code 16000 times
  {
    Loop_start = micros();                                        // Start loop timer

    // ----- Point to status register 1
               // Wait for the data
      mag_x = (lis3mdl.x | lis3mdl.x << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
      mag_y = (lis3mdl.y | lis3mdl.y << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
      mag_z = (lis3mdl.z | lis3mdl.z << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
                                      // Read status and signal data read

      // ----- Validate data
     
    
    delay(4);                                                     // Time interval between magnetometer readings
  }

  // ----- Calculate hard-iron offsets
  Mag_x_offset = (mag_x_max + mag_x_min) / 2;                     // Get average magnetic bias in counts
  Mag_y_offset = (mag_y_max + mag_y_min) / 2;
  Mag_z_offset = (mag_z_max + mag_z_min) / 2;

  // ----- Calculate soft-iron scale factors
  chord_x = ((float)(mag_x_max - mag_x_min)) / 2;                 // Get average max chord length in counts
  chord_y = ((float)(mag_y_max - mag_y_min)) / 2;
  chord_z = ((float)(mag_z_max - mag_z_min)) / 2;

  chord_average = (chord_x + chord_y + chord_z) / 3;              // Calculate average chord length

  Mag_x_scale = chord_average / chord_x;                          // Calculate X scale factor
  Mag_y_scale = chord_average / chord_y;                          // Calculate Y scale factor
  Mag_z_scale = chord_average / chord_z;                          // Calculate Z scale factor

  // ----- Record magnetometer offsets
  /*
     When active this feature sends the magnetometer data
     to the Serial Monitor then halts the program.
  */
  if (Record_data == true)
  {
    // ----- Display data extremes
    Serial.print("XYZ Max/Min: ");
    Serial.print(mag_x_min); Serial.print("\t");
    Serial.print(mag_x_max); Serial.print("\t");
    Serial.print(mag_y_min); Serial.print("\t");
    Serial.print(mag_y_max); Serial.print("\t");
    Serial.print(mag_z_min); Serial.print("\t");
    Serial.println(mag_z_max);
    Serial.println("");

    // ----- Display hard-iron offsets
    Serial.print("Hard-iron: ");
    Serial.print(Mag_x_offset); Serial.print("\t");
    Serial.print(Mag_y_offset); Serial.print("\t");
    Serial.println(Mag_z_offset);
    Serial.println("");

    // ----- Display soft-iron scale factors
    Serial.print("Soft-iron: ");
    Serial.print(Mag_x_scale); Serial.print("\t");
    Serial.print(Mag_y_scale); Serial.print("\t");
    Serial.println(Mag_z_scale);
    Serial.println("");

    // ----- Display fuse ROM values
    Serial.print("ASA: ");
    Serial.print(ASAX); Serial.print("\t");
    Serial.print(ASAY); Serial.print("\t");
    Serial.println(ASAZ);

    // ----- Halt program
    while (true);                                       // Wheelspin ... program halt
  }
}

// -------------------------------
//  Read magnetometer
// -------------------------------
void read_magnetometer()
{
  // ----- Locals
  int mag_x, mag_y, mag_z;
  int status_reg_2;

  // ----- Point to status register 1
  
    // ----- Read data from each axis (LSB,MSB)
               // Request 7 data bytes
                         // Wait for the data
    mag_x = (lis3mdl.x | lis3mdl.x << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
    mag_y = (lis3mdl.y | lis3mdl.y << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
    mag_z = (lis3mdl.z | lis3mdl.z << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
    status_reg_2 = Wire.read();                                 // Read status and signal data read

    // ----- Validate data
    
  
}
