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

                                                       bool useDirectionSensors = false;
//                             Turn this to false to turn of direction control, aka, it will vibrate in one spot.

                                                         bool Record_data = false;
//                           Turn this to ture to start the mag claibration, only needs to be done once in a while.

                                                           bool vibePulse = false;
//                 If true the vibrators will pulse faster the closer the object, if false intesity of the vibrator will increase

                                                         bool gyroStabization = true;
//                              if false it will turn off gyro stabization of the magentometer output



#include <RF24Network.h>
#include <RF24Network_config.h>
#include <Sync.h>
#include <SPI.h>
#include <RF24.h>
#include <Wire.h>

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
int midVal, midValL;
int maxVal, maxValL;


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
  SPI.begin();
  Wire.begin();                                         //Start I2C as master
  Wire.setClock(400000);

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
  config_gyro();

  calibrate_gyro();

  

  

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

  //-----------------------------------------------------------------------
  //                                 Mag loop
  //-----------------------------------------------------------------------
  if (useDirectionSensors == true) {

  ////////////////////////////////////////////
  //        PITCH & ROLL CALCULATIONS       //
  ////////////////////////////////////////////

  /*
     --------------------
     MPU-9250 Orientation
     --------------------
     Component side up
     X-axis facing forward
  */

  // ----- read the raw accelerometer and gyro data
  read_mpu_6050_data();                                             // Read the raw acc and gyro data from the MPU-6050

  // ----- Adjust for offsets
  Gyro_x -= Gyro_x_cal;                                             // Subtract the offset from the raw gyro_x value
  Gyro_y -= Gyro_y_cal;                                             // Subtract the offset from the raw gyro_y value
  Gyro_z -= Gyro_z_cal;                                             // Subtract the offset from the raw gyro_z value

  // ----- Calculate travelled angles
  /*
    ---------------------------
    Adjust Gyro_xyz signs for:
    ---------------------------
    Pitch (Nose - up) = +ve reading
    Roll (Right - wing down) = +ve reading
    Yaw (Clock - wise rotation)  = +ve reading
  */
  Gyro_pitch += -Gyro_y * Sensor_to_deg;                            // Integrate the raw Gyro_y readings
  Gyro_roll += Gyro_x * Sensor_to_deg;                              // Integrate the raw Gyro_x readings
  Gyro_yaw += -Gyro_z * Sensor_to_deg;                              // Integrate the raw Gyro_x readings

  // ----- Compensate pitch and roll for gyro yaw
  Gyro_pitch += Gyro_roll * sin(Gyro_z * Sensor_to_rad);            // Transfer the roll angle to the pitch angle if the Z-axis has yawed
  Gyro_roll -= Gyro_pitch * sin(Gyro_z * Sensor_to_rad);            // Transfer the pitch angle to the roll angle if the Z-axis has yawed

  // ----- Accelerometer angle calculations
  Accel_total_vector = sqrt((Accel_x * Accel_x) + (Accel_y * Accel_y) + (Accel_z * Accel_z));   // Calculate the total (3D) vector
  Accel_pitch = asin((float)Accel_x / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the pitch angle
  Accel_roll = asin((float)Accel_y / Accel_total_vector) * RAD_TO_DEG;                         //Calculate the roll angle

  // ----- Zero any residual accelerometer readings
  /*
     Place the accelerometer on a level surface
     Adjust the following two values until the pitch and roll readings are zero
  */
  Accel_pitch -= -0.2f;                                             //Accelerometer calibration value for pitch
  Accel_roll -= 1.1f;                                               //Accelerometer calibration value for roll

  // ----- Correct for any gyro drift
  if (Gyro_synchronised)
  {
    // ----- Gyro & accel have been synchronised
    Gyro_pitch = Gyro_pitch * 0.9996 + Accel_pitch * 0.0004;        //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    Gyro_roll = Gyro_roll * 0.9996 + Accel_roll * 0.0004;           //Correct the drift of the gyro roll angle with the accelerometer roll angle
  }
  else
  {
    // ----- Synchronise gyro & accel
    Gyro_pitch = Accel_pitch;                                       //Set the gyro pitch angle equal to the accelerometer pitch angle
    Gyro_roll = Accel_roll;                                         //Set the gyro roll angle equal to the accelerometer roll angle
    Gyro_synchronised = true;                                             //Set the IMU started flag
  }

  // ----- Dampen the pitch and roll angles
  Gyro_pitch_output = Gyro_pitch_output * 0.9 + Gyro_pitch * 0.1;   //Take 90% of the output pitch value and add 10% of the raw pitch value
  Gyro_roll_output = Gyro_roll_output * 0.9 + Gyro_roll * 0.1;      //Take 90% of the output roll value and add 10% of the raw roll value

  ////////////////////////////////////////////
  //        MAGNETOMETER CALCULATIONS       //
  ////////////////////////////////////////////
  /*
     --------------------------------
     Instructions for first time use
     --------------------------------
     Calibrate the compass for Hard-iron and Soft-iron
     distortion by temporarily setting the header to read
     bool    Record_data = true;

     Turn on your Serial Monitor before uploading the code.

     Slowly tumble the compass in all directions until a
     set of readings appears in the Serial Monitor.

     Copy these values into the appropriate header locations.

     Edit the header to read
     bool    Record_data = false;

     Upload the above code changes to your Arduino.

     This step only needs to be done occasionally as the
     values are reasonably stable.
  */

  // ----- Read the magnetometer
  read_magnetometer();

  // ----- Fix the pitch, roll, & signs
  /*
     MPU-9250 gyro and AK8963 magnetometer XY axes are orientated 90 degrees to each other
     which means that Mag_pitch equates to the Gyro_roll and Mag_roll equates to the Gryro_pitch

     The MPU-9520 and AK8963 Z axes point in opposite directions
     which means that the sign for Mag_pitch must be negative to compensate.
  */
  Mag_pitch = -Gyro_roll_output * DEG_TO_RAD;
  Mag_roll = Gyro_pitch_output * DEG_TO_RAD;

  // ----- Apply the standard tilt formulas
  Mag_x_hor = Mag_x * cos(Mag_pitch) + Mag_y * sin(Mag_roll) * sin(Mag_pitch) - Mag_z * cos(Mag_roll) * sin(Mag_pitch);
  Mag_y_hor = Mag_y * cos(Mag_roll) + Mag_z * sin(Mag_roll);

  // ----- Disable tilt stabization if switch closed
  if (gyroStabization == false)
  {
    // ---- Test equations
    Mag_x_hor = Mag_x;
    Mag_y_hor = Mag_y;
  }

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
}

  //---------------------------------------------
  //        Radio read & write
  //---------------------------------------------

  


  DataPack dataPack; // declare data pack
  RF24NetworkHeader header;
  RF24NetworkHeader ping(rightHand01);
  RF24NetworkHeader ping2(leftHand012);

  struct DataOut {   //declare output data
    bool message = true;
    int headingOut = Heading;
  };

  DataOut dataOut;

 
  

  //--- make radio connection
  if (network.available()) {
    //while (network.available()) { // if radio is found

      network.read(header, &dataPack, sizeof(dataPack));   //read radios




      //--- vibration calc
     
     
      
      unsigned long timeElapsedLF = millis();

      //-----------------------------------------------------------------------
      //                     Sensor Location Calculation
      //-----------------------------------------------------------------------


      


      //                    *******************************************
      //                    *               Right Hand                *
      //                    *******************************************
       
      if (header.from_node == 01) { //rh
        unsigned long timeElapsedRH;
        //network.write(ping, &dataOut, sizeof(dataOut)); //send to right hand


        posRH = dataPack.magValIn / 30; //split up in to 12 section

        /*                                    N
                                            11| 0
                                         10   |    1
                                    W __9_____|_____2__ E
                                        8     |     3
                                         7    |    4
                                            6 | 5
                                              S
             At any givin monent each sensor could be at any one of these 12 points.
             However we want only the infornt of the user to be active
        */
        

        minVal = Heading / 30;  //create min val
        maxVal = minVal + 2; //create max val
        midVal = minVal + 1; // create mid val
        
        if (maxVal > 11) { maxVal -= 12; }  //make the circle go around
        if (midVal > 11) { midVal -= 12; }
        if (minVal < 0) { minVal += 12;  } 
        if (midVal < 0) { midVal += 12; }

             // start timers
        if (useDirectionSensors == true) {
          if (posRH <= minVal && posRH < (minVal + 1)) { vibAngleRH = 0; }
          else if (midVal == posRH) { vibAngleRH = 1; }  
          else if (posRH >= maxVal) { vibAngleRH = 2; }

          if (vibePulse == true) { // if pulsing and tracking is turned on
            timeElapsedRH = millis();
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
            } else  {
              digitalWrite(vpRH[0], LOW); //make sure off
              digitalWrite(vpRH[1], LOW);
              digitalWrite(vpRH[2], LOW);
            }  // to else
          }


        } else {

          //if direction is turned off this will just vibrate at one point
          timeElapsedRH = millis();
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
        
            }  // to else
          } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpRH[0], ((100 - dataPack.distance) * 2.55)); 
            }else  {
              digitalWrite(vpRH[0], LOW); //make sure off
              
            }  // to else
          }
          
        }



      }else  {
              digitalWrite(vpRH[0], LOW); //make sure off
              digitalWrite(vpRH[1], LOW);
              digitalWrite(vpRH[2], LOW);
            }  // to else

      //                    *******************************************
      //                    *               Left Hand                 *
      //                    *******************************************

      
      if (header.from_node == 02) {
        unsigned long timeElapsedLH = millis();
        //network.write(ping2, &dataOut, sizeof(dataOut)); //send to left hand
        posLH = dataPack.magValIn / 30; //split up in to 12 section

        /*                                    N
                                            11| 0
                                         10   |    1
                                    W __9_____|_____2__ E
                                        8     |     3
                                         7    |    4
                                            6 | 5
                                              S
             At any givin monent each sensor could be at any one of these 12 points.
             However we want only the infornt of the user to be active
        */
        

        minVal = Heading / 30;  //create min val
        maxValL = minVal - 2; //create max val
        midValL = minVal - 1; // create mid val
        
        if (maxValL > 11) { maxValL -= 12; }  //make the circle go around
        if (midValL > 11) { midValL -= 12; }
        if (minVal < 0) { minVal += 12;  } 
        if (midValL < 0) { midValL += 12; }


        if (useDirectionSensors == true) {
          if (posLH <= minVal && posLH < (minVal + 1)) { vibAngleLH = 0; }
          else if (midValL == posLH) { vibAngleLH = 1; }  
          else if (posLH >= maxValL) { vibAngleLH = 2; }

          if (vibePulse == true) { // if pulsing and tracking is turned on
            if (dataPack.distance < 100) { // distance limit, increas to increase limit
              if (timeElapsedLH - timeAftLH >= dataPack.distance * 5) { //create pulsing
                timeAftLH = timeElapsedLH;
                if (vibStateLH == LOW) {
                  vibStateLH = HIGH;
                } else {
                  vibStateLH = LOW;
                } // to else
              } // to if time of vib
              digitalWrite(vpLH[vibAngleLH], vibStateLH);  // output vib
            } //if range limit
            else  {
              digitalWrite(vpLH[0], LOW); //make sure off
              digitalWrite(vpLH[1], LOW);
              digitalWrite(vpLH[2], LOW);
            }  // to else
          } else { //vibe pulse
            if (dataPack.distance < 100) {
              analogWrite(vpLH[vibAngleLH], ((100 - dataPack.distance) * 2.55)); 
            } else  {
              digitalWrite(vpLH[0], LOW); //make sure off
              digitalWrite(vpLH[1], LOW);
              digitalWrite(vpLH[2], LOW);
            }  // to else
          }


        } else {

          //if direction is turned off this will just vibrate at one point

          if (vibePulse == true) { // if pulsing is turned on but not tracking
            if (dataPack.distance < 100) { // distance limit, increas to increase limit
              if (timeElapsedLH - timeAftLH >= dataPack.distance * 5) { //create pulsing
                timeAftLH = timeElapsedLH;
                if (vibStateLH == LOW) {
                  vibStateLH = HIGH;
                } else {
                  vibStateLH = LOW;
                } // to else
              } // to if time of vib
              digitalWrite(vpLH[0], vibStateLH);  // output vib
            } //if range limit
            else  {
              digitalWrite(vpLH[0], LOW); //make sure off
            
            }  // to else
          } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpLH[0], ((100 - dataPack.distance) * 2.55)); 
            }  else  {
              digitalWrite(vpLH[0], LOW); //make sure off
             
            }  // to else
          }
          
        }
       
      }  else  {
              digitalWrite(vpLH[0], LOW); //make sure off
              digitalWrite(vpLH[1], LOW);
              digitalWrite(vpLH[2], LOW);
            }  // to else


      //                    *******************************************
      //                    *               Right Foot                *
      //                    *******************************************


      if (header.from_node == 03) {
          unsigned long timeElapsedRF = millis();
      
          //In this version the foot sensors are none directional and theyt do not have magnetometers.
          vibAngleRF = 0;

          if (vibePulse == true) { // if pulsing is turned on but not tracking
            if (dataPack.distance < 100) { // distance limit, increas to increase limit
              if (timeElapsedRF - timeAftRF >= dataPack.distance * 5) { //create pulsing
                timeAftRF = timeElapsedRF;
                if (vibStateRF == LOW) {
                  vibStateRF = HIGH;
                } else {
                  vibStateRF = LOW;
                } // to else
              } // to if time of vib
              digitalWrite(vpRF[2], vibStateRF);  // output vib
            } //if range limit
            else  {
              digitalWrite(vpRF[0], LOW); //make sure off
              digitalWrite(vpRF[1], LOW);
              digitalWrite(vpRF[2], LOW);
            }  // to else
          } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpRF[2], ((100 - dataPack.distance) * 2.55)); 
            } else {
              digitalWrite(vpRF[0], LOW); //make sure off
              digitalWrite(vpRF[1], LOW);
              digitalWrite(vpRF[2], LOW);
            }
          }
          
        
       
      }else {
              digitalWrite(vpRF[0], LOW); //make sure off
              digitalWrite(vpRF[1], LOW);
              digitalWrite(vpRF[2], LOW);
            }


      //                    *******************************************
      //                    *               Left Foot                 *
      //                    *******************************************



      if (header.from_node == 04) {
           unsigned long timeElapsedLF = millis();
      //In this version the foot sensors are none directional and theyt do not have magnetometers.
          vibAngleLF =0;

          if (vibePulse == true) { // if pulsing is turned on but not tracking
            if (dataPack.distance < 100) { // distance limit, increas to increase limit
              if (timeElapsedLF - timeAftLF >= dataPack.distance * 5) { //create pulsing
                timeAftLF = timeElapsedLF;
                if (vibStateLF == LOW) {
                  vibStateLF = HIGH;
                } else {
                  vibStateLF = LOW;
                } // to else
              } // to if time of vib
              digitalWrite(vpLF[2], vibStateLF);  // output vib
            } //if range limit
            else  {
              digitalWrite(vpLF[0], LOW); //make sure off
              digitalWrite(vpLF[1], LOW);
              digitalWrite(vpLF[2], LOW);
            }  // to else
          } else {// vibe pulse 
            if (dataPack.distance < 100) {
              analogWrite(vpLF[2], ((100 - dataPack.distance) * 2.55)); 
            } else {
              digitalWrite(vpLF[0], LOW); //make sure off
              digitalWrite(vpLF[1], LOW);
              digitalWrite(vpLF[2], LOW);
            }
          }
          
        
      } else {
              digitalWrite(vpLF[0], LOW); //make sure off
              digitalWrite(vpLF[1], LOW);
              digitalWrite(vpLF[2], LOW);
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
  /*
     The MPU-9250 contains an AK8963 magnetometer and an
     MPU-6050 gyro/accelerometer within the same package.

     To access the AK8963 magnetometer chip The MPU-9250 I2C bus
     must be changed to pass-though mode. To do this we must:
      - disable the MPU-9250 slave I2C and
      - enable the MPU-9250 interface bypass mux
  */
  // ----- Disable MPU9250 I2C master interface
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_I2C_master_enable);                            // Point USER_CTRL[5] = I2C_MST_EN
  Wire.write(0x00);                                                 // Disable the I2C master interface
  Wire.endTransmission();

  // ----- Enable MPU9250 interface bypass mux
  Wire.beginTransmission(MPU9250_I2C_address);                      // Open session with MPU9250
  Wire.write(MPU9250_Interface_bypass_mux_enable);                  // Point to INT_PIN_CFG[1] = BYPASS_EN
  Wire.write(0x02);                                                 // Enable the bypass mux
  Wire.endTransmission();

  // ----- Access AK8963 fuse ROM
  /* The factory sensitivity readings for the XYZ axes are stored in a fuse ROM.
     To access this data we must change the AK9863 operating mode.
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // CNTL[3:0] mode bits
  Wire.write(0b00011111);                                           // Output data=16-bits; Access fuse ROM
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  // ----- Get factory XYZ sensitivity adjustment values from fuse ROM
  /* There is a formula on page 53 of "MPU-9250, Register Map and Decriptions, Revision 1.4":
      Hadj = H*(((ASA-128)*0.5)/128)+1 where
      H    = measurement data output from data register
      ASA  = sensitivity adjustment value (from fuse ROM)
      Hadj = adjusted measurement data (after applying
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_fuse_ROM);                                      // Point to AK8963 fuse ROM
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 3);                          // Request 3 bytes of data
  while (Wire.available() < 3);                                     // Wait for the data
  ASAX = (Wire.read() - 128) * 0.5 / 128 + 1;                       // Adjust data
  ASAY = (Wire.read() - 128) * 0.5 / 128 + 1;
  ASAZ = (Wire.read() - 128) * 0.5 / 128 + 1;

  // ----- Power down AK8963 while the mode is changed
  /*
     This wasn't necessary for the first mode change as the chip was already powered down
  */
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00000000);                                           // Set mode to power down
  Wire.endTransmission();
  delay(100);                                                       // Wait for mode change

  // ----- Set output to mode 2 (16-bit, 100Hz continuous)
  Wire.beginTransmission(AK8963_I2C_address);                       // Open session with AK8963
  Wire.write(AK8963_cntrl_reg_1);                                   // Point to mode control register
  Wire.write(0b00010110);                                           // Output=16-bits; Measurements = 100Hz continuous
  Wire.endTransmission();
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
    Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
    Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
    Wire.endTransmission();
    Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
    while (Wire.available() < 1);                                 // Wait for the data
    if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
    {
      // ----- Read data from each axis (LSB,MSB)
      Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
      while (Wire.available() < 7);                               // Wait for the data
      mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
      mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
      mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
      status_reg_2 = Wire.read();                                 // Read status and signal data read

      // ----- Validate data
      if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
      {
        // ----- Find max/min xyz values
        mag_x_min = min(mag_x, mag_x_min);
        mag_x_max = max(mag_x, mag_x_max);
        mag_y_min = min(mag_y, mag_y_min);
        mag_y_max = max(mag_y, mag_y_max);
        mag_z_min = min(mag_z, mag_z_min);
        mag_z_max = max(mag_z, mag_z_max);
      }
    }
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
  Wire.beginTransmission(AK8963_I2C_address);                   // Open session with AK8963
  Wire.write(AK8963_status_reg_1);                              // Point to ST1[0] status bit
  Wire.endTransmission();
  Wire.requestFrom(AK8963_I2C_address, 1);                      // Request 1 data byte
  while (Wire.available() < 1);                                 // Wait for the data
  if (Wire.read() & AK8963_data_ready_mask)                     // Check data ready bit
  {
    // ----- Read data from each axis (LSB,MSB)
    Wire.requestFrom(AK8963_I2C_address, 7);                    // Request 7 data bytes
    while (Wire.available() < 7);                               // Wait for the data
    mag_x = (Wire.read() | Wire.read() << 8) * ASAX;            // Combine LSB,MSB X-axis, apply ASA corrections
    mag_y = (Wire.read() | Wire.read() << 8) * ASAY;            // Combine LSB,MSB Y-axis, apply ASA corrections
    mag_z = (Wire.read() | Wire.read() << 8) * ASAZ;            // Combine LSB,MSB Z-axis, apply ASA corrections
    status_reg_2 = Wire.read();                                 // Read status and signal data read

    // ----- Validate data
    if (!(status_reg_2 & AK8963_overflow_mask))                 // Check HOFL flag in ST2[3]
    {
      Mag_x = (mag_x - Mag_x_offset) * Mag_x_scale;
      Mag_y = (mag_y - Mag_y_offset) * Mag_y_scale;
      Mag_z = (mag_z - Mag_z_offset) * Mag_z_scale;
    }
  }
}
// -----------------------------------
//  Configure the gyro & accelerometer
// -----------------------------------
void config_gyro()
{
  // ----- Activate the MPU-6050
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x6B);                                     //Point to power management register
  Wire.write(0x00);                                     //Use internal 20MHz clock
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1C);                                     //Point to accelerometer configuration reg
  Wire.write(0x10);                                     //Select +/-8g full-scale
  Wire.endTransmission();                               //End the transmission

  // ----- Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                         //Open session with the MPU-6050
  Wire.write(0x1B);                                     //Point to gyroscope configuration
  Wire.write(0x08);                                     //Select 500dps full-scale
  Wire.endTransmission();                               //End the transmission
}


// -----------------------------------
//  Calibrate gyro
// -----------------------------------
void calibrate_gyro()
{



  // ----- Calibrate gyro
  for (int counter = 0; counter < 2000 ; counter ++)    //Run this code 2000 times
  {
    Loop_start = micros();
    read_mpu_6050_data();                               //Read the raw acc and gyro data from the MPU-6050
    Gyro_x_cal += Gyro_x;                               //Add the gyro x-axis offset to the gyro_x_cal variable
    Gyro_y_cal += Gyro_y;                               //Add the gyro y-axis offset to the gyro_y_cal variable
    Gyro_z_cal += Gyro_z;                               //Add the gyro z-axis offset to the gyro_z_cal variable
    while (micros() - Loop_start < Loop_time);           // Wait until "Loop_time" microseconds have elapsed
  }
  Gyro_x_cal /= 2000;                                   //Divide the gyro_x_cal variable by 2000 to get the average offset
  Gyro_y_cal /= 2000;                                   //Divide the gyro_y_cal variable by 2000 to get the average offset
  Gyro_z_cal /= 2000;                                   //Divide the gyro_z_cal variable by 2000 to get the average offset

}

// --------------------
//  Read MPU 6050 data
// --------------------
void read_mpu_6050_data()
{
  /*
    Subroutine for reading the raw gyro and accelerometer data
  */

  // ----- Locals
  int     temperature;                                  // Needed when reading the MPU-6050 data ... not used

  // ----- Point to data
  Wire.beginTransmission(0x68);                         // Start communicating with the MPU-6050
  Wire.write(0x3B);                                     // Point to start of data
  Wire.endTransmission();                               // End the transmission

  // ----- Read the data
  Wire.requestFrom(0x68, 14);                           // Request 14 bytes from the MPU-6050
  while (Wire.available() < 14);                        // Wait until all the bytes are received
  Accel_x = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_x variable
  Accel_y = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_y variable
  Accel_z = Wire.read() << 8 | Wire.read();             // Combine MSB,LSB Accel_z variable
  temperature = Wire.read() << 8 | Wire.read();         // Combine MSB,LSB temperature variable
  Gyro_x = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_y = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
  Gyro_z = Wire.read() << 8 | Wire.read();              // Combine MSB,LSB Gyro_x variable
}


//----------------------------------
//        Postion fuctions
//----------------------------------
