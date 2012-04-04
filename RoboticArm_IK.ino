/* 
   Inverse kinematic control for AL5D arm using standard Servo library
   Modified from Oleg Mazurov's code here:
   http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino
   
   Last modified: 04-03-2012
*/

#include <Servo.h>

// Declare servos
Servo BAS_SERVO;  // Base servo HS-475HB (900 - 2100)
Servo SHL_SERVO;  // Shoulder servo HS-805BB (900 - 2100)
Servo ELB_SERVO;  // Elbow servo HS-755HB (900 - 2100)
Servo WRI_SERVO;  // Wrist tilt servo HS-645MG (900 - 2100)
Servo WRO_SERVO;  // Wrist rotate servo HS-225MG (900 - 2100)
Servo GRI_SERVO;  // Grip servo HS-422 (900 - 2100)
 
// Arm dimensions(mm)
#define BASE_HGT 67.31      //base hight 2.65"
#define HUMERUS 146.05      //shoulder-to-elbow "bone" 5.75"
#define ULNA 187.325        //elbow-to-wrist "bone" 7.375"
//#define GRIPPER 0      //gripper (wrist rotate mechanism) length 3.94"

// Define right angle errors (90 degree value (us))
//#define BAS_SERVO_ERROR
#define SHL_SERVO_ERROR 1520
#define ELB_SERVO_ERROR 1490
#define WRI_SERVO_ERROR 1550
//#define WRO_SERVO_ERROR
//#define GRI_SERVO_ERROR
 
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion
 
/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

float robotX = 0, robotY = 0, robotZ = 0;
 
void setup(){
  // Attach servo and set limits
  BAS_SERVO.attach(3, 900, 2100);
  SHL_SERVO.attach(5, 900, 2100);
  ELB_SERVO.attach(6, 900, 2100);
  WRI_SERVO.attach(9, 900, 2100);
  WRO_SERVO.attach(10, 900, 2100);
  GRI_SERVO.attach(11, 900, 2100);
  /**/
  set_arm(0,250,150,0);
  Serial.begin(19200);
  //Serial.println("Start");
  delay(500);
}
 
void loop(){
  if(Serial.available() > 3){
    if(Serial.read() == 127 && Serial.read() == 0){
      robotX = Serial.read();
      robotZ = Serial.read();
      set_arm(robotX, 200, robotZ, 0);
    }
  }
  
  //zero_x();
  //line();
  //circle();
 }
 
/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
void set_arm(float x, float y, float z, float grip_angle_d)
{
  //float grip_angle_r = radians(grip_angle_d);    //grip angle in radians for use in calculations
  /* Base angle and radial distance from x,y coordinates */
  float bas_angle_r = atan2(x, y);
  float rdist = sqrt((x * x) + (y * y));
  /* rdist is y coordinate for the arm */
  y = rdist;
  /* Grip offsets calculated based on grip angle */
  //float grip_off_z = (sin(grip_angle_r)) * GRIPPER;
  //float grip_off_y = (cos(grip_angle_r)) * GRIPPER;
  /* Wrist position */
  float wrist_z = z - BASE_HGT;
  float wrist_y = y;
  /* Shoulder to wrist distance (AKA sw) */
  float s_w = (wrist_z * wrist_z) + (wrist_y * wrist_y);
  float s_w_sqrt = sqrt(s_w);
  /* s_w angle to ground */
  float a1 = atan2(wrist_z, wrist_y);
  /* s_w angle to humerus */
  float a2 = acos(((hum_sq - uln_sq) + s_w) / (2 * HUMERUS * s_w_sqrt));
  /* shoulder angle */
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees(shl_angle_r);
  /* elbow angle */
  float elb_angle_r = acos((hum_sq + uln_sq - s_w) / (2 * HUMERUS * ULNA));
  float elb_angle_d = degrees(elb_angle_r);
  float elb_angle_dn = -(180.0 - elb_angle_d);
  
  /* wrist angle */
  //float wri_angle_d = (grip_angle_d - elb_angle_dn) - shl_angle_d;
 
  /* Servo pulses */
  float bas_servopulse = 1500.0 - ((degrees(bas_angle_r)) * 11.11);
  float shl_servopulse = SHL_SERVO_ERROR + ((shl_angle_d - 90.0) * 6.6);
  float elb_servopulse = ELB_SERVO_ERROR -  ((elb_angle_d - 90.0) * 6.6);
  //float wri_servopulse = WRI_SERVO_ERROR + (wri_angle_d  * 11.1);
  float wri_servopulse = WRI_SERVO_ERROR + (elb_servopulse - shl_servopulse);
 
  /* Set servos */
  BAS_SERVO.writeMicroseconds(ftl(bas_servopulse));
  WRI_SERVO.writeMicroseconds(ftl(wri_servopulse));
  SHL_SERVO.writeMicroseconds(ftl(shl_servopulse));
  ELB_SERVO.writeMicroseconds(ftl(elb_servopulse));
}
 
void zero_x()
{
  for(double yaxis = 150.0; yaxis < 356.0; yaxis += 1){
    set_arm(0, yaxis, 127.0, 0);
    delay(10);
  }
  for(double yaxis = 356.0; yaxis > 150.0; yaxis -= 1){
    set_arm(0, yaxis, 127.0, 0);
    delay(10);
  }
}
 
/* moves arm in a straight line */
void line()
{
    for(double xaxis = -100.0; xaxis < 100.0; xaxis += 0.5){
      set_arm(xaxis, 250, 100, 50);
      delay(10);
    }
    for(float xaxis = 100.0; xaxis > -100.0; xaxis -= 0.5){
      set_arm(xaxis, 250, 100, 50);
      delay(10);
    }
}
 
void circle()
{
  #define RADIUS 80.0
  //float angle = 0;
  float zaxis,yaxis;
  for(float angle = 0.0; angle < 360.0; angle += 1.0){
      yaxis = RADIUS * sin(radians(angle)) + 200;
      zaxis = RADIUS * cos(radians(angle)) + 200;
      set_arm(0, yaxis, zaxis, 90);
      delay(5);
  }
}
