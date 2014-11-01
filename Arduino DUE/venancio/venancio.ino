/*
	Venenacio 5-axis robot Arm
	Oscar Gonzalez - 2013-2015
	www.bricogeek.com

	Source, schematics and 3D models:
	https://github.com/oscargonzalez/venancio

	Some code inspired from Oleg Mazurov, many thanks!
	http://www.circuitsathome.com/mcu/robotic-arm-inverse-kinematics-on-arduino

	LICENSE:
	Creative Commons Attribution, Share-Alike license. All text above must be included in any redistribution
	http://creativecommons.org/licenses/by-sa/2.5/es/

*/

/* Arm dimensions( mm ) */
#define BASE_HGT 67.31      // base hight
#define HUMERUS 146.05      // shoulder-to-elbow "bone"
#define ULNA 187.325        // elbow-to-wrist "bone"
#define GRIPPER 100.00          //gripper (incl.heavy duty wrist rotate mechanism) length 3.94"
 
#define ftl(x) ((x)>=0?(long)((x)+0.5):(long)((x)-0.5))  //float to long conversion
 
/* Servo names/numbers */
/* Base servo HS-485HB */
#define BAS_SERVO 0
/* Shoulder Servo HS-5745-MG */
#define SHL_SERVO 1
/* Elbow Servo HS-5745-MG */
#define ELB_SERVO 2
/* Wrist servo HS-645MG */
#define WRI_SERVO 3
/* Wrist rotate servo HS-485HB */
#define WRO_SERVO 4
/* Gripper servo HS-422 */
#define GRI_SERVO 5
 
/* pre-calculations */
float hum_sq = HUMERUS*HUMERUS;
float uln_sq = ULNA*ULNA;

/* arm positioning routine utilizing inverse kinematics */
/* z is height, y is distance from base center out, x is side to side. y,z can only be positive */
//void set_arm( uint16_t x, uint16_t y, uint16_t z, uint16_t grip_angle )
void set_arm( float x, float y, float z, float grip_angle_d )
{
  float grip_angle_r = radians( grip_angle_d );    //grip angle in radians for use in calculations
  /* Base angle and radial distance from x,y coordinates */
  float bas_angle_r = atan2( x, y );
  float rdist = sqrt(( x * x ) + ( y * y ));
  /* rdist is y coordinate for the arm */
  y = rdist;
  /* Grip offsets calculated based on grip angle */
  float grip_off_z = ( sin( grip_angle_r )) * GRIPPER;
  float grip_off_y = ( cos( grip_angle_r )) * GRIPPER;
  /* Wrist position */
  float wrist_z = ( z - grip_off_z ) - BASE_HGT;
  float wrist_y = y - grip_off_y;
  /* Shoulder to wrist distance ( AKA sw ) */
  float s_w = ( wrist_z * wrist_z ) + ( wrist_y * wrist_y );
  float s_w_sqrt = sqrt( s_w );
  /* s_w angle to ground */
  //float a1 = atan2( wrist_y, wrist_z );
  float a1 = atan2( wrist_z, wrist_y );
  /* s_w angle to humerus */
  float a2 = acos((( hum_sq - uln_sq ) + s_w ) / ( 2 * HUMERUS * s_w_sqrt ));
  /* shoulder angle */
  float shl_angle_r = a1 + a2;
  float shl_angle_d = degrees( shl_angle_r );
  /* elbow angle */
  float elb_angle_r = acos(( hum_sq + uln_sq - s_w ) / ( 2 * HUMERUS * ULNA ));
  float elb_angle_d = degrees( elb_angle_r );
  float elb_angle_dn = -( 180.0 - elb_angle_d );
  /* wrist angle */
  float wri_angle_d = ( grip_angle_d - elb_angle_dn ) - shl_angle_d;
 
  /* Servo pulses */
  float bas_servopulse = 1500.0 - (( degrees( bas_angle_r )) * 11.11 );
  float shl_servopulse = 1500.0 + (( shl_angle_d - 90.0 ) * 6.6 );
  float elb_servopulse = 1500.0 -  (( elb_angle_d - 90.0 ) * 6.6 );
  float wri_servopulse = 1500 + ( wri_angle_d  * 11.1 );
 
  /* Set servos */
  /*
  servos.setposition( BAS_SERVO, ftl( bas_servopulse ));
  servos.setposition( WRI_SERVO, ftl( wri_servopulse ));
  servos.setposition( SHL_SERVO, ftl( shl_servopulse ));
  servos.setposition( ELB_SERVO, ftl( elb_servopulse ));
  */
 
}

void setup()
{
	
}

void loop()
{
	
}