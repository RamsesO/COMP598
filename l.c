#include <stdio.h>
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <math.h>
#include <time.h>
#include <stdlib.h>
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif

#define L_MOTOR_PORT      			OUTPUT_A
#define L_MOTOR_EXT_PORT  			EV3_PORT__NONE_
#define R_MOTOR_PORT      			OUTPUT_B
#define R_MOTOR_EXT_PORT  			EV3_PORT__NONE_
//I do not know if this will work!
#define L_TOUCH_SENSOR_PORT	  		OUTPUT_1 //<- is this a valid reference?
#define L_TOUCH_SENSOR_EXT_PORT  	EV3_PORT__NONE_
#define L_TOUCH_SENSOR_PORT   		OUTPUT_2
#define R_TOUCH_SENSOR_EXT_PORT   	EV3_PORT__NONE_
#define SPEED_LINEAR      			75
#define SPEED_CIRCULAR    			50
#define CIRCLE_RADIUS     			203.2 // metric millimeters
#define TIRE_RADIUS       			21.6 // diameter is 43.2
#define MAX_BLACK_RANGE   			15

int max_speed;  /* Motor maximal speed */
int is_button_pressed = 0;
int color_value = 100;


int app_alive;
int mode;
enum{
    MOVE_NONE,
    MOVE_FORWARD,
    MOVE_BACKWARD, // instead of moving towards circle we can move backwards
    TURN_ANGLE,
    TURN,
    STEP_BACKWARD,
    STEP_FORWARD,
};

int moving;
int command;
int angle;

uint8_t sn_touch, sn_color;
// to use multi set you need array of motors followed by set of desc limit
enum{ L,R}; // use this to set indicies of motor
uint8_t motor[ 3 ] = {DESC_LIMIT,DESC_LIMIT,DESC_LIMIT};
uint8_t touch_sensor[3] = {DESC_LIMIT,DESC_LIMIT,DESC_LIMIT};

static bool withinColRange(int beg, int end)
{
    if(color_value >= beg && color_value <= end)
        return true;
    else
        return false;
}

static bool _check_pressed( uint8_t sn )
{
    int val;
    if ( sn == SENSOR__NONE_ ) {
        return ( ev3_read_keys(( uint8_t *) &val ) && ( val & EV3_KEY_UP ));
    }
    return ( get_sensor_value( 0, sn, &val ) && ( val != 0 ));
}

static bool _has_motors(uint8_t *sn_l_motor, uint8_t *sn_r_motor)
{
    return ( ev3_search_tacho_plugged_in (OUTPUT_A, EV3_PORT__NONE_, sn_l_motor, 0 ) && ev3_search_tacho_plugged_in (OUTPUT_B, EV3_PORT__NONE_, sn_r_motor, 0 ));
}

static void _run_forever( int l_speed, int r_speed )
{
    set_tacho_speed_sp( motor[ L ], l_speed );
    set_tacho_speed_sp( motor[ R ], r_speed );
    multi_set_tacho_command_inx( motor, TACHO_RUN_FOREVER );
}

static void _run_to_rel_pos( int l_speed, int l_pos, int r_speed, int r_pos )
{
    set_tacho_speed_sp( motor[ L ], l_speed );
    set_tacho_speed_sp( motor[ R ], r_speed );
    set_tacho_position_sp( motor[ L ], l_pos );
    set_tacho_position_sp( motor[ R ], r_pos );
    multi_set_tacho_command_inx( motor, TACHO_RUN_TO_REL_POS );
}

//Code to turn the vehicle using wheel rotations
//Positive turns left, negative turns right
int turn_vehicle(int motor_speed, int number_of_rotations){

    //Get encoder counts for both wheels
    int encoder_count[2];
    get_tacho_position(motor[L], &encoder_count[L]);
    get_tacho_position(motor[R], &encoder_count[R]);

    //Give one motor more power than the other to turn
    //Turning right
    if(number_of_rotations < 0){
        set_tacho_speed_sp(motor[L], motor_speed);
        set_tacho_speed_sp(motor[R], -motor_speed);
    }
    //Turning left
    else {
        set_tacho_speed_sp(motor[R], motor_speed);
        set_tacho_speed_sp(motor[L], -motor_speed);
    }

    //ORder the motors turn run until...
    multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);

    printf("I am in here\n");
    //Get the initial reference for encoders
    int initial_encoder_reading_L = encoder_count[L];
    int initial_encoder_reading_R = encoder_count[R];
    //If the user requests a rotation
    if (number_of_rotations < 0) {
        //Turn left
        //Loop until the encoder counts have matched up
        while(encoder_count[L] > (initial_encoder_reading_L - (360 * number_of_rotations)) && !is_button_pressed && !withinColRange(0,MAX_BLACK_RANGE)){
            get_tacho_position(motor[L], &encoder_count[L]);
            if(!get_sensor_value(0, sn_color, &color_value)){
                color_value = 0;
            }
            if(_check_pressed(touch_sensor[L]) || _check_pressed(touch_sensor[R])){
                is_button_pressed = 1;
            }
        }
    } else {
        //Turn right
        //Loop until the counter is less than the initial reading
        while(encoder_count[R] < initial_encoder_reading_R + (360 * number_of_rotations) && !is_button_pressed && !withinColRange(0,MAX_BLACK_RANGE)){
            get_tacho_position(motor[R], &encoder_count[R]);
             if(!get_sensor_value(0, sn_color, &color_value)){
                color_value = 0;
            }
            if(_check_pressed(touch_sensor[L]) || _check_pressed(touch_sensor[R])){
                is_button_pressed = 1;
            }
        }
    }

    multi_set_tacho_command_inx(motor, TACHO_STOP);
}

static int _is_running( void )
{
    FLAGS_T state = TACHO_STATE__NONE_;
    get_tacho_state_flags( motor[ L ], &state );
    if ( state != TACHO_STATE__NONE_ ) return ( 1 );
    get_tacho_state_flags( motor[ R ], &state );
    if ( state != TACHO_STATE__NONE_ ) return ( 1 );
    return ( 0 );
}

static void _stop( void )
{
    multi_set_tacho_command_inx( motor, TACHO_STOP );
}


// returns degrees
static double _theta(int x /*chord length*/)
{
    double pi = 3.141592;
    double convert = 180/pi;
    double val = acos( (x/ (2*CIRCLE_RADIUS) ) );
    val = val*convert;
    return val; // law of cosine calculation creds to ram
}

//tires are 43.2 mm by 22 mm  meaning radius of
//converts degrees into distance metrics.
static double _distance_travled( int start, int end_degrees_rotated)
{
    double total_degrees = start - end_degrees_rotated;
    double distance = TIRE_RADIUS * (total_degrees/360);
    return distance;
}

//must return an integer since tachometers only take integers
static int _distance_to_degrees(double distance)
{
    double temp = (distance / TIRE_RADIUS)*360;
    int degrees = (int)round(temp);
    return degrees;
}

int app_init( void )
{
    char s[ 16 ];
	
	//Attempting to find left and right motors
    if ( ev3_search_tacho_plugged_in( L_MOTOR_PORT, L_MOTOR_EXT_PORT, motor + L, 0 )) {
        get_tacho_max_speed( motor[ L ], &max_speed );
        /* Reset the motor */
        set_tacho_command_inx( motor[ L ], TACHO_RESET );
    } else {
        printf( "LEFT motor (%s) is NOT found.\n", ev3_port_name( L_MOTOR_PORT, L_MOTOR_EXT_PORT, 0, s ));
        /* Inoperative without left motor */
        return ( 0 );
    }
    if ( ev3_search_tacho_plugged_in( R_MOTOR_PORT, R_MOTOR_EXT_PORT, motor + R, 0 )) {
        /* Reset the motor */
        set_tacho_command_inx( motor[ R ], TACHO_RESET );
    } else {
        printf( "RIGHT motor (%s) is NOT found.\n", ev3_port_name( R_MOTOR_PORT, R_MOTOR_EXT_PORT, 0, s ));
        /* Inoperative without right motor */
        return ( 0 );
    }

	//Activating color sensor
    if ( ev3_search_sensor( LEGO_EV3_COLOR, &sn_color, 0 )) {
        printf("color activated");
    }
    command = moving = MOVE_NONE;
	
	//Attempting to find left and right touch sensors
	//TODO verify these function calls exist and delete this comment.
    if ( ev3_search_sensor_plugged_in(L_TOUCH_SENSOR_PORT, L_TOUCH_SENSOR_EXT_PORT, touch_sensor + L, 0 )) {
        //printf( " use the TOUCH sensor.\n" );
    } else {
        sn_touch = DESC_LIMIT;
        printf( "LEFT touch sensor (%s) not found.\n", ev3_port_name( L_TOUCH_SENSOR_PORT, L_TOUCH_SENSOR_EXT_PORT, 0, s ));
    }
	if ( ev3_search_sensor_plugged_in(L_TOUCH_SENSOR_PORT, L_TOUCH_SENSOR_EXT_PORT, touch_sensor + L, 0 )) {
        //printf( " use the TOUCH sensor.\n" );
    } else {
        sn_touch = DESC_LIMIT;
        printf( "RIGHT touch sensor (%s) not found.\n", ev3_port_name( R_TOUCH_SENSOR_PORT, R_TOUCH_SENSOR_EXT_PORT, 0, s ));
    }
    return ( 1 );
}


//Sets the motors to hold their position when ordered to stop.
//This makes it hard to move the wheels
void set_motor_variables(){
    set_tacho_stop_action_inx(motor[L], TACHO_HOLD);
    set_tacho_stop_action_inx(motor[R], TACHO_HOLD);
    _stop();
}

int main( void )
{
    printf( "Waiting the EV3 brick online...\n" );
    if ( ev3_init() < 1 ) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
	//Initialize sensors
    ev3_sensor_init();
	//Initialize motors
    ev3_tacho_init();
	//Assign all constants etc for operation
    app_alive=app_init();
    set_motor_variables();
    is_button_pressed = 0;
    srand(time(NULL));
	
	//Operational logic
	//This loop will never terminate!
    while(app_alive) {
		//Set vehicle to move forward with half of the max speed for both sides
        set_tacho_speed_sp(motor[L], max_speed/2);
        set_tacho_speed_sp(motor[R], max_speed/2);
        multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);
		
		//drive forward until:
		//A touch sensor is activated
		//The color sensor detects a black object
        while(!is_button_pressed && !withinColRange(0,MAX_BLACK_RANGE)){
			//Check that either touch sensor has been activated
            if(_check_pressed(touch_sensor[L]) || _check_pressed(touch_sensor[R])){
                is_button_pressed = 1;
            }
			//Get new color value
            get_sensor_value(0, sn_color, &color_value);
        }
		
		//If the color currently detected is not within the black threshold
		//activate the backup and random turning sequence
        if(!withinColRange(0,MAX_BLACK_RANGE)){
			//Order motors to stop
            multi_set_tacho_command_inx(motor, TACHO_STOP);
			//Set to go in reverse and drive backwards
            set_tacho_speed_sp(motor[L], -max_speed/2);
            set_tacho_speed_sp(motor[R], -max_speed/2);
            multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);
			//Back up for 1 second
            Sleep(1000);
			//Order the motors to stop
            multi_set_tacho_command_inx(motor, TACHO_STOP);
			//randomly turn from 2 - 6 wheel rotations
            turn_vehicle(max_speed/2, rand()%4 + 2);
		
		//If we left the loop we are in range we see black
		//At this point we go full speed for a short while to push others off and stop
        } else {
			//Set full speed and drive forward
            set_tacho_speed_sp(motor[L], max_speed);
            set_tacho_speed_sp(motor[R], max_speed);
            multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);
			//Forward for 200ms
            Sleep(200);
			//Order the motors to stop
            multi_set_tacho_command_inx(motor, TACHO_STOP);
        }
		
		//If it is the condition where we currently see black the car will stop
		//If black is no longer detected we will begin to search again
        while(withinColRange(0, MAX_BLACK_RANGE)){
            multi_set_tacho_command_inx(motor, TACHO_STOP);
            get_sensor_value(0, sn_color, &color_value);
        }
		
		//Reset button flag
        is_button_pressed = 0;
    }
	
	//If we leave the while loop the vehicle will cease moving
    multi_set_tacho_command_inx(motor, TACHO_STOP);
    return(0);
}