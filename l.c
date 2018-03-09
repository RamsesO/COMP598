#include <stdio.h>
#include "coroutine.h"
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

#define L_MOTOR_PORT      OUTPUT_A
#define L_MOTOR_EXT_PORT  EV3_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EV3_PORT__NONE_
#define SPEED_LINEAR      75
#define SPEED_CIRCULAR    50
#define CIRCLE_RADIUS     203.2 // metric milimeters
#define TIRE_RADIUS       21.6 // diameter is 43.2

#define MAX_BLACK_RANGE 15

int max_speed;  /* Motor maximal speed */
int is_button_pressed = 0;
int color_value = 0;


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
    //Get the inital reference for encoders
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
            if(_check_pressed(sn_touch)){
                is_button_pressed = 1;
            }
        }
    } else {
        //Turn right
        //Loop until the couner counts are less than the inital reading
        while(encoder_count[R] < initial_encoder_reading_R + (360 * number_of_rotations) && !is_button_pressed && !withinColRange(0,MAX_BLACK_RANGE)){
            get_tacho_position(motor[R], &encoder_count[R]);
             if(!get_sensor_value(0, sn_color, &color_value)){
                color_value = 0;
            }
            if(_check_pressed(sn_touch)){
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

    if ( ev3_search_sensor( LEGO_EV3_COLOR, &sn_color, 0 )) {
        printf("color activated");
    }
    command = moving = MOVE_NONE;
    if ( ev3_search_sensor( LEGO_EV3_TOUCH, &sn_touch, 0 )) {
        printf( " use the TOUCH sensor.\n" );
    } else {
        sn_touch = DESC_LIMIT;
        printf( " press UP on the EV3 brick.\n" );
    }
    return ( 1 );
}

/*
CORO_CONTEXT( turn );



*/
CORO_CONTEXT( handle_touch );
CORO_CONTEXT( handle_color);
CORO_CONTEXT( drive );
CORO_CONTEXT( measure_distance);


CORO_DEFINE( handle_touch )
{
    CORO_LOCAL int val;
    CORO_BEGIN();
    if ( sn_touch == DESC_LIMIT ) CORO_QUIT();
    for ( ; ; ) {
        // Waiting the button is pressed
        CORO_WAIT( get_sensor_value( 0, sn_touch, &val ) && ( val ));
        // Stop the vehicle
        if (command == MOVE_BACKWARD)
        {
            command = MOVE_FORWARD;
        }else{
            command = MOVE_BACKWARD;
        }

    }
    CORO_END();
}

CORO_DEFINE( handle_color)
{
    CORO_LOCAL int color_val;
    CORO_BEGIN();
    set_sensor_mode( sn_color, "COL-AMBIENT" );
        for ( ; ; ) {

            if ( !get_sensor_value( 0, sn_color, &color_val ) || ( color_val < 0 ) )
            {
                color_val = 0;
            }
            if (color_val == 0)
            {
                CORO_CALL(measure_distance);
            }
            CORO_YIELD();
        }


    CORO_END();
}

CORO_DEFINE(measure_distance)
{
    CORO_LOCAL int l_travled, r_travled;
    CORO_BEGIN();
    int l_pos,l_pos_end;
    int r_pos,r_pos_end;
    get_tacho_position(motor[L],&l_pos);
    get_tacho_position(motor[R],&r_pos);
    for( ; ; ){
        CORO_WAIT(l_travled != 1);
        get_tacho_position( motor[L], &l_pos_end);
        get_tacho_position( motor[R], &r_pos_end);
        l_travled = l_pos_end;
        r_travled = r_pos_end;
    }
    CORO_END();
}

CORO_DEFINE( drive )
{
    CORO_LOCAL int speed_linear, speed_circular;
    CORO_LOCAL int _wait_stopped;
    CORO_BEGIN();
    speed_linear = max_speed/2;
    speed_circular = max_speed/2;
    for ( ; ; ) {
        // /* Waiting new command
        CORO_WAIT( moving != command );
        _wait_stopped = 0;
        switch ( command ) {
        case MOVE_NONE:
            _stop();
            _wait_stopped = 1;
            break;
        case MOVE_FORWARD:
            _run_forever( speed_linear, speed_linear );
            break;
        case MOVE_BACKWARD:
            _run_forever( -speed_linear, -speed_linear );
            break;
        case TURN:
            turn_vehicle(speed_circular, 10);
        }
        moving = command;
        if ( _wait_stopped ) {
            // Waiting the command is completed
            //CORO_WAIT( !_is_running());
            command = moving = MOVE_NONE;
        }
    }
}


//Sets the motors to hold their position when ordered to stop.
//This makes it hard to move the wheels
void set_motor_variables(){
    set_tacho_stop_action_inx(motor[L], TACHO_HOLD);
    set_tacho_stop_action_inx(motor[R], TACHO_HOLD);
    _stop();
}

bool withinColRange(int beg, int end)
{
    if(color_value >= beg && color_value <= end)
        return true;
    else
        return false;
}

int main( void )
{
    printf( "Waiting the EV3 brick online...\n" );
    if ( ev3_init() < 1 ) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
    ev3_sensor_init();
    ev3_tacho_init();
    app_alive=app_init();
    set_motor_variables();
    //command = TURN;
    is_button_pressed = 0;
    srand(time(NULL));
    while(app_alive && !withinColRange(0,MAX_BLACK_RANGE)){
       set_tacho_speed_sp(motor[L], max_speed/2);
       set_tacho_speed_sp(motor[R], max_speed/2);
       multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);
       while(!is_button_pressed && !withinColRange(0,MAX_BLACK_RANGE)){
            if(_check_pressed(sn_touch)){
                is_button_pressed = 1;
            }
            get_sensor_value(0, sn_color, &color_value);
        }
      multi_set_tacho_command_inx(motor, TACHO_STOP);
      set_tacho_speed_sp(motor[L], -max_speed/2);
      set_tacho_speed_sp(motor[R], -max_speed/2);
      multi_set_tacho_command_inx(motor, TACHO_RUN_FOREVER);
      Sleep(1000);
      multi_set_tacho_command_inx(motor, TACHO_STOP);
      is_button_pressed = 0;
      if(!withinColRange(0,MAX_BLACK_RANGE)){
          turn_vehicle(max_speed/2, rand()%10);
      }
      is_button_pressed = 0;
    }
    command = MOVE_NONE;
    CORO_CALL(drive);
    return(0);
}