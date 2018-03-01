#include <stdio.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
#include <math.h>
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
#define RADIUS            8 //inches should we use metric ?

int max_speed;  /* Motor maximal speed */

#define DEGREE_TO_COUNT(d) ((d) * 260 / 90);

int app_alive;
int mode;
enum{
    MOVE_NONE,
    MOVE_FORWARD,
    MOVE_BACKWARD, // instead of moving towards circle we can move backwards
    TURN_ANGLE,
    TURN_LEFT,
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
    double val = acos( (x/ (2*RADIUS) ) );
    val = val*convert;
    return val; // law of cosine calculation creds to ram
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
    CORO_LOCAL int val;
    CORO_BEGIN();
    set_sensor_mode( sn_color, "COL-COLOR" );
        for ( ; ; ) {
            if ( !get_sensor_value( 0, sn_color, &val ) || ( val < 0 ) ) 
            {
                val = 0;
            }
            if (val == 0)
            {
                printf( "hi \n" );
            }else{
            }
            CORO_YIELD();
        }
            

    CORO_END();
}

CORO_DEFINE( drive )
{
    CORO_LOCAL int speed_linear, speed_circular;
    CORO_LOCAL int _wait_stopped;
    CORO_BEGIN();
    speed_linear = max_speed * SPEED_LINEAR / 100;
    speed_circular = max_speed * SPEED_CIRCULAR / 100;
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
        }
        moving = command;
        if ( _wait_stopped ) {
            // Waiting the command is completed 
            CORO_WAIT( !_is_running());
            command = moving = MOVE_NONE;
        }
    }
    CORO_END();
}

int main( void )
{
    printf( "Waiting the EV3 brick online...\n" );
    if ( ev3_init() < 1 ) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
    ev3_sensor_init();
    ev3_tacho_init();
    app_alive=app_init();
    while(app_alive){
        CORO_CALL( handle_touch );
        CORO_CALL( drive );
        CORO_CALL( handle_color);
        // if ( _check_pressed( sn_touch )) break;
        // Sleep( 200 );
    }   
    return(0);
}
