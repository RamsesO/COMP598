#include <stdio.h>
#include "coroutine.h"
#include "ev3.h"
#include "ev3_port.h"
#include "ev3_sensor.h"
#include "ev3_tacho.h"
// WIN32 /////////////////////////////////////////
#ifdef __WIN32__
#include <windows.h>
// UNIX //////////////////////////////////////////
#else
#include <unistd.h>
#define Sleep( msec ) usleep(( msec ) * 1000 )
#endif

#define L_MOTOR_PORT      OUTPUT_A
#define L_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define R_MOTOR_PORT      OUTPUT_B
#define R_MOTOR_EXT_PORT  EXT_PORT__NONE_
#define SPEED_LINEAR      75

int max_speed;  /* Motor maximal speed */

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

static bool _run_forever(uint8_t sn_l_motor, uint8_t sn_r_motor)
{
    get_tacho_max_speed( sn_l_motor, &max_speed );
    set_tacho_speed_sp(sn_l_motor,max_speed);
    set_tacho_speed_sp(sn_r_motor,-max_speed);
    set_tacho_command_inx( sn_l_motor,TACHO_RUN_FOREVER );
    set_tacho_command_inx( sn_r_motor,TACHO_RUN_FOREVER );
}

// int app_init( void )
// {
//     char s[ 16 ];

//     get_tacho_max_speed( , &max_speed );
//     set_tacho_command_inx( , TACHO_RESET );
//     set_tacho_command_inx( , TACHO_RESET );
// }

int main( void )
{
    int i;
    FLAGS_T state;
    char s[ 256 ];
    uint8_t sn_touch, sn_l_motor, sn_r_motor;
    int max_speed;
    printf( "Waiting the EV3 brick online...\n" );
    if ( ev3_init() < 1 ) return ( 1 );
    printf( "*** ( EV3 ) Hello! ***\n" );
    ev3_sensor_init();
    ev3_tacho_init();
    for ( ; ; ) {
        if (_has_motors(&sn_l_motor, &sn_r_motor))
        {
            _run_forever(sn_l_motor,sn_r_motor);
        }
        if ( _check_pressed( sn_touch )) break;
        Sleep( 200 );
        if ( _check_pressed( sn_touch )) break;
        Sleep( 200 );
    }
    set_tacho_stop_action_inx(sn_l_motor, TACHO_COAST);
    set_tacho_stop_action_inx(sn_r_motor, TACHO_COAST);
    set_tacho_command_inx(sn_l_motor, TACHO_STOP);
    set_tacho_command_inx(sn_r_motor, TACHO_STOP);
    ev3_uninit();
    printf( "*** ( EV3 ) Bye! ***\n" );
    return ( 0 );
}
