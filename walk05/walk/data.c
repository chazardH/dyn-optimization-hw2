/*****************************************************************************/
/*
  data.c: save data from simulation
*/
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "../useful/mrdplot/mrdplot.h"

#include "main.h"
#include "main2.h"

/*****************************************************************************/

extern SIM sim;

/*****************************************************************************/

enum channels {
MRDPLOT_L_HIP=0,
MRDPLOT_L_KNEE,
MRDPLOT_L_ANKLE,
MRDPLOT_R_HIP,
MRDPLOT_R_KNEE,
MRDPLOT_R_ANKLE,
MRDPLOT_L_HIP_D,
MRDPLOT_L_KNEE_D,
MRDPLOT_L_ANKLE_D,
MRDPLOT_R_HIP_D,
MRDPLOT_R_KNEE_D,
MRDPLOT_R_ANKLE_D,
MRDPLOT_L_HIPD,
MRDPLOT_L_KNEED,
MRDPLOT_L_ANKLED,
MRDPLOT_R_HIPD,
MRDPLOT_R_KNEED,
MRDPLOT_R_ANKLED,
MRDPLOT_L_HIPD_D,
MRDPLOT_L_KNEED_D,
MRDPLOT_L_ANKLED_D,
MRDPLOT_R_HIPD_D,
MRDPLOT_R_KNEED_D,
MRDPLOT_R_ANKLED_D,
MRDPLOT_X,
MRDPLOT_Z,
MRDPLOT_PITCH,
MRDPLOT_XD,
MRDPLOT_ZD,
MRDPLOT_PITCHD,
MRDPLOT_L_HIP_TORQUE,
MRDPLOT_L_KNEE_TORQUE,
MRDPLOT_L_ANKLE_TORQUE,
MRDPLOT_R_HIP_TORQUE,
MRDPLOT_R_KNEE_TORQUE,
MRDPLOT_R_ANKLE_TORQUE,
MRDPLOT_HEAD_X,
MRDPLOT_HEAD_Z,
MRDPLOT_HIP_X,
MRDPLOT_HIP_Z,
MRDPLOT_L_KNEE_X,
MRDPLOT_L_KNEE_Z,
MRDPLOT_R_KNEE_X,
MRDPLOT_R_KNEE_Z,
MRDPLOT_L_FOOT_X,
MRDPLOT_L_FOOT_Z,
MRDPLOT_R_FOOT_X,
MRDPLOT_R_FOOT_Z,
MRDPLOT_L_FOOT_FX,
MRDPLOT_L_FOOT_FZ,
MRDPLOT_R_FOOT_FX,
MRDPLOT_R_FOOT_FZ,
MRDPLOT_STATE,
MRDPLOT_STATE_TIME,
MRDPLOT_TIME,
MRDPLOT_TORSO_ABS_ANGLE,
MRDPLOT_L_THIGH_ABS_ANGLE,
MRDPLOT_L_CALF_ABS_ANGLE,
MRDPLOT_R_THIGH_ABS_ANGLE,
MRDPLOT_R_CALF_ABS_ANGLE,
MRDPLOT_L_HIP_COMMAND,
MRDPLOT_L_KNEE_COMMAND,
MRDPLOT_L_ANKLE_COMMAND,
MRDPLOT_R_HIP_COMMAND,
MRDPLOT_R_KNEE_COMMAND,
MRDPLOT_R_ANKLE_COMMAND,
N_CHANNELS
};

#define MAX_N_POINTS 100000

int n_points = 0;
float data[MAX_N_POINTS*N_CHANNELS];

char *names[N_CHANNELS];
char *units[N_CHANNELS];

/*****************************************************************************/

void init_data( SIM *s )
{
  int i;

  /*
  printf( "N_CHANNELS: %d; MRDPLOT_L_HIP: %d; last: %d\n", N_CHANNELS,
	  MRDPLOT_L_HIP, MRDPLOT_R_ANKLE_COMMAND );
  */

  for( i = 0; i < N_CHANNELS; i++ )
    {
      names[i] = "xxx";
      units[i] = "xxx";
    }

  names[MRDPLOT_L_HIP] = "l_hip";
  names[MRDPLOT_L_KNEE] = "l_knee";
  names[MRDPLOT_L_ANKLE] = "l_ankle";
  
  names[MRDPLOT_R_HIP] = "r_hip";
  names[MRDPLOT_R_KNEE] = "r_knee";
  names[MRDPLOT_R_ANKLE] = "r_ankle";

  names[MRDPLOT_L_HIP_D] = "l_hip_d";
  names[MRDPLOT_L_KNEE_D] = "l_knee_d";
  names[MRDPLOT_L_ANKLE_D] = "l_ankle_d";
  
  names[MRDPLOT_R_HIP_D] = "r_hip_d";
  names[MRDPLOT_R_KNEE_D] = "r_knee_d";
  names[MRDPLOT_R_ANKLE_D] = "r_ankle_d";
  
  names[MRDPLOT_L_HIPD] = "l_hipd";
  names[MRDPLOT_L_KNEED] = "l_kneed";
  names[MRDPLOT_L_ANKLED] = "l_ankled";
  
  names[MRDPLOT_R_HIPD] = "r_hipd";
  names[MRDPLOT_R_KNEED] = "r_kneed";
  names[MRDPLOT_R_ANKLED] = "r_ankled";
  
  names[MRDPLOT_L_HIPD_D] = "l_hipd_d";
  names[MRDPLOT_L_KNEED_D] = "l_kneed_d";
  names[MRDPLOT_L_ANKLED_D] = "l_ankled_d";
  
  names[MRDPLOT_R_HIPD_D] = "r_hipd_d";
  names[MRDPLOT_R_KNEED_D] = "r_kneed_d";
  names[MRDPLOT_R_ANKLED_D] = "r_ankled_d";

  names[MRDPLOT_X] = "x";
  names[MRDPLOT_Z] = "z";
  names[MRDPLOT_PITCH] = "pitch";
  
  names[MRDPLOT_XD] = "xd";
  names[MRDPLOT_ZD] = "zd";
  names[MRDPLOT_PITCHD] = "pitchd";
  
  names[MRDPLOT_L_HIP_TORQUE] = "l_hip_torque";
  names[MRDPLOT_L_KNEE_TORQUE] = "l_knee_torque";
  names[MRDPLOT_L_ANKLE_TORQUE] = "l_ankle_torque";
  
  names[MRDPLOT_R_HIP_TORQUE] = "r_hip_torque";
  names[MRDPLOT_R_KNEE_TORQUE] = "r_knee_torque";
  names[MRDPLOT_R_ANKLE_TORQUE] = "r_ankle_torque";
  
  names[MRDPLOT_HEAD_X] = "head_x";
  names[MRDPLOT_HEAD_Z] = "head_z";
  names[MRDPLOT_HIP_X] = "hip_x";
  names[MRDPLOT_HIP_Z] = "hip_z";
  names[MRDPLOT_L_KNEE_X] = "l_knee_x";
  names[MRDPLOT_L_KNEE_Z] = "l_knee_z";

  names[MRDPLOT_R_KNEE_X] = "r_knee_x";
  names[MRDPLOT_R_KNEE_Z] = "r_knee_z";

  names[MRDPLOT_L_FOOT_X] = "l_foot_x";
  names[MRDPLOT_L_FOOT_Z] = "l_foot_z";
  
  names[MRDPLOT_R_FOOT_X] = "r_foot_x";
  names[MRDPLOT_R_FOOT_Z] = "r_foot_z";
  
  names[MRDPLOT_L_FOOT_FX] = "l_foot_fx";
  names[MRDPLOT_L_FOOT_FZ] = "l_foot_fz";
  
  names[MRDPLOT_R_FOOT_FX] = "r_foot_fx";
  names[MRDPLOT_R_FOOT_FZ] = "r_foot_fz";
  
  names[MRDPLOT_STATE] = "state";
  names[MRDPLOT_STATE_TIME] = "state_time";
  names[MRDPLOT_TIME] = "time";
  
  names[MRDPLOT_TORSO_ABS_ANGLE] = "torso_abs_angle";
  
  names[MRDPLOT_L_THIGH_ABS_ANGLE] = "l_thigh_abs_angle";
  names[MRDPLOT_L_CALF_ABS_ANGLE] = "l_calf_abs_angle";
  
  names[MRDPLOT_R_THIGH_ABS_ANGLE] = "r_thigh_abs_angle";
  names[MRDPLOT_R_CALF_ABS_ANGLE] = "r_calf_abs_angle";
  
  names[MRDPLOT_L_HIP_COMMAND] = "l_hip_command";
  names[MRDPLOT_L_KNEE_COMMAND] = "l_knee_command";
  names[MRDPLOT_L_ANKLE_COMMAND] = "l_ankle_command";

  names[MRDPLOT_R_HIP_COMMAND] = "r_hip_command";
  names[MRDPLOT_R_KNEE_COMMAND] = "r_knee_command";
  names[MRDPLOT_R_ANKLE_COMMAND] = "r_ankle_command";

  units[MRDPLOT_L_HIP] = "r";
  units[MRDPLOT_L_KNEE] = "r";
  units[MRDPLOT_L_ANKLE] = "r";
  
  units[MRDPLOT_R_HIP] = "r";
  units[MRDPLOT_R_KNEE] = "r";
  units[MRDPLOT_R_ANKLE] = "r";

  units[MRDPLOT_L_HIP_D] = "r";
  units[MRDPLOT_L_KNEE_D] = "r";
  units[MRDPLOT_L_ANKLE_D] = "r";
  
  units[MRDPLOT_R_HIP_D] = "r";
  units[MRDPLOT_R_KNEE_D] = "r";
  units[MRDPLOT_R_ANKLE_D] = "r";
  
  units[MRDPLOT_L_HIPD] = "r/s";
  units[MRDPLOT_L_KNEED] = "r/s";
  units[MRDPLOT_L_ANKLED] = "r/s";
  
  units[MRDPLOT_R_HIPD] = "r/s";
  units[MRDPLOT_R_KNEED] = "r/s";
  units[MRDPLOT_R_ANKLED] = "r/s";
  
  units[MRDPLOT_L_HIPD_D] = "r/s";
  units[MRDPLOT_L_KNEED_D] = "r/s";
  units[MRDPLOT_L_ANKLED_D] = "r/s";
  
  units[MRDPLOT_R_HIPD_D] = "r/s";
  units[MRDPLOT_R_KNEED_D] = "r/s";
  units[MRDPLOT_R_ANKLED_D] = "r/s";

  units[MRDPLOT_X] = "m";
  units[MRDPLOT_Z] = "m";
  units[MRDPLOT_PITCH] = "r";
  
  units[MRDPLOT_XD] = "m/s";
  units[MRDPLOT_ZD] = "m/s";
  units[MRDPLOT_PITCHD] = "r/s";
  
  units[MRDPLOT_L_HIP_TORQUE] = "Nm";
  units[MRDPLOT_L_KNEE_TORQUE] = "Nm";
  units[MRDPLOT_L_ANKLE_TORQUE] = "Nm";
  
  units[MRDPLOT_R_HIP_TORQUE] = "Nm";
  units[MRDPLOT_R_KNEE_TORQUE] = "Nm";
  units[MRDPLOT_R_ANKLE_TORQUE] = "Nm";
  
  units[MRDPLOT_HEAD_X] = "m";
  units[MRDPLOT_HEAD_Z] = "m";
  units[MRDPLOT_HIP_X] = "m";
  units[MRDPLOT_HIP_Z] = "m";
  units[MRDPLOT_L_KNEE_X] = "m";
  units[MRDPLOT_L_KNEE_Z] = "m";

  units[MRDPLOT_R_KNEE_X] = "m";
  units[MRDPLOT_R_KNEE_Z] = "m";

  units[MRDPLOT_L_FOOT_X] = "m";
  units[MRDPLOT_L_FOOT_Z] = "m";
  
  units[MRDPLOT_R_FOOT_X] = "m";
  units[MRDPLOT_R_FOOT_Z] = "m";
  
  units[MRDPLOT_L_FOOT_FX] = "N";
  units[MRDPLOT_L_FOOT_FZ] = "N";
  
  units[MRDPLOT_R_FOOT_FX] = "N";
  units[MRDPLOT_R_FOOT_FZ] = "N";
  
  units[MRDPLOT_STATE] = "-";
  units[MRDPLOT_STATE_TIME] = "s";
  units[MRDPLOT_TIME] = "s";
  
  units[MRDPLOT_TORSO_ABS_ANGLE] = "r";
  
  units[MRDPLOT_L_THIGH_ABS_ANGLE] = "r";
  units[MRDPLOT_L_CALF_ABS_ANGLE] = "r";
  
  units[MRDPLOT_R_THIGH_ABS_ANGLE] = "r";
  units[MRDPLOT_R_CALF_ABS_ANGLE] = "r";
  
  units[MRDPLOT_L_HIP_COMMAND] = "Nm";
  units[MRDPLOT_L_KNEE_COMMAND] = "Nm";
  units[MRDPLOT_L_ANKLE_COMMAND] = "Nm";

  units[MRDPLOT_R_HIP_COMMAND] = "Nm";
  units[MRDPLOT_R_KNEE_COMMAND] = "Nm";
  units[MRDPLOT_R_ANKLE_COMMAND] = "Nm";
  
  /*
  for( i = 0; i < N_CHANNELS; i++ )
    {
      printf( "%d %s %s\n", i, names[i], units[i] );
    }
  */
}

/*****************************************************************************/

int save_data( SIM *s )
{
  static int myindex = 0;

  if ( n_points >= MAX_N_POINTS )
    return 0;

  data[myindex + MRDPLOT_L_HIP] = s->hip_angle[LEFT];
  data[myindex + MRDPLOT_L_KNEE] = s->knee_angle[LEFT];
  data[myindex + MRDPLOT_L_ANKLE] = s->ankle_angle[LEFT];

  data[myindex + MRDPLOT_R_HIP] = s->hip_angle[RIGHT];
  data[myindex + MRDPLOT_R_KNEE] = s->knee_angle[RIGHT];
  data[myindex + MRDPLOT_R_ANKLE] = s->ankle_angle[RIGHT];

  data[myindex + MRDPLOT_L_HIP_D] = s->hip_angle_d[LEFT];
  data[myindex + MRDPLOT_L_KNEE_D] = s->knee_angle_d[LEFT];
  data[myindex + MRDPLOT_L_ANKLE_D] = s->ankle_angle_d[LEFT];

  data[myindex + MRDPLOT_R_HIP_D] = s->hip_angle_d[RIGHT];
  data[myindex + MRDPLOT_R_KNEE_D] = s->knee_angle_d[RIGHT];
  data[myindex + MRDPLOT_R_ANKLE_D] = s->ankle_angle_d[RIGHT];

  data[myindex + MRDPLOT_L_HIPD] = s->hip_angled[LEFT];
  data[myindex + MRDPLOT_L_KNEED] = s->knee_angled[LEFT];
  data[myindex + MRDPLOT_L_ANKLED] = s->ankle_angled[LEFT];

  data[myindex + MRDPLOT_R_HIPD] = s->hip_angled[RIGHT];
  data[myindex + MRDPLOT_R_KNEED] = s->knee_angled[RIGHT];
  data[myindex + MRDPLOT_R_ANKLED] = s->ankle_angled[RIGHT];

  data[myindex + MRDPLOT_L_HIPD_D] = s->hip_angled_d[LEFT];
  data[myindex + MRDPLOT_L_KNEED_D] = s->knee_angled_d[LEFT];
  data[myindex + MRDPLOT_L_ANKLED_D] = s->ankle_angled_d[LEFT];

  data[myindex + MRDPLOT_R_HIPD_D] = s->hip_angled_d[RIGHT];
  data[myindex + MRDPLOT_R_KNEED_D] = s->knee_angled_d[RIGHT];
  data[myindex + MRDPLOT_R_ANKLED_D] = s->ankle_angled_d[RIGHT];

  data[myindex + MRDPLOT_X] = s->hip[XX];
  data[myindex + MRDPLOT_Z] = s->hip[ZZ];
  data[myindex + MRDPLOT_PITCH] = s->pitch;

  data[myindex + MRDPLOT_XD] = s->hipd[XX];
  data[myindex + MRDPLOT_ZD] = s->hipd[ZZ];
  data[myindex + MRDPLOT_PITCHD] = s->pitchd;

  data[myindex + MRDPLOT_L_HIP_TORQUE] = s->hip_torque[LEFT];
  data[myindex + MRDPLOT_L_KNEE_TORQUE] = s->knee_torque[LEFT];
  data[myindex + MRDPLOT_L_ANKLE_TORQUE] = s->ankle_torque[LEFT];

  data[myindex + MRDPLOT_R_HIP_TORQUE] = s->hip_torque[RIGHT];
  data[myindex + MRDPLOT_R_KNEE_TORQUE] = s->knee_torque[RIGHT];
  data[myindex + MRDPLOT_R_ANKLE_TORQUE] = s->ankle_torque[RIGHT];

  data[myindex + MRDPLOT_HEAD_X] = s->head[XX];
  data[myindex + MRDPLOT_HEAD_Z] = s->head[ZZ];
  data[myindex + MRDPLOT_HIP_X] = s->hip[XX];
  data[myindex + MRDPLOT_HIP_Z] = s->hip[ZZ];
  data[myindex + MRDPLOT_L_KNEE_X] = s->knee[LEFT][XX];
  data[myindex + MRDPLOT_L_KNEE_Z] = s->knee[LEFT][ZZ];
  data[myindex + MRDPLOT_R_KNEE_X] = s->knee[RIGHT][XX];
  data[myindex + MRDPLOT_R_KNEE_Z] = s->knee[RIGHT][ZZ];

  data[myindex + MRDPLOT_L_FOOT_X] = s->foot[LEFT][XX];
  data[myindex + MRDPLOT_L_FOOT_Z] = s->foot[LEFT][ZZ];

  data[myindex + MRDPLOT_R_FOOT_X] = s->foot[RIGHT][XX];
  data[myindex + MRDPLOT_R_FOOT_Z] = s->foot[RIGHT][ZZ];

  data[myindex + MRDPLOT_L_FOOT_FX] = s->ground_force[LEFT][XX];
  data[myindex + MRDPLOT_L_FOOT_FZ] = s->ground_force[LEFT][ZZ];

  data[myindex + MRDPLOT_R_FOOT_FX] = s->ground_force[RIGHT][XX];
  data[myindex + MRDPLOT_R_FOOT_FZ] = s->ground_force[RIGHT][ZZ];

  data[myindex + MRDPLOT_STATE] = s->controller_state;
  data[myindex + MRDPLOT_STATE_TIME] = s->state_elapsed_time;
  data[myindex + MRDPLOT_TIME] = s->time;

  data[myindex + MRDPLOT_TORSO_ABS_ANGLE] = s->torso_abs_angle;

  data[myindex + MRDPLOT_L_THIGH_ABS_ANGLE] = s->thigh_abs_angle[LEFT];
  data[myindex + MRDPLOT_L_CALF_ABS_ANGLE] = s->calf_abs_angle[LEFT];

  data[myindex + MRDPLOT_R_THIGH_ABS_ANGLE] = s->thigh_abs_angle[RIGHT];
  data[myindex + MRDPLOT_R_CALF_ABS_ANGLE] = s->calf_abs_angle[RIGHT];

  data[myindex + MRDPLOT_L_HIP_COMMAND] = s->hip_command[LEFT];
  data[myindex + MRDPLOT_L_KNEE_COMMAND] = s->knee_command[LEFT];
  data[myindex + MRDPLOT_L_ANKLE_COMMAND] = s->ankle_command[LEFT];

  data[myindex + MRDPLOT_R_HIP_COMMAND] = s->hip_command[RIGHT];
  data[myindex + MRDPLOT_R_KNEE_COMMAND] = s->knee_command[RIGHT];
  data[myindex + MRDPLOT_R_ANKLE_COMMAND] = s->ankle_command[RIGHT];

  myindex += N_CHANNELS;
  n_points++;

  return 0;
}

/*****************************************************************************/

int write_matlab_file( char *filename )
{
  FILE *stream;
  int i;
  static int myindex = 0;

  stream = fopen( filename, "w" );
  if ( stream == NULL )
    {
      fprintf( stderr, "Can't open %s for write.\n", filename );
      exit( -1 );
    }
  for ( i = 0; i < n_points; i++ )
    {
      fprintf( stream, "%g ", data[myindex + MRDPLOT_TIME] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_ANKLE] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_KNEE] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_HIP] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_ANKLED] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_KNEED] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_HIPD] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_ANKLE_TORQUE] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_KNEE_TORQUE] );
      fprintf( stream, "%g ", data[myindex + MRDPLOT_L_HIP_TORQUE] );
      fprintf( stream, "\n" );
      myindex += N_CHANNELS;
    }
  fclose( stream );

  return 0;
}

/*****************************************************************************/

int write_the_mrdplot_file( SIM *s )
{
  MRDPLOT_DATA *d; 

  d = malloc_mrdplot_data( 0, 0 );
  d->filename = generate_file_name();
  d->n_channels = N_CHANNELS;
  d->n_points = n_points;
  d->total_n_numbers = d->n_channels*d->n_points;
  d->frequency = 1/s->time_step;
  d->data = data;
  d->names = names;
  d->units = units;

  write_mrdplot_file( d );

  // write_matlab_file( "dd" );

  return 0;
}

/*****************************************************************************/
