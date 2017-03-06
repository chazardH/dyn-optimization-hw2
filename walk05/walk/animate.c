/*****************************************************************************/
/*
  animate.c: animate a data file.
*/
/*****************************************************************************/
/* This version flips X and angles to get lighting right */
#define XFLIP -
#define XXFLIP 

/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "drawstuff.h" /* Graphics stuff */
#include "drawstuff-cga.h" /* CGA stuff to make things clearer */

#include "../useful/mrdplot/mrdplot.h"
#include "sdfast/b1g.h"
#include "main.h"
#include "main2.h"

/*****************************************************************************/

enum channels {
  TIME,
  HEAD_X,
  HEAD_Z,
  HIP_X,
  HIP_Z,
  L_KNEE_X,
  L_KNEE_Z,
  R_KNEE_X,
  R_KNEE_Z,
  L_FOOT_X,
  L_FOOT_Z,
  R_FOOT_X,
  R_FOOT_Z,
  TORSO_ABS_ANGLE,
  L_THIGH_ABS_ANGLE,
  R_THIGH_ABS_ANGLE,
  L_CALF_ABS_ANGLE,
  R_CALF_ABS_ANGLE,
  N_CHANNELS,
};

#define CALF_LENGTH (0.3810f) // length of link
#define CALF_WIDTH (0.05f)    // width of link

#define THIGH_LENGTH (0.3918f) // length of link
#define THIGH_WIDTH (0.05f)    // width of link

#define TORSO_LENGTH (0.8f)     // length of link
#define TORSO_WIDTH (0.05f)    // width of link

/*****************************************************************************/

SIM sim;

MRDPLOT_DATA *d;

int channels[N_CHANNELS];

/*****************************************************************************/
/*****************************************************************************/

double lookup_value( MRDPLOT_DATA *d, int point, int channel )
{
  if ( channels[channel] < 0 || channels[channel] >= d->n_channels )
    {
      fprintf( stderr, "Bad channel: %d %d\n", channel, channels[channel] );
      exit( -1 );
    }
  return d->data[point*d->n_channels + channels[channel]];
}

/********************************************************************/
// start(): set things up

static void start()
{
  // set up view point
  static float xyz[3] = {-4.0f,6.0f,0.8f};
  static float hpr[3] = {-90.0000f,0.0000f,0.0000f};
  dsSetViewpoint (xyz,hpr);
}

/********************************************************************/
// called when a key pressed

static void command (int cmd)
{
  // don't handle user input yet.
  dsPrint ("received command %d (`%c')\n",cmd,cmd);
}

/*****************************************************************************/

static void display (int pause)
{
  static int index = 0;
  float f;
  float center[3];
  float R[12];
  int i;
  float calf_sides[3] = { CALF_WIDTH, CALF_WIDTH, CALF_LENGTH };
  float thigh_sides[3] = { THIGH_WIDTH, THIGH_WIDTH, THIGH_LENGTH };
  float torso_sides[3] = { TORSO_WIDTH, TORSO_WIDTH, TORSO_LENGTH };

  /* Left Calf */
  center[XX] = XFLIP (lookup_value( d, index, L_KNEE_X )
		+ lookup_value( d, index, L_FOOT_X ))/2;
  center[YY] = 0;
  center[ZZ] = (lookup_value( d, index, L_KNEE_Z )
		+ lookup_value( d, index, L_FOOT_Z ))/2;

  for ( i = 0; i < 12; i++ )
    R[i] = 0;
  R[0] = cos( lookup_value( d, index, L_CALF_ABS_ANGLE ) );
  R[10] = R[0];
  R[2] = XXFLIP sin( lookup_value( d, index, L_CALF_ABS_ANGLE ) );
  R[8] = - R[2];

  dsSetTexture (DS_WOOD);
  dsSetColor (1,1,0);
  dsDrawBox( center, R, calf_sides );

  /* Right Calf */
  center[XX] = XFLIP (lookup_value( d, index, R_KNEE_X )
		+ lookup_value( d, index, R_FOOT_X ))/2;
  center[YY] = 0;
  center[ZZ] = (lookup_value( d, index, R_KNEE_Z )
		+ lookup_value( d, index, R_FOOT_Z ))/2;

  for ( i = 0; i < 12; i++ )
    R[i] = 0;
  R[0] = cos( lookup_value( d, index, R_CALF_ABS_ANGLE ) );
  R[10] = R[0];
  R[2] = XXFLIP sin( lookup_value( d, index, R_CALF_ABS_ANGLE ) );
  R[8] = - R[2];

  dsSetTexture (DS_WOOD);
  dsSetColor (1,0,1);
  dsDrawBox( center, R, calf_sides );

  /* Left Thigh */
  center[XX] = XFLIP (lookup_value( d, index, L_KNEE_X )
		+ lookup_value( d, index, HIP_X ))/2;
  center[YY] = 0;
  center[ZZ] = (lookup_value( d, index, L_KNEE_Z )
		+ lookup_value( d, index, HIP_Z ))/2;

  for ( i = 0; i < 12; i++ )
    R[i] = 0;
  R[0] = cos( lookup_value( d, index, L_THIGH_ABS_ANGLE ) );
  R[10] = R[0];
  R[2] = XXFLIP sin( lookup_value( d, index, L_THIGH_ABS_ANGLE ) );
  R[8] = - R[2];

  dsSetTexture (DS_WOOD);
  dsSetColor (0,0,1);
  dsDrawBox( center, R, thigh_sides );

  /* Right Thigh */
  center[XX] = XFLIP (lookup_value( d, index, R_KNEE_X )
		+ lookup_value( d, index, HIP_X ))/2;
  center[YY] = 0;
  center[ZZ] = (lookup_value( d, index, R_KNEE_Z )
		+ lookup_value( d, index, HIP_Z ))/2;

  for ( i = 0; i < 12; i++ )
    R[i] = 0;
  R[0] = cos( lookup_value( d, index, R_THIGH_ABS_ANGLE ) );
  R[10] = R[0];
  R[2] = XXFLIP sin( lookup_value( d, index, R_THIGH_ABS_ANGLE ) );
  R[8] = - R[2];

  dsSetTexture (DS_WOOD);
  dsSetColor (0,1,0);
  dsDrawBox( center, R, thigh_sides );

  /* Torso */
  center[XX] = XFLIP (lookup_value( d, index, HEAD_X )
		+ lookup_value( d, index, HIP_X ))/2;
  center[YY] = 0;
  center[ZZ] = (lookup_value( d, index, HEAD_Z )
		+ lookup_value( d, index, HIP_Z ))/2;

  for ( i = 0; i < 12; i++ )
    R[i] = 0;
  R[0] = cos( lookup_value( d, index, TORSO_ABS_ANGLE ) );
  R[10] = R[0];
  R[2] = XXFLIP sin( lookup_value( d, index, TORSO_ABS_ANGLE ) );
  R[8] = - R[2];

  dsSetTexture (DS_WOOD);
  dsSetColor (1,0,0);
  dsDrawBox( center, R, torso_sides );

  /* Delay */
#ifdef WIN32
  for( f = 0.0; f < 7000000.0; f += 1.0 )
    ;
#else
  for( f = 0.0; f < 12000000.0; f += 1.0 )
    ;
#endif

  index += 33;
  if ( index > d->n_points )
    index = 0;
}

/*****************************************************************************/
/********************************************************************/

int main (int argc, char **argv)
{
  char *filename;
  int i;
  dsFunctions fn; 

  if ( argc > 1 )
    filename = strdup( argv[1] );
  else
  //  filename = last_data();
  filename = "25n1s6trialsfailed";

  printf( "Reading: %s\n", filename );

  d = read_mrdplot( filename );

  for( i = 0; i < N_CHANNELS; i++ )
    channels[i] = -1;

  channels[TIME] = find_channel( "time", d );
  channels[HEAD_X] = find_channel( "head_x", d );
  channels[HEAD_Z] = find_channel( "head_z", d );
  channels[HIP_X] = find_channel( "hip_x", d );
  channels[HIP_Z] = find_channel( "hip_z", d );
  channels[L_KNEE_X] = find_channel( "l_knee_x", d );
  channels[L_KNEE_Z] = find_channel( "l_knee_z", d );
  channels[R_KNEE_X] = find_channel( "r_knee_x", d );
  channels[R_KNEE_Z] = find_channel( "r_knee_z", d );
  channels[L_FOOT_X] = find_channel( "l_foot_x", d );
  channels[L_FOOT_Z] = find_channel( "l_foot_z", d );
  channels[R_FOOT_X] = find_channel( "r_foot_x", d );
  channels[R_FOOT_Z] = find_channel( "r_foot_z", d );
  channels[TORSO_ABS_ANGLE] = find_channel( "torso_abs_angle", d );
  channels[L_THIGH_ABS_ANGLE] = find_channel( "l_thigh_abs_angle", d );
  channels[R_THIGH_ABS_ANGLE] = find_channel( "r_thigh_abs_angle", d );
  channels[L_CALF_ABS_ANGLE] = find_channel( "l_calf_abs_angle", d );
  channels[R_CALF_ABS_ANGLE] = find_channel( "r_calf_abs_angle", d );

  for( i = 0; i < N_CHANNELS; i++ )
    printf( "%d %d\n", i, channels[i] );

  //write to the file for plotting output
  int j;
  FILE *fp;
  fp = fopen("../plotoutput.txt", "w");
  //write out to a textfile for plotting in matlab
  for (j = 0; j < d->n_points; j++) {
	  for (i = 0; i < N_CHANNELS; i++)
		  fprintf(fp,"%f ", lookup_value(d, j, i));
	  fprintf(fp,"\n");
  }
  fclose(fp);

  // setup pointers to drawstuff callback functions
  fn.version = DS_VERSION; 
  fn.start = &start; 
  fn.step = &display; 
  fn.command = &command; 
  fn.stop = 0; 
#ifdef WIN32
  fn.path_to_textures = "C:/Users/ChrisH/Documents/cmu classes/dynamic optimization/hw2/walk05/useful/drawstuff-windows/textures";
#else
  fn.path_to_textures = WALK_BASE_PATH "/useful/drawstuff-linux/textures";
#endif

  // do display
  dsSimulationLoop( argc, argv, /* command line arguments */
                    1000, 200, /* window size */
                    &fn ); /* callback info */

  return 0;
}

/********************************************************************/
