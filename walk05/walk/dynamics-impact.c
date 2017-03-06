/************************************************************************/
/*
dynamics-impact.c: 
This is where the numerical integration and SDFAST stuff is done.
Use multiple models (in_air, single_support, double_support).
Use impact equations to analytically compute effect of impacts
and model transitions.
*/
/************************************************************************/
/*
In this version we are going to use the following conventions (slightly
different from the SDFAST code generated from sdfast/b1g.sd):
x forward
y left
z up

pitch is the torso angle relative to vertical
pitch + pitches the torso forward (head ahead of hips)

hip is the joint angle between the torso and the thigh
hip +
  moving the torso forward for a fixed vertical thigh, 
  moving the thigh forward for a fixed vertical torso.

knee is the joint angle between the thigh and the calf
knee +
  moving the thigh forward for a fixed vertical calf, 
  moving the calf forward for a fixed vertical thigh.

ankle is the angle between the calf and vertical
ankle +
  moving the calf forward for a fixed horizontal foot,
  moving the toe upward for a fixed vertical calf.
*/
/************************************************************************/
/* BUGS
We could try to change step size to find the exact time of impact.
*/
/************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "main2.h"
#include "sdfast/b1g.h"

/************************************************************************/
/* DEFINES */

#define FEET_TOGETHER (1e-3) // meters. Feet too close together to measure tilt

/* Dynamic model modes */
#define INITIALIZE 3
#define DOUBLE_SUPPORT 2
#define SINGLE_SUPPORT 1
#define IN_AIR 0

// Keep track of which body part to reference DS model to 
#define REF_HIP 0
#define REF_LFOOT 1
#define REF_RFOOT 2

#define EPSILON1 -1e-6 /* Fudge factor for ground level */

/* single support model */
#define SS_CTOL	1e-5	/* constraint tolerance */
#define SS_NJNT 20      /* overestimate of number of joints */
/* double support model */
#define DS_CTOL	1e-5	/* constraint tolerance */
#define DS_NJNT 20      /* overestimate of number of joints */

#ifdef WIN32
int isnan( double x )
{ return 0; }

int isinf( double x )
{ return 0; }
#endif

/************************************************************************/
/* GLOBAL VARIABLES */

SIM sim; /* The simulation structure */

int interactive = 0;
int impact1_print = 0;
int impact2_print = 0;

/************************************************************************/
/************************************************************************/
/************************************************************************/

int other_side( int value )
{
  if ( value == LEFT )
    return RIGHT;
  else
    return LEFT;
}

/************************************************************************/
/* Make up for the fact that the ss and ds models have different
   coordinates, and adjust x */

fix_positions( SIM *s, double v1[3], double v2[3] )
{
  v2[0] = v1[0] + s->x_offset;
  v2[1] = v1[2];
  v2[2] = v1[1];
}

/************************************************************************/
/* Make up for the fact that the ss and ds models have different
   coordinates. */

fix_yz( SIM *s, double v1[3], double v2[3] )
{
  v2[0] = v1[0];
  v2[1] = v1[2];
  v2[2] = v1[1];
}

/************************************************************************/

void debug_positions( SIM *s )
{

  printf( "head: %g %g %g\n", s->head[XX], s->head[YY], s->head[ZZ] );
  printf( "torso: %g %g %g\n", s->torso[XX], s->torso[YY], s->torso[ZZ] );
  printf( "hip: %g %g %g\n", s->hip[XX], s->hip[YY], s->hip[ZZ] );
  printf( "lthigh: %g %g %g\n", s->thigh[LEFT][XX], s->thigh[LEFT][YY],
	  s->thigh[LEFT][ZZ] );
  printf( "lknee: %g %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][YY],
	  s->knee[LEFT][ZZ] );
  printf( "lcalf: %g %g %g\n", s->calf[LEFT][XX], s->calf[LEFT][YY],
	  s->calf[LEFT][ZZ] );
  printf( "lfoot: %g %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][YY],
	  s->foot[LEFT][ZZ] );
  printf( "rthigh: %g %g %g\n", s->thigh[RIGHT][XX], s->thigh[RIGHT][YY],
	  s->thigh[RIGHT][ZZ] );
  printf( "rknee: %g %g %g\n", s->knee[RIGHT][XX], s->knee[RIGHT][YY],
	  s->knee[RIGHT][ZZ] );
  printf( "rcalf: %g %g %g\n", s->calf[RIGHT][XX], s->calf[RIGHT][YY],
	  s->calf[RIGHT][ZZ] );
  printf( "rfoot: %g %g %g\n", s->foot[RIGHT][XX], s->foot[RIGHT][YY],
	  s->foot[RIGHT][ZZ] );

  printf( "pitch: %g\n", s->pitch );
  printf( "hips: %g %g\n", s->hip_angle[LEFT], s->hip_angle[RIGHT] );
  printf( "knees: %g %g\n", s->knee_angle[LEFT], s->knee_angle[RIGHT] );
  printf( "ankles: %g %g\n", s->ankle_angle[LEFT], s->ankle_angle[RIGHT] );
}

/************************************************************************/

init_in_air_model( SIM *s )
{
  int i, j;
  double vector[3];
  double inertia[3][3];

  b1gmass( BODY_TORSO, s->torso_mass );
  b1gmass( BODY_L_THIGH, s->thigh_mass );
  b1gmass( BODY_R_THIGH, s->thigh_mass );
  b1gmass( BODY_L_CALF, s->calf_mass );
  b1gmass( BODY_R_CALF, s->calf_mass );

  for ( i = 0; i < 3; i++ )
    {
      vector[i] = 0;
      for ( j = 0; j < 3; j++ )
	{
	  if ( i == j )
	    inertia[i][i] = 1.0;
	  else
	    inertia[i][j] = 0;
	}
    }
  inertia[YY][YY] = s->torso_I;
  b1giner( BODY_TORSO, inertia );
  inertia[YY][YY] = s->thigh_I;
  b1giner( BODY_L_THIGH, inertia );
  b1giner( BODY_R_THIGH, inertia );
  inertia[YY][YY] = s->calf_I;
  b1giner( BODY_L_CALF, inertia );
  b1giner( BODY_R_CALF, inertia );

  vector[ZZ] = -s->torso_cm;
  b1gbtj( BODY_TORSO, vector );
  vector[ZZ] = s->thigh_cm;
  b1gbtj( BODY_L_THIGH, vector );
  b1gbtj( BODY_R_THIGH, vector );
  vector[ZZ] = s->calf_cm;
  b1gbtj( BODY_L_CALF, vector );
  b1gbtj( BODY_R_CALF, vector );

  vector[ZZ] = -s->torso_cm;
  b1gitj( BODY_L_THIGH, vector );
  b1gitj( BODY_R_THIGH, vector );

  vector[ZZ] = -(s->thigh_length - s->thigh_cm);
  b1gitj( BODY_L_CALF, vector );
  b1gitj( BODY_R_CALF, vector );

  b1ginit(); /* initialize SDFAST model */
  b1gstab( 2.0*s->sdfast_baumgarte, s->sdfast_baumgarte*s->sdfast_baumgarte ); 
  for( i = 0; i < NSTATE; i++ )
    s->sdfast_state[i] = 0;
  b1gprinterr(stderr);
}

/************************************************************************/

void in_air_forward_kinematics( SIM *s )
{
  int i;
  double zero_offset[3] = { 0.0, 0.0, 0.0 };

  b1gpos( BODY_TORSO, s->head_offset, s->head );    
  b1gpos( BODY_TORSO, s->hip_offset, s->hip );    
  b1gpos( BODY_L_CALF, s->knee_offset, &(s->knee[LEFT][0]) );    
  b1gpos( BODY_R_CALF, s->knee_offset, &(s->knee[RIGHT][0]) );    
  b1gpos( BODY_L_CALF, s->foot_offset, &(s->foot[LEFT][0]) );    
  b1gpos( BODY_R_CALF, s->foot_offset, &(s->foot[RIGHT][0]) );    

  s->head[ZZ] += s->thigh_length + s->calf_length;
  s->hip[ZZ] += s->thigh_length + s->calf_length;
  s->knee[LEFT][ZZ] += s->thigh_length + s->calf_length;
  s->knee[RIGHT][ZZ] += s->thigh_length + s->calf_length;
  s->foot[LEFT][ZZ] += s->thigh_length + s->calf_length;
  s->foot[RIGHT][ZZ] += s->thigh_length + s->calf_length;

  /*
  printf( "head: %g\n", s->head[ZZ] );
  printf( "hip: %g\n", s->hip[ZZ] );
  printf( "kneeL: %g\n", s->knee[LEFT][ZZ] );
  printf( "footL: %g\n", s->foot[LEFT][ZZ] );
  */

  b1gpos( BODY_TORSO, zero_offset, s->torso );
  b1gpos( BODY_L_THIGH, zero_offset, &(s->thigh[LEFT][0]) );
  b1gpos( BODY_R_THIGH, zero_offset, &(s->thigh[RIGHT][0]) );
  b1gpos( BODY_L_CALF, zero_offset, &(s->calf[LEFT][0]) );
  b1gpos( BODY_R_CALF, zero_offset, &(s->calf[RIGHT][0]) );

  s->torso[ZZ] += s->thigh_length + s->calf_length;
  s->thigh[LEFT][ZZ] += s->thigh_length + s->calf_length;
  s->thigh[RIGHT][ZZ] += s->thigh_length + s->calf_length;
  s->calf[LEFT][ZZ] += s->thigh_length + s->calf_length;
  s->calf[RIGHT][ZZ] += s->thigh_length + s->calf_length;

  b1gvel( BODY_TORSO, s->head_offset, s->headd );    
  b1gvel( BODY_TORSO, s->hip_offset, s->hipd );    
  b1gvel( BODY_L_CALF, s->knee_offset, &(s->kneed[LEFT][0]) );    
  b1gvel( BODY_R_CALF, s->knee_offset, &(s->kneed[RIGHT][0]) );    
  b1gvel( BODY_L_CALF, s->foot_offset, &(s->footd[LEFT][0]) );    
  b1gvel( BODY_R_CALF, s->foot_offset, &(s->footd[RIGHT][0]) );    

  b1gvel( BODY_TORSO, zero_offset, s->torsod );
  b1gvel( BODY_L_THIGH, zero_offset, &(s->thighd[LEFT][0]) );
  b1gvel( BODY_R_THIGH, zero_offset, &(s->thighd[RIGHT][0]) );
  b1gvel( BODY_L_CALF, zero_offset, &(s->calfd[LEFT][0]) );
  b1gvel( BODY_R_CALF, zero_offset, &(s->calfd[RIGHT][0]) );

  s->pitch = s->sdfast_state[Q_PITCH];
  s->hip_angle[LEFT] = -s->sdfast_state[Q_L_HIP];
  s->hip_angle[RIGHT] = -s->sdfast_state[Q_R_HIP];
  s->knee_angle[LEFT] = -s->sdfast_state[Q_L_KNEE];
  s->knee_angle[RIGHT] = -s->sdfast_state[Q_R_KNEE];

  s->pitchd = s->sdfast_state[QD_PITCH];
  s->hip_angled[LEFT] = -s->sdfast_state[QD_L_HIP];
  s->hip_angled[RIGHT] = -s->sdfast_state[QD_R_HIP];
  s->knee_angled[LEFT] = -s->sdfast_state[QD_L_KNEE];
  s->knee_angled[RIGHT] = -s->sdfast_state[QD_R_KNEE];

  // Since the ankle angles don't exist in the sdfast model, we have to
  // compute them.
  s->ankle_angle[LEFT] = s->pitch 
    - s->hip_angle[LEFT] - s->knee_angle[LEFT];
  s->ankle_angle[RIGHT] = s->pitch 
    - s->hip_angle[RIGHT] - s->knee_angle[RIGHT];
  s->ankle_angled[LEFT] = s->pitchd 
    - s->hip_angled[LEFT] - s->knee_angled[LEFT];
  s->ankle_angled[RIGHT] = s->pitchd 
    - s->hip_angled[RIGHT] - s->knee_angled[RIGHT];

  s->torso_abs_angle = 
    -atan2( s->head[XX] - s->hip[XX], s->head[ZZ] - s->hip[ZZ] );
  for( i = 0; i < 2; i++ )
    {
      s->thigh_abs_angle[i] = -atan2( s->hip[XX] - s->knee[i][XX], 
				      s->hip[ZZ] - s->knee[i][ZZ] );
      s->calf_abs_angle[i] = -atan2( s->knee[i][XX] - s->foot[i][XX], 
				     s->knee[i][ZZ] - s->foot[i][ZZ] );
    }

}

/************************************************************************/
/* Initialize the state vector */

void init_state_in_air( SIM *s )
{
  int i, j;

  s->sdfast_state[Q_X] = s->hip[XX];
  s->sdfast_state[Q_Z] = s->hip[ZZ];
  s->sdfast_state[Q_PITCH] = s->pitch;
  s->sdfast_state[Q_L_HIP] = -s->hip_angle[LEFT];
  s->sdfast_state[Q_R_HIP] = -s->hip_angle[RIGHT];
  s->sdfast_state[Q_L_KNEE] = -s->knee_angle[LEFT];
  s->sdfast_state[Q_R_KNEE] = -s->knee_angle[RIGHT];

  s->sdfast_state[QD_X] = s->hipd[XX];
  s->sdfast_state[QD_Z] = s->hipd[ZZ];
  s->sdfast_state[QD_PITCH] = s->pitchd;
  s->sdfast_state[QD_L_HIP] = -s->hip_angled[LEFT];
  s->sdfast_state[QD_R_HIP] = -s->hip_angled[RIGHT];
  s->sdfast_state[QD_L_KNEE] = -s->knee_angled[LEFT];
  s->sdfast_state[QD_R_KNEE] = -s->knee_angled[RIGHT];

  for( i = 0; i < NSTATE; i++ )
    {
      s->sdfast_stated[i] = 0;
    }

  for( i = 0, j = NQ; i < NU; i++, j++ )
    { 
      s->sdfast_stated[i] = s->sdfast_state[j];
    }

  s->sdfast_model = IN_AIR;
  b1gstate( 0.0, s->sdfast_state, s->sdfast_stated );
  s->sdfast_flag = 1;
  s->status = OK;

  in_air_forward_kinematics( s );

  b1gprinterr(stderr);
}

/************************************************************************/

init_single_support_model( SIM *s )
{
  int i, j;
  double vector[3];
  double inertia[3][3];
  static int firsttime = 0;

  b1ssmass( SS_TORSO_ID, s->torso_mass );
  b1ssmass( SS_THIGH1_ID, s->thigh_mass );
  b1ssmass( SS_THIGH2_ID, s->thigh_mass );
  b1ssmass( SS_CALF1_ID, s->calf_mass );
  b1ssmass( SS_CALF2_ID, s->calf_mass );

  for ( i = 0; i < 3; i++ )
    {
      vector[i] = 0;
      for ( j = 0; j < 3; j++ )
	{
	  if ( i == j )
	    inertia[i][i] = 1.0;
	  else
	    inertia[i][j] = 0;
	}
    }
  inertia[ZZ][ZZ] = s->torso_I;
  b1ssiner( SS_TORSO_ID, inertia );
  inertia[ZZ][ZZ] = s->thigh_I;
  b1ssiner( SS_THIGH1_ID, inertia );
  b1ssiner( SS_THIGH2_ID, inertia );
  inertia[ZZ][ZZ] = s->calf_I;
  b1ssiner( SS_CALF1_ID, inertia );
  b1ssiner( SS_CALF2_ID, inertia );


  if ( firsttime )
    {
      for( i = 0; i < 5; i++ )
	{
	  b1ssgetbtj( i, vector );
	  printf( "ss r btj %d: %g %g %g\n", i, vector[0],
		  vector[1], vector[2] );
	}
    }

  vector[YY] = -(s->calf_length - s->calf_cm);
  b1ssbtj( SS_CALF1_ID, vector );
  vector[YY] = -(s->thigh_length - s->thigh_cm);
  b1ssbtj( SS_THIGH1_ID, vector );
  vector[YY] = -s->torso_cm;
  b1ssbtj( SS_TORSO_ID, vector );
  vector[YY] = s->thigh_cm;
  b1ssbtj( SS_THIGH2_ID, vector );
  vector[YY] = s->calf_cm;
  b1ssbtj( SS_CALF2_ID, vector );

  if ( firsttime )
    {
      for( i = 0; i < 5; i++ )
	{
	  b1ssgetbtj( i, vector );
	  printf( "ss r btj %d: %g %g %g\n", i, vector[0],
		  vector[1], vector[2] );
	}
    }

  if ( firsttime )
    {
      for( i = 0; i < 5; i++ )
	{
	  b1ssgetitj( i, vector );
	  printf( "ss r itj %d: %g %g %g\n", i, vector[0],
		  vector[1], vector[2] );
	}
    }

  vector[YY] = s->calf_cm;
  b1ssitj( SS_THIGH1_ID, vector );
  vector[YY] = s->thigh_cm;
  b1ssitj( SS_TORSO_ID, vector );
  vector[YY] = -s->torso_cm;
  b1ssitj( SS_THIGH2_ID, vector );
  vector[YY] = -(s->thigh_length - s->thigh_cm);
  b1ssitj( SS_CALF2_ID, vector );

  if ( firsttime )
    {
      for( i = 0; i < 5; i++ )
	{
	  b1ssgetitj( i, vector );
	  printf( "ss itj %d: %g %g %g\n", i, vector[0],
		  vector[1], vector[2] );
	}
      printf( "press return to continue.\n" );
      getchar();
    }

  firsttime = 0;

  b1ssinit(); /* initialize SDFAST model */
  b1ssprinterr(stderr);
}

/************************************************************************/
/* Single support: convert sdfast to sim coordinates. */

void ss_forward_kinematics( SIM *s )
{
  double foot1[3];
  double knee1[3];
  double hip[3];
  double head[3];
  double knee2[3];
  double foot2[3];
  double offset[3];
  double foot1d[3];
  double knee1d[3];
  double hipd[3];
  double headd[3];
  double knee2d[3];
  double foot2d[3];
  double calf1[3];
  double thigh1[3];
  double torso[3];
  double thigh2[3];
  double calf2[3];
  double calf1d[3];
  double thigh1d[3];
  double torsod[3];
  double thigh2d[3];
  double calf2d[3];
  int i;

  foot1[XX] = 0;
  foot1[YY] = 0;
  foot1[ZZ] = 0;
  foot1d[XX] = 0;
  foot1d[YY] = 0;
  foot1d[ZZ] = 0;
  offset[XX] = 0;
  offset[YY] = s->knee_offset[ZZ];
  offset[ZZ] = 0;
  b1sspos( SS_CALF1_ID, offset, knee1 );
  b1ssvel( SS_CALF1_ID, offset, knee1d );
  offset[YY] = s->hip_offset[ZZ];
  b1sspos( SS_TORSO_ID, offset, hip );
  b1ssvel( SS_TORSO_ID, offset, hipd );
  offset[YY] = s->head_offset[ZZ];
  b1sspos( SS_TORSO_ID, offset, head );
  b1ssvel( SS_TORSO_ID, offset, headd );
  offset[YY] = s->knee_offset[ZZ];
  b1sspos( SS_CALF2_ID, offset, knee2 );
  b1ssvel( SS_CALF2_ID, offset, knee2d );
  offset[YY] = s->foot_offset[ZZ];
  b1sspos( SS_CALF2_ID, offset, foot2 );
  b1ssvel( SS_CALF2_ID, offset, foot2d ); 
  offset[XX] = 0;
  offset[YY] = 0;
  offset[ZZ] = 0;
  b1sspos( SS_CALF1_ID, offset, calf1 );
  b1ssvel( SS_CALF1_ID, offset, calf1d );
  b1sspos( SS_THIGH1_ID, offset, thigh1 );
  b1ssvel( SS_THIGH1_ID, offset, thigh1d );
  b1sspos( SS_TORSO_ID, offset, torso );
  b1ssvel( SS_TORSO_ID, offset, torsod );
  b1sspos( SS_THIGH2_ID, offset, thigh2 );
  b1ssvel( SS_THIGH2_ID, offset, thigh2d );
  b1sspos( SS_CALF2_ID, offset, calf2 );
  b1ssvel( SS_CALF2_ID, offset, calf2d );

  fix_positions( s, head, s->head );
  fix_yz( s, headd, s->headd );
  fix_positions( s, hip, s->hip );
  fix_yz( s, hipd, s->hipd );
  fix_positions( s, torso, s->torso );
  fix_yz( s, torsod, s->torsod );

  if ( s->ss_foot_down == LEFT )
    {
      fix_positions( s, knee1, s->knee[LEFT] );
      fix_yz( s, knee1d, s->kneed[LEFT] );
      fix_positions( s, foot1, s->foot[LEFT] );
      fix_yz( s, foot1d, s->footd[LEFT] );
      fix_positions( s, knee2, s->knee[RIGHT] );
      fix_yz( s, knee2d, s->kneed[RIGHT] );
      fix_positions( s, foot2, s->foot[RIGHT] );
      fix_yz( s, foot2d, s->footd[RIGHT] );
      fix_positions( s, thigh1, s->thigh[LEFT] );
      fix_yz( s, thigh1d, s->thighd[LEFT] );
      fix_positions( s, thigh2, s->thigh[RIGHT] );
      fix_yz( s, thigh2d, s->thighd[RIGHT] );
      fix_positions( s, calf1, s->calf[LEFT] );
      fix_yz( s, calf1d, s->calfd[LEFT] );
      fix_positions( s, calf2, s->calf[RIGHT] );
      fix_yz( s, calf2d, s->calfd[RIGHT] );

      s->pitch = -(s->ss_state[SS_ANKLE1] + s->ss_state[SS_KNEE1]
		   + s->ss_state[SS_HIP1]);
      s->knee_angle[LEFT] = -s->ss_state[SS_KNEE1];
      s->hip_angle[LEFT] = -s->ss_state[SS_HIP1];
      s->hip_angle[RIGHT] = s->ss_state[SS_HIP2];
      s->knee_angle[RIGHT] = s->ss_state[SS_KNEE2];
      s->pitchd = -(s->ss_state[SS_ANKLE1D] + s->ss_state[SS_KNEE1D]
		    + s->ss_state[SS_HIP1D]);
      s->knee_angled[LEFT] = -s->ss_state[SS_KNEE1D];
      s->hip_angled[LEFT] = -s->ss_state[SS_HIP1D];
      s->hip_angled[RIGHT] = s->ss_state[SS_HIP2D];
      s->knee_angled[RIGHT] = s->ss_state[SS_KNEE2D];
    }
  else
    {
      fix_positions( s, knee1, s->knee[RIGHT] );
      fix_yz( s, knee1d, s->kneed[RIGHT] );
      fix_positions( s, foot1, s->foot[RIGHT] );
      fix_yz( s, foot1d, s->footd[RIGHT] );
      fix_positions( s, knee2, s->knee[LEFT] );
      fix_yz( s, knee2d, s->kneed[LEFT] );
      fix_positions( s, foot2, s->foot[LEFT] );
      fix_yz( s, foot2d, s->footd[LEFT] );
      fix_positions( s, thigh1, s->thigh[RIGHT] );
      fix_yz( s, thigh1d, s->thighd[RIGHT] );
      fix_positions( s, thigh2, s->thigh[LEFT] );
      fix_yz( s, thigh2d, s->thighd[LEFT] );
      fix_positions( s, calf1, s->calf[RIGHT] );
      fix_yz( s, calf1d, s->calfd[RIGHT] );
      fix_positions( s, calf2, s->calf[LEFT] );
      fix_yz( s, calf2d, s->calfd[LEFT] );

      s->pitch = -(s->ss_state[SS_ANKLE1] + s->ss_state[SS_KNEE1]
		   + s->ss_state[SS_HIP1]);
      s->knee_angle[RIGHT] = -s->ss_state[SS_KNEE1];
      s->hip_angle[RIGHT] = -s->ss_state[SS_HIP1];
      s->hip_angle[LEFT] = s->ss_state[SS_HIP2];
      s->knee_angle[LEFT] = s->ss_state[SS_KNEE2];
      s->pitchd = -(s->ss_state[SS_ANKLE1D] + s->ss_state[SS_KNEE1D]
		    + s->ss_state[SS_HIP1D]);
      s->knee_angled[RIGHT] = -s->ss_state[SS_KNEE1D];
      s->hip_angled[RIGHT] = -s->ss_state[SS_HIP1D];
      s->hip_angled[LEFT] = s->ss_state[SS_HIP2D];
      s->knee_angled[LEFT] = s->ss_state[SS_KNEE2D];
    }

  s->ankle_angle[LEFT] = s->pitch 
    - s->hip_angle[LEFT] - s->knee_angle[LEFT];
  s->ankle_angle[RIGHT] = s->pitch 
    - s->hip_angle[RIGHT] - s->knee_angle[RIGHT];
  s->ankle_angled[LEFT] = s->pitchd 
    - s->hip_angled[LEFT] - s->knee_angled[LEFT];
  s->ankle_angled[RIGHT] = s->pitchd 
    - s->hip_angled[RIGHT] - s->knee_angled[RIGHT];

  /*
  printf( "fk: %g: %g %g %g\n",
	  s->ankle_angle[RIGHT], s->pitch, s->hip_angle[RIGHT],
	  s->knee_angle[RIGHT] );
  */

  s->torso_abs_angle = 
    -atan2( s->head[XX] - s->hip[XX], s->head[ZZ] - s->hip[ZZ] );
  for( i = 0; i < 2; i++ )
    {
      s->thigh_abs_angle[i] = -atan2( s->hip[XX] - s->knee[i][XX],
				      s->hip[ZZ] - s->knee[i][ZZ] );
      s->calf_abs_angle[i] = -atan2( s->knee[i][XX] - s->foot[i][XX],
				     s->knee[i][ZZ] - s->foot[i][ZZ] );
    }
}

/************************************************************************/

void init_state_one_foot_on_ground( SIM *s )
{
  double min;
  int i, j;
  int which_foot_down = LEFT;

  /* Which foot is on the ground? */
  init_state_in_air( s );
  if (s->foot[LEFT][ZZ] <= s->foot[RIGHT][ZZ] )
    which_foot_down = LEFT;
  else
    which_foot_down = RIGHT;

  s->sdfast_flag = 1;
  s->sdfast_model = SINGLE_SUPPORT;

  for ( i = 0; i < SS_NSTATE; i++ )
    s->ss_state[i] = 0;

  s->ss_state[SS_ANKLE1] = (-s->pitch + s->hip_angle[which_foot_down]
			    + s->knee_angle[which_foot_down]);
  s->ss_state[SS_KNEE1] = -s->knee_angle[which_foot_down];
  s->ss_state[SS_HIP1] = -s->hip_angle[which_foot_down];
  s->ss_state[SS_HIP2] = s->hip_angle[other_side(which_foot_down)];
  s->ss_state[SS_KNEE2] = s->knee_angle[other_side(which_foot_down)];

  s->ss_state[SS_ANKLE1D] = (-s->pitchd + s->hip_angled[which_foot_down]
			     + s->knee_angled[which_foot_down]);
  s->ss_state[SS_KNEE1D] = -s->knee_angled[which_foot_down];
  s->ss_state[SS_HIP1D] = -s->hip_angled[which_foot_down];
  s->ss_state[SS_HIP2D] = s->hip_angled[other_side(which_foot_down)];
  s->ss_state[SS_KNEE2D] = s->knee_angled[other_side(which_foot_down)];

  s->ss_foot_down = which_foot_down;

  for( i = 0; i < NSTATE; i++ )
    {
      s->ss_stated[i] = 0;
    }

  for( i = 0, j = NQ; i < NU; i++, j++ )
    { 
      s->ss_stated[i] = s->ss_state[j];
    }
  b1ssstate( 0.0, s->ss_state, s->ss_stated );
  s->sdfast_flag = 1;
  s->status = OK;
  b1ssprinterr(stderr);

  ss_forward_kinematics( s );
}

/************************************************************************/

init_double_support_model( SIM *s )
{
  int i, j;
  double vector[3];
  double inertia[3][3];
  static int firsttime = 0;
  double inbtojoint[3];

  b1dsmass( DS_TORSO_ID, s->torso_mass );
  b1dsmass( DS_THIGH1_ID, s->thigh_mass );
  b1dsmass( DS_THIGH2_ID, s->thigh_mass );
  b1dsmass( DS_CALF1_ID, s->calf_mass );
  b1dsmass( DS_CALF2_ID, s->calf_mass );

  for ( i = 0; i < 3; i++ )
    {
      vector[i] = 0;
      for ( j = 0; j < 3; j++ )
	{
	  if ( i == j )
	    inertia[i][i] = 1.0;
	  else
	    inertia[i][j] = 0;
	}
    }
  inertia[ZZ][ZZ] = s->torso_I;
  b1dsiner( DS_TORSO_ID, inertia );
  inertia[ZZ][ZZ] = s->thigh_I;
  b1dsiner( DS_THIGH1_ID, inertia );
  b1dsiner( DS_THIGH2_ID, inertia );
  inertia[ZZ][ZZ] = s->calf_I;
  b1dsiner( DS_CALF1_ID, inertia );
  b1dsiner( DS_CALF2_ID, inertia );

  if ( firsttime )
    {
      for( i = 0; i <= 5; i++ )
	{
	  b1dsgetbtj( i, vector );
	  printf( "ds btj %d: %g %g %g\n", i, vector[0],
		  vector[1], vector[2] );
	}
    }

  vector[YY] = -(s->calf_length - s->calf_cm);
  b1dsbtj( DS_CALF1_ID, vector );
  vector[YY] = -(s->thigh_length - s->thigh_cm);
  b1dsbtj( DS_THIGH1_ID, vector );
  vector[YY] = -s->torso_cm;
  b1dsbtj( DS_TORSO_ID, vector );
  vector[YY] = s->thigh_cm;
  b1dsbtj( DS_THIGH2_ID, vector );
  vector[YY] = s->calf_cm;
  b1dsbtj( DS_CALF2_ID, vector );
  vector[YY] = -(s->calf_length - s->calf_cm);
  b1dsbtj( DS_LOOP_ID, vector );

  if ( firsttime )
    {
      for( i = 0; i <=5; i++ )
	{
	  b1dsgetbtj( i, vector );
	  printf( "ds btj %d: %g %g %g\n", i, vector[0],
		  vector[1], vector[2] );
	}
    }

  if ( firsttime )
    {
      for( i = 0; i <=5; i++ )
	{
	  b1dsgetitj( i, vector );
	  printf( "ds itj %d: %g %g %g\n", i, vector[0],
		  vector[1], vector[2] );
	}
    }

  vector[YY] = s->calf_cm;
  b1dsitj( DS_THIGH1_ID, vector );
  vector[YY] = s->thigh_cm;
  b1dsitj( DS_TORSO_ID, vector );
  vector[YY] = -s->torso_cm;
  b1dsitj( DS_THIGH2_ID, vector );
  vector[YY] = -(s->thigh_length - s->thigh_cm);
  b1dsitj( DS_CALF2_ID, vector );

  if ( firsttime )
    {
      for( i = 0; i <=5; i++ )
	{
	  b1dsgetitj( i, vector );
	  printf( "%d: %g %g %g\n", i, vector[0],
		  vector[1], vector[2] );
	}
      printf( "press return to continue.\n" );
      getchar();
    }

  firsttime = 0;

  /* We have to initialize this model again when we know where
     the 2nd foot is. */

  inbtojoint[0] = 0;
  inbtojoint[1] = 0;
  inbtojoint[2] = 0;
  b1dsitj( DS_LOOP_ID, inbtojoint );
  b1dsprinterr(stderr);
  b1dsinit(); 
  b1dsprinterr(stderr);
}

/************************************************************************/
/* Double support: convert sdfast to sim coordinates. */

void ds_forward_kinematics( SIM *s )
{
  double foot1[3];
  double knee1[3];
  double hip[3];
  double head[3];
  double knee2[3];
  double foot2[3];
  double offset[3];
  double foot1d[3];
  double knee1d[3];
  double hipd[3];
  double headd[3];
  double knee2d[3];
  double foot2d[3];
  double calf1[3];
  double thigh1[3];
  double torso[3];
  double thigh2[3];
  double calf2[3];
  double calf1d[3];
  double thigh1d[3];
  double torsod[3];
  double thigh2d[3];
  double calf2d[3];
  int i;

  foot1[XX] = 0;
  foot1[YY] = 0;
  foot1[ZZ] = 0;
  foot1d[XX] = 0;
  foot1d[YY] = 0;
  foot1d[ZZ] = 0;
  offset[XX] = 0;
  offset[YY] = s->knee_offset[ZZ];
  offset[ZZ] = 0;
  b1dspos( DS_CALF1_ID, offset, knee1 );
  b1dsvel( DS_CALF1_ID, offset, knee1d );
  offset[YY] = s->hip_offset[ZZ];
  b1dspos( DS_TORSO_ID, offset, hip );
  b1dsvel( DS_TORSO_ID, offset, hipd );
  offset[YY] = s->head_offset[ZZ];
  b1dspos( DS_TORSO_ID, offset, head );
  b1dsvel( DS_TORSO_ID, offset, headd );
  offset[YY] = s->knee_offset[ZZ];
  b1dspos( DS_CALF2_ID, offset, knee2 );
  b1dsvel( DS_CALF2_ID, offset, knee2d );
  offset[YY] = s->foot_offset[ZZ];
  b1dspos( DS_CALF2_ID, offset, foot2 );
  b1dsvel( DS_CALF2_ID, offset, foot2d ); 
  offset[XX] = 0;
  offset[YY] = 0;
  offset[ZZ] = 0;
  b1dspos( DS_CALF1_ID, offset, calf1 );
  b1dsvel( DS_CALF1_ID, offset, calf1d );
  b1dspos( DS_THIGH1_ID, offset, thigh1 );
  b1dsvel( DS_THIGH1_ID, offset, thigh1d );
  b1dspos( DS_TORSO_ID, offset, torso );
  b1dsvel( DS_TORSO_ID, offset, torsod );
  b1dspos( DS_THIGH2_ID, offset, thigh2 );
  b1dsvel( DS_THIGH2_ID, offset, thigh2d );
  b1dspos( DS_CALF2_ID, offset, calf2 );
  b1dsvel( DS_CALF2_ID, offset, calf2d );

  /* Hack to keep foot2 above ground. */
  if ( foot2[YY] < 0 )
    foot2[YY] = 0;

  fix_positions( s, head, s->head );
  fix_yz( s, headd, s->headd );
  fix_positions( s, hip, s->hip );
  fix_yz( s, hipd, s->hipd );
  fix_positions( s, knee1, s->knee[LEFT] );
  fix_yz( s, knee1d, s->kneed[LEFT] );
  fix_positions( s, foot1, s->foot[LEFT] );
  fix_yz( s, foot1d, s->footd[LEFT] );
  fix_positions( s, knee2, s->knee[RIGHT] );
  fix_yz( s, knee2d, s->kneed[RIGHT] );
  fix_positions( s, foot2, s->foot[RIGHT] );
  fix_yz( s, foot2d, s->footd[RIGHT] );
  fix_positions( s, torso, s->torso );
  fix_yz( s, torsod, s->torsod );
  fix_positions( s, thigh1, s->thigh[LEFT] );
  fix_yz( s, thigh1d, s->thighd[LEFT] );
  fix_positions( s, thigh2, s->thigh[RIGHT] );
  fix_yz( s, thigh2d, s->thighd[RIGHT] );
  fix_positions( s, calf1, s->calf[LEFT] );
  fix_yz( s, calf1d, s->calfd[LEFT] );
  fix_positions( s, calf2, s->calf[RIGHT] );
  fix_yz( s, calf2d, s->calfd[RIGHT] );

  s->pitch = -(s->ds_state[DS_ANKLE1] + s->ds_state[DS_KNEE1]
	       + s->ds_state[DS_HIP1]);
  s->knee_angle[LEFT] = -s->ds_state[DS_KNEE1];
  s->hip_angle[LEFT] = -s->ds_state[DS_HIP1];
  s->hip_angle[RIGHT] = s->ds_state[DS_HIP2];
  s->knee_angle[RIGHT] = s->ds_state[DS_KNEE2];
  s->pitchd = -(s->ds_state[DS_ANKLE1D] + s->ds_state[DS_KNEE1D]
		+ s->ds_state[DS_HIP1D]);
  s->knee_angled[LEFT] = -s->ds_state[DS_KNEE1D];
  s->hip_angled[LEFT] = -s->ds_state[DS_HIP1D];
  s->hip_angled[RIGHT] = s->ds_state[DS_HIP2D];
  s->knee_angled[RIGHT] = s->ds_state[DS_KNEE2D];

  s->ankle_angle[LEFT] = s->pitch 
    - s->hip_angle[LEFT] - s->knee_angle[LEFT];
  s->ankle_angle[RIGHT] = s->pitch 
    - s->hip_angle[RIGHT] - s->knee_angle[RIGHT];
  s->ankle_angled[LEFT] = s->pitchd 
    - s->hip_angled[LEFT] - s->knee_angled[LEFT];
  s->ankle_angled[RIGHT] = s->pitchd 
    - s->hip_angled[RIGHT] - s->knee_angled[RIGHT];

  s->torso_abs_angle = 
    -atan2( s->head[XX] - s->hip[XX], s->head[ZZ] - s->hip[ZZ] );
  for( i = 0; i < 2; i++ )
    {
      s->thigh_abs_angle[i] = -atan2( s->hip[XX] - s->knee[i][XX],
				      s->hip[ZZ] - s->knee[i][ZZ] );
      s->calf_abs_angle[i] = -atan2( s->knee[i][XX] - s->foot[i][XX],
				     s->knee[i][ZZ] - s->foot[i][ZZ] );
    }
}

/************************************************************************/
/* Need to handle xd, zd, and pitchd requests correctly */
/* called from integrate_step: air:
        ds_init_double_support_state( s, REF_HIP );
*/

void ds_init_double_support_state( SIM *s, int ref_flag )
{
  double r_foot[3];
  double inbtojoint[3];
  double distance;
  double tt = 0.0;
  double tol = 1e-5;
  double angle_offset;
  int lock[DS_NU];
  int maxevals = 10;
  int fcnt;
  int err;
  int i, j;
  double vector[3];
  double inertia[3][3];
  double foot1[3];
  double knee1[3];
  double hip[3];
  double head[3];
  double knee2[3];
  double foot2[3];
  double offset[3];
  static int firsttime = 0;

  s->sdfast_flag = 1;
  s->sdfast_model = DOUBLE_SUPPORT;

  for ( i = 0; i < DS_NSTATE; i++ )
    s->ds_state[i] = 0;
  /*
    1) Figure out distance between feet.
    2) Figure out x for left foot.
  */

  s->ds_state[DS_ANKLE1] = (-s->pitch + s->hip_angle[LEFT]
			    + s->knee_angle[LEFT]);
  s->ds_state[DS_KNEE1] = -s->knee_angle[LEFT];
  s->ds_state[DS_HIP1] = -s->hip_angle[LEFT];
  s->ds_state[DS_HIP2] = s->hip_angle[RIGHT];
  s->ds_state[DS_KNEE2] = s->knee_angle[RIGHT];

  s->ds_state[DS_ANKLE1D] = (-s->pitchd + s->hip_angled[LEFT]
			     + s->knee_angled[LEFT]);
  s->ds_state[DS_KNEE1D] = -s->knee_angled[LEFT];
  s->ds_state[DS_HIP1D] = -s->hip_angled[LEFT];
  s->ds_state[DS_HIP2D] = s->hip_angled[RIGHT];
  s->ds_state[DS_KNEE2D] = s->knee_angled[RIGHT];

  r_foot[XX] = -s->calf_length*sin( s->ds_state[DS_ANKLE1] ) 
    + -s->thigh_length*sin( s->ds_state[DS_ANKLE1] + s->ds_state[DS_KNEE1] ) 
    + s->thigh_length*sin( s->ds_state[DS_ANKLE1] + s->ds_state[DS_KNEE1]
			   + s->ds_state[DS_HIP1] + s->ds_state[DS_HIP2] ) 
    + s->calf_length*sin( s->ds_state[DS_ANKLE1] + s->ds_state[DS_KNEE1]
			  + s->ds_state[DS_HIP1] + s->ds_state[DS_HIP2]
			  + s->ds_state[DS_KNEE2] );
  r_foot[YY] = 0;
  r_foot[ZZ] = s->calf_length*cos( s->ds_state[DS_ANKLE1] ) 
    + s->thigh_length*cos( s->ds_state[DS_ANKLE1] + s->ds_state[DS_KNEE1] ) 
    - s->thigh_length*cos( s->ds_state[DS_ANKLE1] + s->ds_state[DS_KNEE1]
			   + s->ds_state[DS_HIP1] + s->ds_state[DS_HIP2] ) 
    - s->calf_length*cos( s->ds_state[DS_ANKLE1] + s->ds_state[DS_KNEE1]
			  + s->ds_state[DS_HIP1] + s->ds_state[DS_HIP2]
			  + s->ds_state[DS_KNEE2] );
  distance = sqrt(r_foot[XX]*r_foot[XX] + r_foot[ZZ]*r_foot[ZZ]);

  /* Need a signed distance */
  if ( r_foot[XX] < 0 )
    distance = -distance;

  // should be 0.684582, 0, 0 for test pose.
  /*
  printf( "r_foot: %g %g %g\n", r_foot[XX], r_foot[YY], r_foot[ZZ] );
  printf( "%g %g %g %g\n", s->calf_length, s->ds_state[DS_ANKLE1],
	  sin( s->ds_state[DS_ANKLE1] ), cos( s->ds_state[DS_ANKLE1] ) ); 
  printf( "Distance: %g\n", distance );
  */

  // CGA **** 
  /*
  printf( "OVERRIDING DISTANCE.\n" );
  distance = 0;
  */

  /* initialize SDFAST model two legs on the ground */
  inbtojoint[0] = distance;
  inbtojoint[1] = 0;
  inbtojoint[2] = 0;
  b1dsitj( DS_LOOP_ID, inbtojoint );
  b1dsprinterr(stderr);
  b1dsinit();
  b1dsprinterr(stderr);

  /*
    second_leg_down_transition();
  */

  /*
  printf( "r_foot: %g %g\n", r_foot[XX], r_foot[ZZ] );
  */

  /* Need to handle case where two feet together */
  if ( fabs( r_foot[XX] ) > FEET_TOGETHER )
    {
      if ( r_foot[XX] > 0 )
	angle_offset = atan2( r_foot[ZZ], r_foot[XX] );
      else
	angle_offset = atan2( -r_foot[ZZ], -r_foot[XX] );
      s->ds_state[DS_ANKLE1] -= angle_offset;
    }
  else
    {
      if ( fabs( r_foot[ZZ] ) > FEET_TOGETHER/10 )
	{
	  fprintf( stderr, "ds: rfoot problem: %g %g %g\n", 
		   r_foot[XX], r_foot[ZZ], FEET_TOGETHER );
	}
    };

  for( i = 0; i < DS_NU; i++ )
    lock[i] = 0;

  /*
  printf( "Before assemble: %g %g %g %g %g\n", 
	  s->ds_state[0], s->ds_state[1], s->ds_state[2], s->ds_state[3], s->ds_state[4] );
  */

  b1dsassemble( tt, s->ds_state, lock, tol, maxevals, &fcnt, &err );
  b1dsprinterr(stderr);

  /*
  printf( "After assemble: %d %d: %g %g %g %g %g %g %g %g %g %g\n", err, fcnt,
	  s->ds_state[0], s->ds_state[1], s->ds_state[2], s->ds_state[3], s->ds_state[4],
	  s->ds_state[5], s->ds_state[6], s->ds_state[7], s->ds_state[8], s->ds_state[9] );
  */

  b1dsinitvel( tt, s->ds_state, lock, tol, maxevals, &fcnt, &err );
  b1dsprinterr(stderr);

  /*
  printf( "After initvel: %d %d: %g %g %g %g %g %g %g %g %g %g\n", err, fcnt,
	  s->ds_state[0], s->ds_state[1], s->ds_state[2], s->ds_state[3], s->ds_state[4],
	  s->ds_state[5], s->ds_state[6], s->ds_state[7], s->ds_state[8], s->ds_state[9] );
  */

  s->sdfast_model = DOUBLE_SUPPORT;

  foot1[XX] = 0;
  foot1[YY] = 0;
  foot1[ZZ] = 0;
  offset[XX] = 0;
  offset[YY] = s->knee_offset[ZZ];
  offset[ZZ] = 0;
  b1dspos( DS_CALF1_ID, offset, knee1 );
  offset[YY] = s->hip_offset[ZZ];
  b1dspos( DS_TORSO_ID, offset, hip );
  offset[YY] = s->head_offset[ZZ];
  b1dspos( DS_TORSO_ID, offset, head );
  offset[YY] = s->knee_offset[ZZ];
  b1dspos( DS_CALF2_ID, offset, knee2 );
  offset[YY] = s->foot_offset[ZZ];
  b1dspos( DS_CALF2_ID, offset, foot2 );

  /*
  printf( "hip: %g %g; %g %g\n", s->hip[XX], s->hip[ZZ], hip[XX], s->x_offset );
  */
  /*
  s->x_offset = s->foot[LEFT][XX];
  */
  if ( ref_flag == REF_HIP )
    s->x_offset = s->hip[XX] - hip[XX];
  else if ( ref_flag = REF_LFOOT )
    s->x_offset = s->foot[LEFT][XX] - foot1[XX];
  else if ( ref_flag = REF_RFOOT )
    s->x_offset = s->foot[RIGHT][XX] - foot2[XX];

  /*
  printf( "hip: %g %g; %g %g\n", s->hip[XX], s->hip[ZZ], hip[XX], s->x_offset );
  printf( "foot1: %g %g\n", foot1[XX] + s->x_offset, foot1[YY] );
  printf( "knee1: %g %g\n", knee1[XX] + s->x_offset, knee1[YY] );
  printf( "hip: %g %g\n", hip[XX] + s->x_offset, hip[YY] );
  printf( "head: %g %g\n", head[XX] + s->x_offset, head[YY] );
  printf( "knee2: %g %g\n", knee2[XX] + s->x_offset, knee2[YY] );
  printf( "foot2: %g %g\n", foot2[XX] + s->x_offset, foot2[YY] );
  */

  ds_forward_kinematics( s );

  /*
  printf( "foot-ds2: %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ] );
  printf( "Press return to continue.\n" );
  getchar();}
  */
}

/************************************************************************/

void init_state_two_feet_on_ground( SIM *s )
{

  ds_init_double_support_state( s, REF_HIP );
  s->status = OK;
}

/************************************************************************/
/* Initialize sdfast  */

void init_dynamics( SIM *s )
{
  int i, j;

  if ( NSTATE > MAX_N_SDFAST_STATE )
    {
      fprintf( stderr, 
	       "Need to increast MAX_N_SDFAST_STATE (%d) to be at least %d\n",
	       MAX_N_SDFAST_STATE, NSTATE );
      exit( -1 );
    }

  s->sdfast_model = INITIALIZE;

  init_in_air_model( s );
  init_single_support_model( s );
  init_double_support_model( s );

  for( i = 0; i < NSTATE; i++ )
    s->sdfast_state[i] = 0;
  for( i = 0; i < SS_NSTATE; i++ )
    s->ss_state[i] = 0;
  for( i = 0; i < DS_NSTATE; i++ )
    s->ds_state[i] = 0;

  // Initialize straight up */
  s->hip[XX] = 0.0;
  s->hip[ZZ] =  0.7728;
  s->pitch = 0.0;
  s->hip_angle[LEFT] = 0.0;
  s->hip_angle[RIGHT] = 0.0;
  s->knee_angle[LEFT] = 0.0;
  s->knee_angle[RIGHT] = 0.0;
  s->ankle_angle[LEFT] = 0.0;
  s->ankle_angle[RIGHT] = 0.0;

  // Initialize in test pose */
  /*
  s->hip[XX] = -0.7728;
  s->hip[ZZ] =  0.670752;
  s->pitch = 0.0858461;
  s->hip_angle[LEFT] = -0.2;
  s->hip_angle[RIGHT] = 0.3;
  s->knee_angle[LEFT] = -0.4;
  s->knee_angle[RIGHT] = 0.5;
  s->ankle_angle[LEFT] = 0.685846;
  s->ankle_angle[RIGHT] = -0.714154;
  */

  // init_state_in_air( s, s->sdfast_state );
  // init_state_two_feet_on_ground( s );
  // init_state_one_foot_on_ground( s );

  // debug_positions( s );
}

/************************************************************************/
/* Could be called from DS or SS */

void transition_to_air( SIM *s )
{

  if ( interactive )
    printf( "transition_to_air: %g\n", s->time );

  if ( s->in_air_not_allowed )
    {
      fprintf( stderr, "In air not allowed: %g\n", s->time );
      s->status = CRASHED;
      return;
    }

  init_state_in_air( s );
}

/************************************************************************/
/* Begin single support, right stance; from ds or in air */
/* Called from integrate_step: in air and ds */

ss_right_stance( SIM *s )
{
  int i, j;
  double vector[3];
  double inertia[3][3];
  double foot1[3];
  double knee1[3];
  double hip[3];
  double head[3];
  double knee2[3];
  double foot2[3];
  double offset[3];
  static int firsttime = 0;

  if ( interactive )
    printf( "ss_right_stance: %g\n", s->time );

  /*
  printf( "ss_state: " );
  for( i = 0; i < SS_NQ; i++ )
    printf( "%g \n", s->ss_state[i] );
  printf( "\n" );
  */

  /*
  printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
  printf( "knees: %g %g; %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ],
	  s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
  printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	  s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
  printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
  printf( "kneesd: %g %g; %g %g\n", s->kneed[LEFT][XX], s->kneed[LEFT][ZZ],
	  s->kneed[RIGHT][XX], s->kneed[RIGHT][ZZ] );
  printf( "feetd: %g %g; %g %g\n", s->footd[LEFT][XX], s->footd[LEFT][ZZ],
	  s->footd[RIGHT][XX], s->footd[RIGHT][ZZ] );
  */

  /*
  printf( "pitch: %g\n", s->pitch );
  printf( "hip angles: %g %g\n", s->hip_angle[LEFT], s->hip_angle[RIGHT] );
  printf( "knee angles: %g %g\n", s->knee_angle[LEFT], s->knee_angle[RIGHT] );
  */

  s->sdfast_flag = 1;
  s->sdfast_model = SINGLE_SUPPORT;
  s->ss_foot_down = RIGHT;

  for ( i = 0; i < SS_NSTATE; i++ )
    {
      s->ss_state[i] = 0;
      s->ss_stated[i] = 0;
      s->sdfast_stated[i] = 0;
    }

  s->ss_state[SS_ANKLE1] = (-s->pitch + s->hip_angle[RIGHT] + s->knee_angle[RIGHT]);
  s->ss_state[SS_KNEE1] = -s->knee_angle[RIGHT];
  s->ss_state[SS_HIP1] = -s->hip_angle[RIGHT];
  s->ss_state[SS_HIP2] = s->hip_angle[LEFT];
  s->ss_state[SS_KNEE2] = s->knee_angle[LEFT];

  s->sdfast_stated[SS_ANKLE1] = s->ss_state[SS_ANKLE1D] = 
    (-s->pitchd + s->hip_angled[RIGHT] + s->knee_angled[RIGHT]);
  s->sdfast_stated[SS_KNEE1] = s->ss_state[SS_KNEE1D] = -s->knee_angled[RIGHT];
  s->sdfast_stated[SS_HIP1] = s->ss_state[SS_HIP1D] = -s->hip_angled[RIGHT];
  s->sdfast_stated[SS_HIP2] = s->ss_state[SS_HIP2D] = s->hip_angled[LEFT];
  s->sdfast_stated[SS_KNEE2] = s->ss_state[SS_KNEE2D] = s->knee_angled[LEFT];

  b1ssstate( 0.0, s->ss_state, s->sdfast_stated );
  b1ssprinterr(stderr);
  s->status = OK;

  /*
  printf( "offset1: %g %g\n", s->time, s->x_offset );
  offset[XX] = 0;
  offset[YY] = s->hip_offset[ZZ];
  offset[ZZ] = 0;
  b1sspos( SS_TORSO_ID, offset, hip );
  s->x_offset = s->hip[XX] - hip[XX];
  printf( "offset2: %g %g %g %g\n", s->time, s->hip[XX], hip[XX],
	  s->x_offset );
  */
  s->x_offset = s->foot[RIGHT][XX];

  ss_forward_kinematics( s );

  /*
  printf( "after ss_forward_k: %g %g\n", s->hip[XX], s->hip[ZZ] );

  printf( "ss_state: " );
  for( i = 0; i < SS_NQ; i++ )
    printf( "%g \n", s->ss_state[i] );
  printf( "\n" );
  */

  /*
  printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
  printf( "knees: %g %g; %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ],
	  s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
  printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	  s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
  printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
  printf( "kneesd: %g %g; %g %g\n", s->kneed[LEFT][XX], s->kneed[LEFT][ZZ],
	  s->kneed[RIGHT][XX], s->kneed[RIGHT][ZZ] );
  printf( "feetd: %g %g; %g %g\n", s->footd[LEFT][XX], s->footd[LEFT][ZZ],
	  s->footd[RIGHT][XX], s->footd[RIGHT][ZZ] );

  printf( "pitch: %g\n", s->pitch );
  printf( "hip angles: %g %g\n", s->hip_angle[LEFT], s->hip_angle[RIGHT] );
  printf( "knee angles: %g %g\n", s->knee_angle[LEFT], s->knee_angle[RIGHT] );
  */
}

/************************************************************************/
/* Begin single support, left stance; from ds or in air */
/* Called from integrate_step: in air and ds */

ss_left_stance( SIM *s )
{ 
  int i, j;
  double vector[3];
  double inertia[3][3];
  double foot1[3];
  double knee1[3];
  double hip[3];
  double head[3];
  double knee2[3];
  double foot2[3];
  double offset[3];
  static int firsttime = 0;

  if ( interactive )
    printf( "ss_left_stance: %g\n", s->time );

  /*
  printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	  s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
  */
  /*
  printf( "ss_left_stance:\n" );
  printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
  printf( "knees: %g %g; %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ],
	  s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
  printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	  s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );

  printf( "pitch: %g\n", s->pitch );
  printf( "hip angles: %g %g\n", s->hip_angle[LEFT], s->hip_angle[RIGHT] );
  printf( "knee angles: %g %g\n", s->knee_angle[LEFT], s->knee_angle[RIGHT] );

  printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
  printf( "kneesd: %g %g; %g %g\n", s->kneed[LEFT][XX], s->kneed[LEFT][ZZ],
	  s->kneed[RIGHT][XX], s->kneed[RIGHT][ZZ] );
  printf( "feetd: %g %g; %g %g\n", s->footd[LEFT][XX], s->footd[LEFT][ZZ],
	  s->footd[RIGHT][XX], s->footd[RIGHT][ZZ] );

  printf( "pitchd: %g\n", s->pitchd );
  printf( "hip angular velocities: %g %g\n", s->hip_angled[LEFT], s->hip_angled[RIGHT] );
  printf( "knee angular velocities: %g %g\n", s->knee_angled[LEFT], s->knee_angled[RIGHT] );
  */

  s->sdfast_flag = 1;
  s->sdfast_model = SINGLE_SUPPORT;
  s->ss_foot_down = LEFT;

  for ( i = 0; i < SS_NSTATE; i++ )
    {
      s->ss_state[i] = 0;
      s->ss_stated[i] = 0;
      s->sdfast_stated[i] = 0;
    }

  s->ss_state[SS_ANKLE1] = (-s->pitch + s->hip_angle[LEFT] 
			    + s->knee_angle[LEFT]);
  s->ss_state[SS_KNEE1] = -s->knee_angle[LEFT];
  s->ss_state[SS_HIP1] = -s->hip_angle[LEFT];
  s->ss_state[SS_HIP2] = s->hip_angle[RIGHT];
  s->ss_state[SS_KNEE2] = s->knee_angle[RIGHT];

  s->sdfast_stated[SS_ANKLE1] = s->ss_state[SS_ANKLE1D] = 
    (-s->pitchd + s->hip_angled[LEFT] + s->knee_angled[LEFT]);
  s->sdfast_stated[SS_KNEE1] = s->ss_state[SS_KNEE1D] = -s->knee_angled[LEFT];
  s->sdfast_stated[SS_HIP1] = s->ss_state[SS_HIP1D] = -s->hip_angled[LEFT];
  s->sdfast_stated[SS_HIP2] = s->ss_state[SS_HIP2D] = s->hip_angled[RIGHT];
  s->sdfast_stated[SS_KNEE2] = s->ss_state[SS_KNEE2D] = s->knee_angled[RIGHT];

  b1ssstate( 0.0, s->ss_state, s->sdfast_stated );
  b1ssprinterr(stderr);
  s->status = OK;

  /*
  printf( "offset1: %g %g\n", s->time, s->x_offset );
  offset[XX] = 0;
  offset[YY] = s->hip_offset[ZZ];
  offset[ZZ] = 0;
  b1sspos( SS_THIGH1_ID, offset, hip );
  s->x_offset = s->hip[XX] - hip[XX];
  printf( "offset2: %g %g %g %g\n", s->time, s->hip[XX], hip[XX],
	  s->x_offset );
  */
  s->x_offset = s->foot[LEFT][XX];

  ss_forward_kinematics( s );

  /*
  printf( "xfeet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	  s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
  */

  /*
  printf( "after ss_forward_k\n" );
  printf( "ss_state: " );
  for( i = 0; i < SS_NQ; i++ )
    printf( "%g \n", s->ss_state[i] );
  printf( "\n" );

  printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
  printf( "knees: %g %g; %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ],
	  s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
  printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	  s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );

  printf( "pitch: %g\n", s->pitch );
  printf( "hip angles: %g %g\n", s->hip_angle[LEFT], s->hip_angle[RIGHT] );
  printf( "knee angles: %g %g\n", s->knee_angle[LEFT], s->knee_angle[RIGHT] );

  printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
  printf( "kneesd: %g %g; %g %g\n", s->kneed[LEFT][XX], s->kneed[LEFT][ZZ],
	  s->kneed[RIGHT][XX], s->kneed[RIGHT][ZZ] );
  printf( "feetd: %g %g; %g %g\n", s->footd[LEFT][XX], s->footd[LEFT][ZZ],
	  s->footd[RIGHT][XX], s->footd[RIGHT][ZZ] );

  printf( "pitchd: %g\n", s->pitchd );
  printf( "hip angular velocities: %g %g\n", s->hip_angled[LEFT], s->hip_angled[RIGHT] );
  printf( "knee angular velocities: %g %g\n", s->knee_angled[LEFT], s->knee_angled[RIGHT] );
  */
}

/*************************************************************************/
/* Foot 1 is on the ground, and foot 2 just touched down */

/* Assume SS_IMPACT1_N == SS_IMPACT2_N in a, b, allocation */
#define SS_IMPACT1_N 17

/* Not thread safe */
float **a = NULL;
float *b = NULL;

ss_impact1( SIM *s, double *result )
{
  float **matrix();
  float *vector();
  float d;
  int n, indx[SS_IMPACT1_N+1];
  int i, j;
  double m1, I1, l1, r1;
  double a1, j1xdi, j1ydi, a1di;
  double c1, s1;
  double m2, I2, l2, r2;
  double a2, j2xdi, j2ydi, a2di;
  double c12, s12;
  double m3, I3, r3;
  double a3, j3xdi, j3ydi, a3di;
  double c123, s123;
  double m4, I4, l4, r4;
  double a4, j4xdi, j4ydi, a4di;
  double c1234, s1234;
  double m5, I5, l5, r5;
  double a5, j5xdi, j5ydi, a5di;
  double c12345, s12345;

  m1 = s->calf_mass;
  I1 = s->calf_I;
  l1 = s->calf_length;
  r1 = s->calf_length - s->calf_cm;

  m2 = s->thigh_mass;
  I2 = s->thigh_I;
  l2 = s->thigh_length;
  r2 = s->thigh_length - s->thigh_cm;

  m3 = s->torso_mass;
  I3 = s->torso_I;
  r3 = s->torso_cm;

  m4 = s->thigh_mass;
  I4 = s->thigh_I;
  l4 = s->thigh_length;
  r4 = s->thigh_cm;

  m5 = s->calf_mass;
  I5 = s->calf_I;
  l5 = s->calf_length;;
  r5 = s->calf_cm;

  a1 = s->ss_state[0];
  a2 = s->ss_state[1];
  a3 = s->ss_state[2];
  a4 = s->ss_state[3];
  a5 = s->ss_state[4];
  a1di = s->ss_state[5];
  a2di = s->ss_state[6];
  a3di = s->ss_state[7];
  a4di = s->ss_state[8];
  a5di = s->ss_state[9];

  if ( impact1_print )
    printf( "%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n",
	    a1, a2, a3, a4, a5, a1di, a2di, a3di, a4di, a5di );

  c1 = cos( a1 );
  s1 = sin( a1 );
  c12 = cos( a1 + a2 );
  s12 = sin( a1 + a2 );
  c123 = cos( a1 + a2 + a3 );
  s123 = sin( a1 + a2 + a3 );
  c1234 = cos( a1 + a2 + a3 + a4 );
  s1234 = sin( a1 + a2 + a3 + a4 );
  c12345 = cos( a1 + a2 + a3 + a4 + a5 );
  s12345 = sin( a1 + a2 + a3 + a4 + a5 );

  n = SS_IMPACT1_N;
  if ( a == NULL )
    a = matrix( 1, n, 1, n );
  if ( b == NULL )
    b = vector( 1, n );

  for( i = 1; i <= n; i++ )
    {
      b[i] = 0;
      for( j = 1; j <= n; j++ )
        {
          a[i][j] = 0;
        }
    }

  a[1][1] = -(m1*r1*c1);
  a[1][6] = 1;
  a[1][8] = -1;

  a[2][1] = -(m1*r1*s1);
  a[2][7] = 1;
  a[2][9] = -1;

  a[3][1] = -(l1*m2*c1) - m2*r2*c12;
  a[3][2] = -(m2*r2*c12);
  a[3][8] = 1;
  a[3][10] = -1;

  a[4][1] = -(l1*m2*s1) - m2*r2*s12;
  a[4][2] = -(m2*r2*s12);
  a[4][9] = 1;
  a[4][11] = -1;

  a[5][1] = -(l1*m3*c1) - l2*m3*c12 - m3*r3*c123;
  a[5][2] = -(l2*m3*c12) - m3*r3*c123;
  a[5][3] = -(m3*r3*c123);
  a[5][10] = 1;
  a[5][12] = -1;

  a[6][1] = -(l1*m3*s1) - l2*m3*s12 - m3*r3*s123;
  a[6][2] = -(l2*m3*s12) - m3*r3*s123;
  a[6][3] = -(m3*r3*s123);
  a[6][11] = 1;
  a[6][13] = -1;

  a[7][1] = -(l1*m4*c1) - l2*m4*c12 + m4*r4*c1234;
  a[7][2] = -(l2*m4*c12) + m4*r4*c1234;
  a[7][3] = m4*r4*c1234;
  a[7][4] = m4*r4*c1234;
  a[7][12] = 1;
  a[7][14] = -1;

  a[8][1] = -(l1*m4*s1) - l2*m4*s12 + m4*r4*s1234;
  a[8][2] = -(l2*m4*s12) + m4*r4*s1234;
  a[8][3] = m4*r4*s1234;
  a[8][4] = m4*r4*s1234;
  a[8][13] = 1;
  a[8][15] = -1;

  a[9][1] = -(l1*m5*c1) - l2*m5*c12 + l4*m5*c1234 + 
    m5*r5*c12345;
  a[9][2] = -(l2*m5*c12) + l4*m5*c1234 + 
    m5*r5*c12345;
  a[9][3] = l4*m5*c1234 + m5*r5*c12345;
  a[9][4] = l4*m5*c1234 + m5*r5*c12345;
  a[9][5] = m5*r5*c12345;
  a[9][14] = 1;
  a[9][16] = -1;

  a[10][1] = -(l1*m5*s1) - l2*m5*s12 + l4*m5*s1234 + 
    m5*r5*s12345;
  a[10][2] = -(l2*m5*s12) + l4*m5*s1234 + 
    m5*r5*s12345;
  a[10][3] = l4*m5*s1234 + m5*r5*s12345;
  a[10][4] = l4*m5*s1234 + m5*r5*s12345;
  a[10][5] = m5*r5*s12345;
  a[10][15] = 1;
  a[10][17] = -1;

  a[11][1] = I1;
  a[11][6] = r1*c1;
  a[11][7] = r1*s1;
  a[11][8] = l1*c1 - r1*c1;
  a[11][9] = l1*s1 - r1*s1;

  a[12][1] = I2;
  a[12][2] = I2;
  a[12][8] = r2*c12;
  a[12][9] = r2*s12;
  a[12][10] = l2*c12 - r2*c12;
  a[12][11] = l2*s12 - r2*s12;

  a[13][1] = I3;
  a[13][2] = I3;
  a[13][3] = I3;
  a[13][10] = r3*c123;
  a[13][11] = r3*s123;
  a[13][12] = -r3*c123;
  a[13][13] = -r3*s123;

  a[14][1] = I4;
  a[14][2] = I4;
  a[14][3] = I4;
  a[14][4] = I4;
  a[14][12] = -(r4*c1234);
  a[14][13] = -(r4*s1234);
  a[14][14] = -(l4*c1234) + r4*c1234;
  a[14][15] = -(l4*s1234) + r4*s1234;

  a[15][1] = I5;
  a[15][2] = I5;
  a[15][3] = I5;
  a[15][4] = I5;
  a[15][5] = I5;
  a[15][14] = -(r5*c12345);
  a[15][15] = -(r5*s12345);
  a[15][16] = -(l5*c12345) + r5*c12345;
  a[15][17] = -(l5*s12345) + r5*s12345;

  a[16][1] = -(l1*c1) - l2*c12 + l4*c1234 + 
    l5*c12345;
  a[16][2] = -(l2*c12) + l4*c1234  + 
    l5*c12345;
  a[16][3] = l4*c1234 + l5*c12345;
  a[16][4] = l4*c1234 + l5*c12345;
  a[16][5] = l5*c12345;

  a[17][1] = -(l1*s1) - l2*s12 + l4*s1234 + 
    l5*s12345;
  a[17][2] = -(l2*s12) + l4*s1234 + 
    l5*s12345;
  a[17][3] = l4*s1234 + l5*s12345;
  a[17][4] = l4*s1234 + l5*s12345;
  a[17][5] = l5*s12345;

  b[1] = -(a1di*m1*r1*c1);
  b[2] = -(a1di*m1*r1*s1);
  b[3] = -(a1di*l1*m2*c1) - a1di*m2*r2*c12 - a2di*m2*r2*c12;
  b[4] = -(a1di*l1*m2*s1) - a1di*m2*r2*s12 - a2di*m2*r2*s12;
  b[5] = -(a1di*l1*m3*c1) - a1di*l2*m3*c12 - 
    a2di*l2*m3*c12 - a1di*m3*r3*c123 - 
    a2di*m3*r3*c123 - a3di*m3*r3*c123;
  b[6] = -(a1di*l1*m3*s1) - a1di*l2*m3*s12 - 
    a2di*l2*m3*s12 - a1di*m3*r3*s123 - 
    a2di*m3*r3*s123 - a3di*m3*r3*s123;
  b[7] = -(a1di*l1*m4*c1) - a1di*l2*m4*c12 - 
    a2di*l2*m4*c12 + a1di*m4*r4*c1234 + 
    a2di*m4*r4*c1234 + a3di*m4*r4*c1234 + 
    a4di*m4*r4*c1234;
  b[8] = -(a1di*l1*m4*s1) - a1di*l2*m4*s12 - 
    a2di*l2*m4*s12 + a1di*m4*r4*s1234 + 
    a2di*m4*r4*s1234 + a3di*m4*r4*s1234 + 
    a4di*m4*r4*s1234;
  b[9] =  -(a1di*l1*m5*c1) - a1di*l2*m5*c12 - 
    a2di*l2*m5*c12 + a1di*l4*m5*c1234 + 
    a2di*l4*m5*c1234 + a3di*l4*m5*c1234 + 
    a4di*l4*m5*c1234 + 
    a1di*m5*r5*c12345 + 
    a2di*m5*r5*c12345 + 
    a3di*m5*r5*c12345 + 
    a4di*m5*r5*c12345 + 
    a5di*m5*r5*c12345;
  b[10] =  -(a1di*l1*m5*s1) - a1di*l2*m5*s12 - 
    a2di*l2*m5*s12 + a1di*l4*m5*s1234 + 
    a2di*l4*m5*s1234 + a3di*l4*m5*s1234 + 
    a4di*l4*m5*s1234 + 
    a1di*m5*r5*s12345 + 
    a2di*m5*r5*s12345 + 
    a3di*m5*r5*s12345 + 
    a4di*m5*r5*s12345 + 
    a5di*m5*r5*s12345;
  b[11] = a1di*I1;
  b[12] = a1di*I2 + a2di*I2;
  b[13] = a1di*I3 + a2di*I3 + a3di*I3;
  b[14] = a1di*I4 + a2di*I4 + a3di*I4 + a4di*I4;
  b[15] = a1di*I5 + a2di*I5 + a3di*I5 + a4di*I5 + a5di*I5;
  b[16] = 0;
  b[17] = 0;

  ludcmp(a,n,indx,&d);
  lubksb(a,n,indx,b);

  for( i = 1; i <= n; i++ )
    {
      /*
      printf( "%d: %g\n", i, b[i] );
      */
      result[i-1] = b[i];
    }
}

/************************************************************************/
/* This impact analysis assumes robot in air and one
   foot hits */

/* Assume SS_IMPACT1_N == SS_IMPACT2_N in a, b, allocation */
#define SS_IMPACT2_N 17

ss_impact2( SIM *s, double *result )
{
  float **matrix();
  float *vector();
  float d;
  int n, indx[SS_IMPACT2_N+1];
  int i, j;
  double j1x, j1y;
  double m1, I1, r1;
  double a1, j1xdi, j1ydi, a1di;
  double c1, s1;
  double m2, I2, l2, r2;
  double a2, j2xdi, j2ydi, a2di;
  double c12, s12;
  double m3, I3, l3, r3;
  double a3, j3xdi, j3ydi, a3di;
  double c123, s123;
  double m4, I4, l4, r4;
  double a4, j4xdi, j4ydi, a4di;
  double c14, s14;
  double m5, I5, l5, r5;
  double a5, j5xdi, j5ydi, a5di;
  double c145, s145;
  double footx, footy, footxd, footyd;

  m1 = s->torso_mass;
  I1 = s->torso_I;
  r1 = s->torso_cm;

  m2 = s->thigh_mass;
  I2 = s->thigh_I;
  l2 = s->thigh_length;
  r2 = s->thigh_cm;

  m3 = s->calf_mass;
  I3 = s->calf_I;
  l3 = s->calf_length;
  r3 = s->calf_cm;

  m4 = s->thigh_mass;
  I4 = s->thigh_I;
  l4 = s->thigh_length;
  r4 = s->thigh_cm;

  m5 = s->calf_mass;
  I5 = s->calf_I;
  l5 = s->calf_length;
  r5 = s->calf_cm;

  a1 = -s->pitch;
  j1x = s->hip[XX];
  j1y = s->hip[ZZ];
  a1di = -s->pitchd;
  j1xdi = s->hipd[XX];
  j1ydi = s->hipd[ZZ];
  if ( s->ss_foot_down == LEFT )
    {
      a2 = s->hip_angle[RIGHT];
      a3 = s->knee_angle[RIGHT];
      a4 = s->hip_angle[LEFT];
      a5 = s->knee_angle[LEFT];
      a2di = s->hip_angled[RIGHT];
      a3di = s->knee_angled[RIGHT];
      a4di = s->hip_angled[LEFT];
      a5di = s->knee_angled[LEFT];
    }
  else
    {
      a2 = s->hip_angle[LEFT];
      a3 = s->knee_angle[LEFT];
      a4 = s->hip_angle[RIGHT];
      a5 = s->knee_angle[RIGHT];
      a2di = s->hip_angled[LEFT];
      a3di = s->knee_angled[LEFT];
      a4di = s->hip_angled[RIGHT];
      a5di = s->knee_angled[RIGHT];
    }

  if ( impact2_print )
    printf( "%10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f %10.5f\n",
	    j1x, j1y, a1, a2, a3, a4, a5, j1xdi, j1ydi, a1di, a2di, a3di, a4di, a5di );

  c1 = cos( a1 );
  s1 = sin( a1 );
  c12 = cos( a1 + a2 );
  s12 = sin( a1 + a2 );
  c123 = cos( a1 + a2 + a3 );
  s123 = sin( a1 + a2 + a3 );
  c14 = cos( a1 + a4 );
  s14 = sin( a1 + a4 );
  c145 = cos( a1 + a4 + a5 );
  s145 = sin( a1 + a4 + a5 );

  n = SS_IMPACT2_N;
  if ( a == NULL )
    a = matrix( 1, n, 1, n );
  if ( b == NULL )
    b = vector( 1, n );

  for( i = 1; i <= n; i++ )
    {
      b[i] = 0;
      for( j = 1; j <= n; j++ )
        {
          a[i][j] = 0;
        }
    }

  a[1][1] = m1;
  a[1][3] = -m1*r1*c1;
  a[1][8] = -1;
  a[1][14] = -1;

  a[2][2] = m1;
  a[2][3] = -m1*r1*s1;
  a[2][9] = -1;
  a[2][15] = -1;

  a[3][1] = m2;
  a[3][3] = m2*r2*c12;
  a[3][4] = m2*r2*c12;
  a[3][8] = 1;
  a[3][10] = -1;

  a[4][2] = m2;
  a[4][3] = m2*r2*s12;
  a[4][4] = m2*r2*s12;
  a[4][9] = 1;
  a[4][11] = -1;

  a[5][1] = m3;
  a[5][3] = l2*m3*c12 + m3*r3*c123;
  a[5][4] = l2*m3*c12 + m3*r3*c123;
  a[5][5] = m3*r3*c123;
  a[5][10] = 1;
  a[5][12] = -1;

  a[6][2] = m3;
  a[6][3] = l2*m3*s12 + m3*r3*s123;
  a[6][4] = l2*m3*s12 + m3*r3*s123;
  a[6][5] = m3*r3*s123;
  a[6][11] = 1;
  a[6][13] = -1;

  a[7][1] = m4;
  a[7][3] = m4*r4*c14;
  a[7][6] = m4*r4*c14;
  a[7][14] = 1;
  a[7][16] = -1;

  a[8][2] = m4;
  a[8][3] = m4*r4*s14;
  a[8][6] = m4*r4*s14;
  a[8][15] = 1;
  a[8][17] = -1;

  a[9][1] = m5;
  a[9][3] = l4*m5*c14 + m5*r5*c145;
  a[9][6] = l4*m5*c14 + m5*r5*c145;
  a[9][7] = m5*r5*c145;
  a[9][16] = 1;

  a[10][2] = m5;
  a[10][3] = l4*m5*s14 + m5*r5*s145;
  a[10][6] = l4*m5*s14 + m5*r5*s145;
  a[10][7] = m5*r5*s145;
  a[10][17] = 1;

  a[11][3] = I1;
  a[11][8] = -r1*c1;
  a[11][9] = -r1*s1;
  a[11][14] = -r1*c1;
  a[11][15] = -r1*s1;

  a[12][3] = I2;
  a[12][4] = I2;
  a[12][8] = -(r2*c12);
  a[12][9] = -(r2*s12);
  a[12][10] = -(l2*c12) + r2*c12;
  a[12][11] = -(l2*s12) + r2*s12;

  a[13][3] = I3;
  a[13][4] = I3;
  a[13][5] = I3;
  a[13][10] = -(r3*c123);
  a[13][11] = -(r3*s123);
  a[13][12] = -(l3*c123) + r3*c123;
  a[13][13] = -(l3*s123) + r3*s123;

  a[14][3] = I4;
  a[14][6] = I4;
  a[14][14] = -(r4*c14);
  a[14][15] = -(r4*s14);
  a[14][16] = -(l4*c14) + r4*c14;
  a[14][17] = -(l4*s14) + r4*s14;

  a[15][3] = I5;
  a[15][6] = I5;
  a[15][7] = I5;
  a[15][16] = -(r5*c145);
  a[15][17] = -(r5*s145);

  a[16][1] = 1;
  a[16][3] = l2*c12 + l3*c123;
  a[16][4] = l2*c12 + l3*c123;
  a[16][5] = l3*c123;

  a[17][2] = 1;
  a[17][3] = l2*s12 + l3*s123;
  a[17][4] = l2*s12 + l3*s123;
  a[17][5] = l3*s123;

  b[1] = j1xdi*m1 - a1di*m1*r1*c1;
  b[2] = j1ydi*m1 - a1di*m1*r1*s1;
  b[3] = j1xdi*m2 + a1di*m2*r2*c12 + a2di*m2*r2*c12;
  b[4] = j1ydi*m2 + a1di*m2*r2*s12 + a2di*m2*r2*s12;
  b[5] = j1xdi*m3 + a1di*l2*m3*c12 + a2di*l2*m3*c12 + a1di*m3*r3*c123 + 
    a2di*m3*r3*c123 + a3di*m3*r3*c123;
  b[6] = j1ydi*m3 + a1di*l2*m3*s12 + a2di*l2*m3*s12 + a1di*m3*r3*s123 + 
    a2di*m3*r3*s123 + a3di*m3*r3*s123;
  b[7] = j1xdi*m4 + a1di*m4*r4*c14 + a4di*m4*r4*c14;
  b[8] = j1ydi*m4 + a1di*m4*r4*s14 + a4di*m4*r4*s14;
  b[9] = j1xdi*m5 + a1di*l4*m5*c14 + a4di*l4*m5*c14 + a1di*m5*r5*c145 
    + a4di*m5*r5*c145 + a5di*m5*r5*c145;
  b[10] = j1ydi*m5 + a1di*l4*m5*s14 + a4di*l4*m5*s14 + a1di*m5*r5*s145
    + a4di*m5*r5*s145 + a5di*m5*r5*s145;
  b[11] = a1di*I1;
  b[12] = a1di*I2 + a2di*I2;
  b[13] = a1di*I3 + a2di*I3 + a3di*I3;
  b[14] = a1di*I4 + a4di*I4;
  b[15] = a1di*I5 + a4di*I5 + a5di*I5;
  b[16] = 0;
  b[17] = 0;

  ludcmp(a,n,indx,&d);
  lubksb(a,n,indx,b);

  for( i = 1; i <= n; i++ )
    {
      /*
      printf( "%d: %g\n", i, b[i] );
      */
      result[i-1] = b[i];
    }

  footx = j1x + l4*s14 + l5*s145;
  footy = j1y - l4*c14 - l5*c145;
  footxd = b[1] + (b[3] + b[6])*l4*c14
    + (b[3] + b[6] + b[7])*l5*c145;
  footyd = b[2] + (b[3] + b[6])*l4*s14
    + (b[3] + b[6] + b[7])*l5*s145;

  result[17] = footx;
  result[18] = footy;
  result[19] = footxd;
  result[20] = footyd;
  result[21] = b[1];
  result[22] = b[2];
  result[23] = b[1] + (b[3] + b[6])*l4*c14;
  result[24] = b[2] + (b[3] + b[6])*l4*s14;
}

/************************************************************************/

ss_impact2_update( SIM *s, double *impact2_analysis )
{

  s->hipd[XX] = impact2_analysis[0];
  s->hipd[ZZ] = impact2_analysis[1];
  s->pitchd = -impact2_analysis[2];
  if ( s->ss_foot_down == LEFT )
    {
      s->hip_angled[RIGHT] = impact2_analysis[3];
      s->knee_angled[RIGHT] = impact2_analysis[4];
      s->hip_angled[LEFT] = impact2_analysis[5];
      s->knee_angled[LEFT] = impact2_analysis[6];
    }
  else
    {
      s->hip_angled[LEFT] = impact2_analysis[3];
      s->knee_angled[LEFT] = impact2_analysis[4];
      s->hip_angled[RIGHT] = impact2_analysis[5];
      s->knee_angled[RIGHT] = impact2_analysis[6];
    }
}

/************************************************************************/
/* impact analysis with a foot free.*/
/* This can be called from air or single support */

touchdown_to_ss( SIM *s )
{
  double impact2_analysis[SS_IMPACT2_N+4+4];
  int i;

  if ( s->sdfast_model == SINGLE_SUPPORT )
    for ( i = 0; i < SS_NSTATE; i++ )
      s->stashed_ss_state[i] = s->ss_state[i];

  if ( interactive )
    printf( "touchdown_to_ss: %g %d\n", s->time, s->ss_foot_down );

  ss_impact2( s, impact2_analysis );

  if ( impact2_analysis[12] < 0 && interactive )
    {
      printf( "Negative impulse: ss_impact2????\n" );
      impact2_print = 1;
    }

  if ( impact2_print )
    {
      printf( "p, lh, lk, rh, rk: %g %g %g %g %g\n",
	      s->pitch, s->hip_angle[LEFT],
	      s->knee_angle[LEFT],
	      s->hip_angle[RIGHT],
	      s->knee_angle[RIGHT] );

      printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
      printf( "knees: %g %g; %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ],
	      s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
      printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	      s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
      printf( "feetd: %g %g; %g %g\n", s->footd[LEFT][XX], s->footd[LEFT][ZZ],
	      s->footd[RIGHT][XX], s->footd[RIGHT][ZZ] );
      printf( "force: %g %g; %g %g\n", s->ground_force[LEFT][XX], s->ground_force[LEFT][ZZ],
	      s->ground_force[RIGHT][XX], s->ground_force[RIGHT][ZZ] );

      for( i = 0; i < SS_IMPACT2_N + 4 + 4; i++ )
	{
	  printf( "%d: %g\n", i, impact2_analysis[i] );
	}
    }

  ss_impact2_update( s, impact2_analysis );

  if ( s->ss_foot_down == LEFT )
    ss_right_stance( s );
  else
    ss_left_stance( s );

  if ( impact2_print )
    {
      printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
      printf( "knees: %g %g; %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ],
	      s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
      printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	      s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
      printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
      printf( "kneesd: %g %g; %g %g\n", s->kneed[LEFT][XX], s->kneed[LEFT][ZZ],
	      s->kneed[RIGHT][XX], s->kneed[RIGHT][ZZ] );
      printf( "feetd: %g %g; %g %g\n", s->footd[LEFT][XX], s->footd[LEFT][ZZ],
	      s->footd[RIGHT][XX], s->footd[RIGHT][ZZ] );
    }
  impact2_print = 0;
}

/************************************************************************/
/* Need to rerun impact analysis with foot1 free.*/

ss_touchdown_to_ds( SIM *s )
{
  double impact1_analysis[SS_IMPACT1_N];
  int i;

  if ( interactive )
    printf( "ss_touchdown_to_ds: %g %d\n", s->time, s->ss_foot_down );

  ss_impact1( s, impact1_analysis );

  if ( impact1_analysis[16] < -s->time_step*9.81 && interactive )
    {
      printf( "Negative impulse ss_impact1????\n" );
      impact1_print = 1;
    }

  /* We are allowing negative impulses here.
  if ( impact1_analysis[6] > s->time_step*9.81 )
    {
      printf( "Negative impulse 2 ss_impact1????\n" );
      impact1_print = 1;
    }
  */

  if ( impact1_print )
    {
      printf( "p, lh, lk, rh, rk: %g %g %g %g %g\n",
	      s->pitch, s->hip_angle[LEFT],
	      s->knee_angle[LEFT],
	      s->hip_angle[RIGHT],
	      s->knee_angle[RIGHT] );
      printf( "pd, lhd, lkd, rhd, rkd: %g %g %g %g %g\n",
	      s->pitchd, s->hip_angled[LEFT],
	      s->knee_angled[LEFT],
	      s->hip_angled[RIGHT],
	      s->knee_angled[RIGHT] );
      
      printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
      printf( "knees: %g %g; %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ],
	      s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
      printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	      s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
      printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
      printf( "kneesd: %g %g; %g %g\n", s->kneed[LEFT][XX], s->kneed[LEFT][ZZ],
	      s->kneed[RIGHT][XX], s->kneed[RIGHT][ZZ] );
      printf( "feetd: %g %g; %g %g\n", s->footd[LEFT][XX], s->footd[LEFT][ZZ],
	      s->footd[RIGHT][XX], s->footd[RIGHT][ZZ] );
      printf( "force: %g %g; %g %g\n", s->ground_force[LEFT][XX], s->ground_force[LEFT][ZZ],
	      s->ground_force[RIGHT][XX], s->ground_force[RIGHT][ZZ] );

      for( i = 0; i < SS_IMPACT1_N; i++ )
	{
	  printf( "%d: %g\n", i, impact1_analysis[i] );
	}
    }

  /*
  if ( s->ss_foot_down == LEFT )
    {
      ds_flag = (impact1_analysis[6] < s->time_step*s->ground_force[LEFT][ZZ]);
      printf( "ds? %d; %g %g; %g %g\n",
	      ds_flag, impact1_analysis[6], s->time_step*s->ground_force[LEFT][ZZ],
	      s->time_step, s->ground_force[LEFT][ZZ] );
    }
  else
    {
      ds_flag = (impact1_analysis[6] < s->time_step*s->ground_force[RIGHT][ZZ]);
      printf( "ds? %d; %g %g; %g %g\n",
	      ds_flag, impact1_analysis[6], s->time_step*s->ground_force[RIGHT][ZZ],
	      s->time_step, s->ground_force[RIGHT][ZZ] );
    }
  */


  for( i = 0; i < SS_NQ; i++ )
    s->ss_state[i+SS_NQ] = impact1_analysis[i];

  /*
    printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
    printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
    s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
    printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
  */
  ss_forward_kinematics( s );
  /*
    printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
    printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
    s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
    printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
  */
  if ( s->ss_foot_down == LEFT )
    ds_init_double_support_state( s, REF_LFOOT );
  else
    ds_init_double_support_state( s, REF_RFOOT );
  /*
    printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
    printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
    s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
    printf( "hipd: %g %g\n", s->hipd[XX], s->hipd[ZZ] );
  */
  impact1_print = 0;
}

/************************************************************************/
/* Assumes foot1 is on ground, and foot2 just touched down.
Two things can happen.
A) foot1 can be jerked off the ground during the impact.
B) Both feet can end up on the ground: double support.
We first assume case A is what will happen, and check the
velocity of foot1. If it is downwards, it must be case B.
A few things can go wrong.
In either case the impulse for foot2 can be downwards. Need
to understand whether this is a bug or is correct.
It may be the impulse for foot1 in the double support case
has the wrong sign. Need
to understand whether this is a bug or is correct.
*/

ss_touchdown( SIM *s )
{
  double impact1_analysis[SS_IMPACT2_N+4+4];
  double impact2_analysis[SS_IMPACT2_N+4+4];
  int ds_flag;

  if ( interactive )
    printf( "\nss_touchdown: %g %d\n", s->time, s->ss_foot_down );

  /*
  printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
  printf( "knees: %g %g; %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ],
	  s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
  printf( "feet: %g %g; %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ],
	  s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );
  */

  /* Check whether the stance foot reaction force has right sign. */
  ss_impact1( s, impact1_analysis );
  /* Check whether the stance foot lifts off. */
  ss_impact2( s, impact2_analysis );

  if ( ( impact1_analysis[6] <= 0 ) != ( impact2_analysis[20] < 0 ) && interactive )
    {
      printf( "Conflict between predictions. %g %g\n", impact1_analysis[6], impact2_analysis[20] );
      printf( "Press return to continue.\n" );
      getchar();
    }

  /* Here we implement a threshold for double support. */
  if ( s->ss_foot_down == LEFT )
    {
      ds_flag = (impact1_analysis[6] < s->time_step*s->ground_force[LEFT][ZZ]);
      if ( interactive )
	printf( "ds? %d; %g %g; %g %g\n",
		ds_flag, impact1_analysis[6], s->time_step*s->ground_force[LEFT][ZZ],
		s->time_step, s->ground_force[LEFT][ZZ] );
    }
  else
    {
      ds_flag = (impact1_analysis[6] < s->time_step*s->ground_force[RIGHT][ZZ]);
      if ( interactive )
	printf( "ds? %d; %g %g; %g %g\n",
		ds_flag, impact1_analysis[6], s->time_step*s->ground_force[RIGHT][ZZ],
		s->time_step, s->ground_force[RIGHT][ZZ] );
    }

  /* everybody agrees this is double support */
  if ( ds_flag || impact2_analysis[20] < 0 )
    {
      ss_touchdown_to_ds( s );
      return;
    }

  touchdown_to_ss( s );
}

/************************************************************************/

ss_touchdown_alt( SIM *s )
{
  double impact_analysis[SS_IMPACT2_N+4+4];

  printf( "\nss_touchdown: %g %d\n", s->time, s->ss_foot_down );

  /* Check whether the stance foot reaction force has right sign. */
  ss_impact1( s, impact_analysis );

  /* check impluse */
  if ( impact_analysis[6] <= 0 )
    ss_touchdown_to_ds( s );
  else touchdown_to_ss( s );
}

/************************************************************************/

ss_touchdown_alt2( SIM *s )
{
  double impact_analysis[SS_IMPACT2_N+4+4];

  printf( "\nss_touchdown: %g %d\n", s->time, s->ss_foot_down );

  /* Check whether the stance foot lifts off. */
  ss_impact2( s, impact_analysis );

  /* check footyd */
  if ( impact_analysis[20] < 0 )
    ss_touchdown_to_ds( s );
  else touchdown_to_ss( s );
}

/************************************************************************/

void integrate_one_time_step_in_air( SIM *s  )
{
  int i, step;
  int err; 
    /* { OK, DERIVATIVE_DISCONTINUITY, SYSTEM_LOCKED, CONSTRAINTS_ERR } */
  double errest;
  int is_bad_number = 0;

  // clear outstanding error flags
  b1gclearerr();

  for( step = 0;
       step < s->sdfast_integration_steps_per_control_step_impact;
       step++ )
    {
      b1gfmotion( &(s->time), s->sdfast_state, s->sdfast_stated,
	  s->time_step/s->sdfast_integration_steps_per_control_step_impact,
		  s->sdfast_ctol, &(s->sdfast_flag), &errest, &err );
      b1gprinterr( stderr );

      for( i = 0; i < NSTATE; i++ )
	{ // isfinite() should have worked.
	  if ( isnan( s->sdfast_state[i] ) || isinf( s->sdfast_state[i])
	       || isnan( s->sdfast_stated[i] )
	       || isinf( s->sdfast_stated[i]) ) 
	    {
	      is_bad_number = 1;
	      fprintf( stderr, "Nan detected.\n" );
	      break;
	    }
	}
	  
      if ( err > 0 )
	{
	  fprintf( stderr, "sdfast error %d\n", err );
	  break;
	}
    }

  if ( is_bad_number || (err > 0) )
    {
      s->status = CRASHED;
      return;
    }

  in_air_forward_kinematics( s );

  if ( s->foot[LEFT][ZZ] < 0 && s->foot[RIGHT][ZZ] < 0 )
    // Need to do double impacts.
    {
      fprintf( stderr, "Need to handle double impact.\n" );
      s->status = CRASHED;
      return;
      // exit( -1 );
    }
  else if ( s->foot[LEFT][ZZ] < 0 )
    {
      s->ss_foot_down = RIGHT; // Fake out collision software
      touchdown_to_ss( s );
    }
  else if ( s->foot[RIGHT][ZZ] < 0 )
    {
      s->ss_foot_down = LEFT; // Fake out collision software
      touchdown_to_ss( s );
    }
}

/************************************************************************/

void integrate_one_time_step_single_support( SIM *s  )
{
  int i, step;
  int err; 
    /* { OK, DERIVATIVE_DISCONTINUITY, SYSTEM_LOCKED, CONSTRAINTS_ERR } */
  double errest;
  int is_bad_number = 0;
  double ground_force[3];
  double ss_joint_forces[SS_NJNT][3];
  double ss_joint_torques[SS_NJNT][3];

  // clear outstanding error flags
  b1ssclearerr();

  for( step = 0;
       step < s->sdfast_integration_steps_per_control_step_impact;
       step++ )
    {
      b1ssfmotion( &(s->time), s->ss_state, s->sdfast_stated,
	   s->time_step/s->sdfast_integration_steps_per_control_step_impact,
		   SS_CTOL, &(s->sdfast_flag),
		   &errest, &err );
      b1ssprinterr(stderr);

      for( i = 0; i < SS_NSTATE; i++ )
	{ // isfinite() should have worked.
	  if ( isnan( s->sdfast_state[i] ) || isinf( s->sdfast_state[i])
	       || isnan( s->sdfast_stated[i] )
	       || isinf( s->sdfast_stated[i]) ) 
	    {
	      is_bad_number = 1;
	      fprintf( stderr, "SS: Nan detected.\n" );
	      break;
	    }
	}
	  
      if ( err > 0 )
	{
	  fprintf( stderr, "SS: sdfast error %d\n", err );
	  break;
	}
    }

  if ( is_bad_number || (err > 0) )
    {
      s->status = CRASHED;
      return;
    }

  b1ssreac( ss_joint_forces, ss_joint_torques );
  b1sstrans( SS_CALF1_ID, ss_joint_forces[SS_ANKLE1], SS_GND_ID,
	     ground_force );
  
  if ( s->ss_foot_down == LEFT )
    {
      fix_yz( s, ground_force, s->ground_force[LEFT] );
      for( i = 0; i < 3; i++ )
	s->ground_force[RIGHT][i] = 0;
    }
  else
    {
      fix_yz( s, ground_force, s->ground_force[RIGHT] );
      for( i = 0; i < 3; i++ )
	s->ground_force[LEFT][i] = 0;
    }
  ss_forward_kinematics( s );

  if ( s->ss_foot_down == LEFT )
    {
      s->foot_status[LEFT] = FOOT_ON_GROUND;
      s->foot_status[RIGHT] = FOOT_IN_AIR;
      if ( s->ground_force[LEFT][ZZ] < 0 )
	s->foot_status[LEFT] = FOOT_IN_AIR;
      else if ( fabs( s->ground_force[LEFT][XX]/s->ground_force[LEFT][ZZ] )
		> sim.friction_cone_limit )
	s->foot_status[LEFT] = FOOT_SLIDING;
	  // impact changes everything, test for stance foot sliding 
	  // after considering impact.

      if ( s->foot[RIGHT][ZZ] < EPSILON1 )
	{
	  ss_touchdown( s );
	  return;
	}
      if ( s->foot_status[LEFT] == FOOT_IN_AIR )
	{
	  transition_to_air( s );
	  return;
	}
      if ( s->foot_status[LEFT] == FOOT_SLIDING )
	{ 
	  fprintf( stderr, "SS1: Need to handle foot sliding.\n" );
	  s->status = CRASHED;
	  return;
	}
    }
  else
    {
      s->foot_status[RIGHT] = FOOT_ON_GROUND;
      s->foot_status[LEFT] = FOOT_IN_AIR;
      if ( s->ground_force[RIGHT][ZZ] < 0 )
	s->foot_status[RIGHT] = FOOT_IN_AIR;
      else if ( fabs( s->ground_force[RIGHT][XX]
		      /s->ground_force[RIGHT][ZZ] )
		> sim.friction_cone_limit )
	s->foot_status[RIGHT] = FOOT_SLIDING;
      // impact changes everything, test for stance foot sliding 
      // after considering impact.

      if ( s->foot[LEFT][ZZ] < EPSILON1 )
	{
	  ss_touchdown( s );
	  return;
	}
      if ( s->foot_status[RIGHT] == FOOT_IN_AIR )
	{
	  transition_to_air( s );
	  return;
	}
      if ( s->foot_status[RIGHT] == FOOT_SLIDING )
	{ 
	  fprintf( stderr, "SS1: Need to handle foot sliding.\n" );
	  s->status = CRASHED;
	  return;
	}
    }
}

/************************************************************************/

void integrate_one_time_step_double_support( SIM *s  )
{
  int i, step;
  int err; 
    /* { OK, DERIVATIVE_DISCONTINUITY, SYSTEM_LOCKED, CONSTRAINTS_ERR } */
  double errest;
  int is_bad_number = 0;
  double ds_joint_forces[DS_NJNT][3];
  double ds_joint_torques[DS_NJNT][3];
  double ground_force[3];

  // printf( "DS %g\n", s->time );

  // clear outstanding error flags
  b1dsclearerr();

  for( step = 0;
       step < s->sdfast_integration_steps_per_control_step_impact;
       step++ )
    {
      // printf( "DS %g\n", s->time );

      b1dsfmotion( &(s->time), s->ds_state, s->sdfast_stated,
	   s->time_step/s->sdfast_integration_steps_per_control_step_impact,
		   DS_CTOL, &(s->sdfast_flag),
		   &errest, &err );
      b1dsprinterr(stderr);

      for( i = 0; i < DS_NSTATE; i++ )
	{ // isfinite() should have worked.
	  if ( isnan( s->sdfast_state[i] ) || isinf( s->sdfast_state[i])
	       || isnan( s->sdfast_stated[i] )
	       || isinf( s->sdfast_stated[i]) ) 
	    {
	      is_bad_number = 1;
	      fprintf( stderr, "DS: Nan detected.\n" );
	      break;
	    }
	}
	  
      if ( err > 0 )
	{
	  fprintf( stderr, "DS: sdfast error %d\n", err );
	  break;
	}
    }

  if ( is_bad_number || (err > 0) )
    {
      s->status = CRASHED;
      return;
    }

  b1dsreac( ds_joint_forces, ds_joint_torques );
  b1dstrans( DS_CALF1_ID, ds_joint_forces[DS_ANKLE1], DS_GND_ID,
	     ground_force );
  fix_yz( s, ground_force, s->ground_force[LEFT] );
  b1dstrans( DS_CALF2_ID, ds_joint_forces[5], DS_GND_ID, ground_force );
  fix_yz( s, ground_force, s->ground_force[RIGHT] );
  ds_forward_kinematics( s );

  for( i = 0; i < DS_NSTATE; i++ )
    { // isfinite() should have worked.
      if ( isnan( s->sdfast_state[i] ) || isinf( s->sdfast_state[i])
	   || isnan( s->sdfast_stated[i] )
	   || isinf( s->sdfast_stated[i]) ) 
	{
	  is_bad_number = 1;
	  fprintf( stderr, "DS2: Nan detected.\n" );
	  s->status = CRASHED;
	  return;
	}
    }

  for ( i = LEFT; i <= RIGHT; i++ )
    {
      s->foot_status[i] = FOOT_ON_GROUND;
      if ( s->ground_force[i][ZZ] < 0 )
	s->foot_status[i] = FOOT_IN_AIR;
      else if ( fabs( s->ground_force[i][XX]/s->ground_force[i][ZZ] )
		> sim.friction_cone_limit )
	s->foot_status[i] = FOOT_SLIDING;
    }

  if ( s->foot_status[LEFT] == FOOT_ON_GROUND &&
       s->foot_status[RIGHT] == FOOT_IN_AIR )
    {
      /* Fix ground forces */
      s->ground_force[LEFT][XX] += s->ground_force[RIGHT][XX];
      s->ground_force[LEFT][ZZ] += s->ground_force[RIGHT][ZZ];
      s->ground_force[RIGHT][XX] = 0;
      s->ground_force[RIGHT][ZZ] = 0;
      ss_left_stance( s );
      return;
    }

  if ( s->foot_status[RIGHT] == FOOT_ON_GROUND &&
       s->foot_status[LEFT] == FOOT_IN_AIR )
    {
      /* Fix ground forces */
      s->ground_force[RIGHT][XX] += s->ground_force[LEFT][XX];
      s->ground_force[RIGHT][ZZ] += s->ground_force[LEFT][ZZ];
      s->ground_force[LEFT][XX] = 0;
      s->ground_force[LEFT][ZZ] = 0;
      ss_right_stance( s );
      return;
    }

  /* Handle one foot slipping with this, which is not quite correct. */
  if ( s->foot_status[LEFT] == FOOT_ON_GROUND &&
       s->foot_status[RIGHT] != FOOT_ON_GROUND )
    {
      ss_left_stance( s );
      return;
    }

  /* Handle one foot slipping with this, which is not quite correct. */
  if ( s->foot_status[RIGHT] == FOOT_ON_GROUND &&
       s->foot_status[LEFT] != FOOT_ON_GROUND )
    {
      ss_right_stance( s );
      return;
    }

  if ( s->foot_status[LEFT] == FOOT_IN_AIR &&
       s->foot_status[RIGHT] == FOOT_IN_AIR )
    {
      transition_to_air( s );
      return;
    }

  if ( s->foot_status[LEFT] != FOOT_ON_GROUND ||
       s->foot_status[RIGHT] != FOOT_ON_GROUND )
    {
      fprintf( stderr, "Need to handle two feet sliding in DS.\n" );
      s->status = CRASHED;
      return;
      // exit( -1 );
    }

  // All feet locked on ground
  // s->foot_status[LEFT] == FOOT_ON_GROUND &&
  // s->foot_status[RIGHT] == FOOT_ON_GROUND

#ifdef COMMENT
  /* Okay, if feet together, get weird force distribution */
  /* Fix it by averaging forces */
  if ( fabs( s->foot[RIGHT][XX] - s->foot[LEFT][XX] ) < FEET_TOGETHER
       || fabs( s->foot[RIGHT][ZZ] - s->foot[LEFT][ZZ] ) < FEET_TOGETHER )
    {
      s->ground_force[LEFT][ZZ] = s->ground_force[RIGHT][ZZ] =
	(s->ground_force[LEFT][ZZ] + s->ground_force[RIGHT][ZZ])/2;
      /*
	printf( "grf: %g %g %g %g %g %g\n", 
	s->ground_force[LEFT][XX],
	s->ground_force[LEFT][YY],
	s->ground_force[LEFT][ZZ],
	s->ground_force[RIGHT][XX],
	s->ground_force[RIGHT][YY],
	s->ground_force[RIGHT][ZZ] );
      */
    }
#endif
}

/************************************************************************/
/* This is what is called on each integration step */

void integrate_one_time_step( SIM *s  )
{ 
  
  // printf( "%g %d %d\n", s->time, s->sdfast_model, s->status );

  if( s->status == CRASHED )
    {
      s->time += s->time_step;
      return;
    }

  /* Give up if hip is too high or too low */
  if ( s->hip[ZZ] < s->hip_min || s->hip[ZZ] > s->hip_max )
    {
      s->status = CRASHED;
      s->time += s->time_step;
      return;
    }

  if ( s->sdfast_model == IN_AIR )
    integrate_one_time_step_in_air( s );
  else if ( s->sdfast_model == SINGLE_SUPPORT )
    integrate_one_time_step_single_support( s );
  else if ( s->sdfast_model == DOUBLE_SUPPORT )
    integrate_one_time_step_double_support( s );

  /*
  printf( "Press return to continue.\n" );
  getchar();
  */
}

/************************************************************************/
/* SDFAST stuff */
/************************************************************************/
/* This is where the control (hip_torque) is applied. May be called many
times per integration step at any time or state. */

void b1guforce(double t, double *q, double *u)
{
  double force[3]; // force vector in world coordinates
  double fvec[3];  // force vector in body coordinates
  double bodypos[3];
  double l_knee_torque = 0;
  double r_knee_torque = 0;

  // Apply horizontal perturbation to torso
  force[0] = sim.torso_perturbation;
  force[1] = 0.0;
  force[2] = 0.0;
  bodypos[0] = 0.0;
  bodypos[1] = 0.0;
  bodypos[2] = 0.0;
  // transform the vector into body coordinates
  b1gtrans( BODY_WORLD, force, BODY_TORSO, fvec );
  // apply to the model
  b1gpointf( BODY_TORSO, bodypos, fvec );

  // Apply knee constraints
  if ( q[Q_L_KNEE] < sim.knee_limit )
    {
      l_knee_torque = - sim.knee_limit_k * (q[Q_L_KNEE] - sim.knee_limit) 
	- sim.knee_limit_b * q[QD_L_KNEE];
      // printf( "%g %g %g ", sim.time, q[Q_L_KNEE], l_knee_torque ); 
      if ( l_knee_torque < 0 )
	l_knee_torque = 0;
      // printf( "%g\n", l_knee_torque );
    } 
  l_knee_torque += -sim.knee_torque[LEFT];
  // sim.knee_torque[LEFT] = -l_knee_torque;

  if ( q[Q_R_KNEE] < sim.knee_limit )
    {
      r_knee_torque = - sim.knee_limit_k * (q[Q_R_KNEE] - sim.knee_limit) 
	- sim.knee_limit_b * q[QD_R_KNEE];
      if ( r_knee_torque < 0 )
	r_knee_torque = 0;
    } 
  r_knee_torque += -sim.knee_torque[RIGHT];
  // sim.knee_torque[RIGHT] = -r_knee_torque;

  /* Apply joint torques */
  b1ghinget( 1, 0, -sim.hip_torque[LEFT] );
  b1ghinget( 2, 0, l_knee_torque );
  // b1ghinget( 2, 0, -sim.knee_torque[LEFT] );
  b1ghinget( 3, 0, -sim.hip_torque[RIGHT] );
  b1ghinget( 4, 0, r_knee_torque );
  // b1ghinget( 4, 0, -sim.knee_torque[RIGHT] );

  /* There are no ankle torques in the air. */
}

/*************************************************************************/

void b1ssuforce(double t, double *q, double *u)
{
  double torque[3];
  double force[3]; // force vector in world coordinates
  double force_b[3];  // force vector in body coordinates
  double offset[3];
  double l_knee_torque = 0;
  double r_knee_torque = 0;

  // Apply horizontal perturbation to torso
  force[0] = sim.torso_perturbation;
  force[1] = 0.0;
  force[2] = 0.0;
  offset[0] = 0.0;
  offset[1] = 0.0;
  offset[2] = 0.0;
  // transform the vector into body coordinates
  b1sstrans( SS_GND_ID, force, SS_TORSO_ID, force_b );
  // apply to the model
  b1sspointf( SS_TORSO_ID, offset, force_b );
  // printf( "force_b: %g %g %g\n", force_b[0], force_b[1], force_b[2] );

  if ( sim.ss_foot_down == LEFT )
    {
      // Apply knee constraints
      if ( q[SS_KNEE1] < sim.knee_limit )
	{
	  l_knee_torque = -sim.knee_limit_k * (q[SS_KNEE1] - sim.knee_limit) 
	    - sim.knee_limit_b * q[SS_KNEE1D];
	  if ( l_knee_torque < 0 )
	    l_knee_torque = 0;
	} 
      l_knee_torque += -sim.knee_torque[LEFT];

      if ( -q[SS_KNEE2] < sim.knee_limit )
	{
	  r_knee_torque = -sim.knee_limit_k * (-q[SS_KNEE2] - sim.knee_limit) 
	    - sim.knee_limit_b * -q[SS_KNEE2D];
	  if ( r_knee_torque < 0 )
	    r_knee_torque = 0;
	} 
      r_knee_torque += -sim.knee_torque[RIGHT];

      b1sshinget( SS_ANKLE1, 0, -sim.ankle_torque[LEFT] );
      // b1sshinget( SS_KNEE1, 0, -sim.knee_torque[LEFT] );
      b1sshinget( SS_KNEE1, 0, l_knee_torque );
      b1sshinget( SS_HIP1, 0, -sim.hip_torque[LEFT] );
      b1sshinget( SS_HIP2, 0, sim.hip_torque[RIGHT] );
      // b1sshinget( SS_KNEE2, 0, sim.knee_torque[RIGHT] );
      b1sshinget( SS_KNEE2, 0, -r_knee_torque );

      /* Apply ankle torques */
      /*
      torque[XX] = torque[YY] = 0;
      torque[ZZ] = -sim.ankle_torque[LEFT];
      b1ssbodyt( SS_CALF1_ID, torque );
      */
    }
  else
    {
      // Apply knee constraints
      if ( q[SS_KNEE1] < sim.knee_limit )
	{
	  r_knee_torque = -sim.knee_limit_k * (q[SS_KNEE1] - sim.knee_limit) 
	    - sim.knee_limit_b * q[SS_KNEE1D];
	  if ( r_knee_torque < 0 )
	    r_knee_torque = 0;
	} 
      r_knee_torque += -sim.knee_torque[RIGHT];

      if ( -q[SS_KNEE2] < sim.knee_limit )
	{
	  l_knee_torque = -sim.knee_limit_k * (-q[SS_KNEE2] - sim.knee_limit) 
	    - sim.knee_limit_b * -q[SS_KNEE2D];
	  if ( l_knee_torque < 0 )
	    l_knee_torque = 0;
	} 
      l_knee_torque += -sim.knee_torque[LEFT];

      b1sshinget( SS_ANKLE1, 0, -sim.ankle_torque[RIGHT] );
      // b1sshinget( SS_KNEE1, 0, -sim.knee_torque[RIGHT] );
      b1sshinget( SS_KNEE1, 0, r_knee_torque );
      b1sshinget( SS_HIP1, 0, -sim.hip_torque[RIGHT] );
      b1sshinget( SS_HIP2, 0, sim.hip_torque[LEFT] );
      // b1sshinget( SS_KNEE2, 0, sim.knee_torque[LEFT] );
      b1sshinget( SS_KNEE2, 0, -l_knee_torque );

      /* Apply ankle torques */
      /*
      torque[XX] = torque[YY] = 0;
      torque[ZZ] = -sim.ankle_torque[RIGHT];
      b1ssbodyt( SS_CALF1_ID, torque );
      */
    }
}

/*************************************************************************/

void b1dsuforce( double t, double *q, double *u )
{
  double torque[3];
  double force[3]; // force vector in world coordinates
  double force_b[3];  // force vector in body coordinates
  double offset[3];
  double l_knee_torque = 0;
  double r_knee_torque = 0;

  // Apply horizontal perturbation to torso
  force[0] = sim.torso_perturbation;
  force[1] = 0.0;
  force[2] = 0.0;
  offset[0] = 0.0;
  offset[1] = 0.0;
  offset[2] = 0.0;
  // transform the vector into body coordinates
  b1dstrans( DS_GND_ID, force, DS_TORSO_ID, force_b );
  // apply to the model
  b1dspointf( DS_TORSO_ID, offset, force_b );
  // printf( "force_b: %g %g %g\n", force_b[0], force_b[1], force_b[2] );

  // Apply knee constraints
  if ( q[DS_KNEE1] < sim.knee_limit )
    {
      l_knee_torque = - sim.knee_limit_k * (q[DS_KNEE1] - sim.knee_limit) 
	- sim.knee_limit_b * q[DS_KNEE1D];
      // printf( "%g %g %g ", sim.time, q[DS_KNEE1], l_knee_torque ); 
      if ( l_knee_torque < 0 )
	l_knee_torque = 0;
      // printf( "%g\n", l_knee_torque );
    } 
  l_knee_torque += -sim.knee_torque[LEFT];
  // sim.knee_torque[LEFT] = -l_knee_torque;

  if ( -q[DS_KNEE2] < sim.knee_limit )
    {
      r_knee_torque = - sim.knee_limit_k * (-q[DS_KNEE2] - sim.knee_limit) 
	- sim.knee_limit_b * -q[DS_KNEE2D];
      if ( r_knee_torque < 0 )
	r_knee_torque = 0;
    } 
  r_knee_torque += -sim.knee_torque[RIGHT];
  // sim.knee_torque[RIGHT] = -r_knee_torque;

  /* Apply joint torques */
  b1dshinget( DS_ANKLE1, 0, -sim.ankle_torque[LEFT] );
  // b1dshinget( DS_KNEE1, 0, -sim.knee_torque[LEFT] );
  b1dshinget( DS_KNEE1, 0, l_knee_torque );
  b1dshinget( DS_HIP1, 0, -sim.hip_torque[LEFT] );
  b1dshinget( DS_HIP2, 0, sim.hip_torque[RIGHT] );
  // b1dshinget( DS_KNEE2, 0, sim.knee_torque[RIGHT] );
  b1dshinget( DS_KNEE2, 0, -r_knee_torque );
  b1dshinget( DS_KNEE2+1, 0, -sim.ankle_torque[RIGHT] );

  /* Apply ankle torques */
  /*
  torque[XX] = torque[YY] = 0;
  torque[ZZ] = -sim.ankle_torque[LEFT];
  b1dsbodyt( DS_CALF1_ID, torque );
  torque[ZZ] = -sim.ankle_torque[RIGHT];
  b1dsbodyt( DS_CALF2_ID, torque );
  */

  /*
  printf( "F: %g %g %g %g %g %g\n",
	  -sim.hip_torque[LEFT],
	  sim.hip_torque[RIGHT],
	  -sim.knee_torque[LEFT],
	  sim.knee_torque[RIGHT],
	  -sim.ankle_torque[LEFT],
	  -sim.ankle_torque[RIGHT] );
  */
}

/************************************************************************/
/************************************************************************/
