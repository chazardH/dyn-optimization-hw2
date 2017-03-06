/************************************************************************/
/*
  dynamics.c: This is where the numerical integration and SDFAST stuff is done.
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

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "main2.h"
#include "sdfast/b1g.h"

/************************************************************************/
/* DEFINES */

#define FEET_TOGETHER (1e-3) // Feet too close together to measure tilt

#ifdef WIN32j
int isnan( double x )
{ return 0; }

int isinf( double x )
{ return 0; }
#endif

/************************************************************************/
/* GLOBAL VARIABLES */

/* Simulation structure */
SIM sim;

/************************************************************************/
/************************************************************************/
/************************************************************************/

static void forward_kinematics( SIM *s )
{
  int i;
  double zero_offset[3] = { 0.0, 0.0, 0.0 };

  b1gpos( BODY_TORSO, s->head_offset, s->head );    
  b1gpos( BODY_TORSO, s->hip_offset, s->hip );    
  b1gpos( BODY_L_CALF, s->knee_offset, &(s->knee[LEFT][0]) );    
  b1gpos( BODY_R_CALF, s->knee_offset, &(s->knee[RIGHT][0]) );    
  b1gpos( BODY_L_CALF, s->foot_offset, &(s->foot[LEFT][0]) );    
  b1gpos( BODY_R_CALF, s->foot_offset, &(s->foot[RIGHT][0]) );    

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
/* Initialize the sdfast state vector */

static void set_sdfast_state( SIM *s, double *sdfast_state )
{
  int i, j;

  for( i = 0; i < NSTATE; i++ )
    {
      s->sdfast_state[i] = sdfast_state[i];
      s->sdfast_stated[i] = 0;
    }

  for( i = 0, j = NQ; i < NU; i++, j++ )
    { 
      s->sdfast_stated[i] = s->sdfast_state[j];
    }

  b1gstate( 0.0, s->sdfast_state, s->sdfast_stated );
  s->sdfast_flag = 1;

  forward_kinematics( s );

  s->status = OK;
}

/************************************************************************/
// For debugging

b1gdump()
{
  double value;
  double vector[3];
  double inertia[3][3];

  b1ggetmass( BODY_TORSO, &value );
  printf( "Torso mass: %g\n", value );
  b1ggetmass( BODY_L_THIGH, &value );
  printf( "L Thigh mass: %g\n", value );
  b1ggetmass( BODY_R_THIGH, &value );
  printf( "R Thigh mass: %g\n", value );
  b1ggetmass( BODY_L_CALF, &value );
  printf( "L Calf mass: %g\n", value );
  b1ggetmass( BODY_R_CALF, &value );
  printf( "R Calf mass: %g\n", value );

  b1ggetiner( BODY_TORSO, inertia );
  printf( "Torso I: %g\n", inertia[YY][YY] );
  b1ggetiner( BODY_L_THIGH, inertia );
  printf( "L Thigh I: %g\n", inertia[YY][YY] );
  b1ggetiner( BODY_R_THIGH, inertia );
  printf( "R Thigh I: %g\n", inertia[YY][YY] );
  b1ggetiner( BODY_L_CALF, inertia );
  printf( "L Calf I: %g\n", inertia[YY][YY] );
  b1ggetiner( BODY_R_CALF, inertia );
  printf( "R Calf I: %g\n", inertia[YY][YY] );

  b1ggetbtj( BODY_L_THIGH, vector );
  printf( "L_THIGH BTJ: %g %g %g\n", vector[0],
	  vector[1], vector[2] );
  b1ggetbtj( BODY_R_THIGH, vector );
  printf( "R_THIGH BTJ: %g %g %g\n", vector[0],
	  vector[1], vector[2] );
  b1ggetbtj( BODY_L_CALF, vector );
  printf( "L_Calf BTJ: %g %g %g\n", vector[0],
	  vector[1], vector[2] );
  b1ggetbtj( BODY_R_CALF, vector );
  printf( "R_Calf BTJ: %g %g %g\n", vector[0],
	  vector[1], vector[2] );

  b1ggetitj( BODY_L_THIGH, vector );
  printf( "L_THIGH ITJ: %g %g %g\n", vector[0],
	  vector[1], vector[2] );
  b1ggetitj( BODY_R_THIGH, vector );
  printf( "R_THIGH ITJ: %g %g %g\n", vector[0],
	  vector[1], vector[2] );
  b1ggetitj( BODY_L_CALF, vector );
  printf( "L_Calf ITJ: %g %g %g\n", vector[0],
	  vector[1], vector[2] );
  b1ggetitj( BODY_R_CALF, vector );
  printf( "R_Calf ITJ: %g %g %g\n", vector[0],
	  vector[1], vector[2] );
}

/************************************************************************/

void debug_positions( SIM *s )
{

  printf( "head: %g %g\n", s->head[XX], s->head[ZZ] );
  printf( "torso: %g %g\n", s->torso[XX], s->torso[ZZ] );
  printf( "hip: %g %g\n", s->hip[XX], s->hip[ZZ] );
  printf( "lthigh: %g %g\n", s->thigh[LEFT][XX], s->thigh[LEFT][ZZ] );
  printf( "lknee: %g %g\n", s->knee[LEFT][XX], s->knee[LEFT][ZZ] );
  printf( "lcalf: %g %g\n", s->calf[LEFT][XX], s->calf[LEFT][ZZ] );
  printf( "lfoot: %g %g\n", s->foot[LEFT][XX], s->foot[LEFT][ZZ] );
  printf( "rthigh: %g %g\n", s->thigh[RIGHT][XX], s->thigh[RIGHT][ZZ] );
  printf( "rknee: %g %g\n", s->knee[RIGHT][XX], s->knee[RIGHT][ZZ] );
  printf( "rcalf: %g %g\n", s->calf[RIGHT][XX], s->calf[RIGHT][ZZ] );
  printf( "rfoot: %g %g\n", s->foot[RIGHT][XX], s->foot[RIGHT][ZZ] );

  printf( "pitch: %g\n", s->pitch );
  printf( "hips: %g %g\n", s->hip_angle[LEFT], s->hip_angle[RIGHT] );
  printf( "knees: %g %g\n", s->knee_angle[LEFT], s->knee_angle[RIGHT] );
  printf( "ankles: %g %g\n", s->ankle_angle[LEFT], s->ankle_angle[RIGHT] );

  printf( "\n" );
  printf( "head: %g %g\n", s->head[XX] - s->foot[LEFT][XX], s->head[ZZ] );
  printf( "torso: %g %g\n", s->torso[XX] - s->foot[LEFT][XX], s->torso[ZZ] );
  printf( "hip: %g %g\n", s->hip[XX] - s->foot[LEFT][XX], s->hip[ZZ] );
  printf( "lthigh: %g %g\n", s->thigh[LEFT][XX] - s->foot[LEFT][XX],
	  s->thigh[LEFT][ZZ] - s->foot[LEFT][ZZ] );
  printf( "lknee: %g %g\n", s->knee[LEFT][XX] - s->foot[LEFT][XX],
	  s->knee[LEFT][ZZ] - s->foot[LEFT][ZZ] );
  printf( "lcalf: %g %g\n", s->calf[LEFT][XX] - s->foot[LEFT][XX],
	  s->calf[LEFT][ZZ] - s->foot[LEFT][ZZ] );
  printf( "lfoot: %g %g\n", s->foot[LEFT][XX] - s->foot[LEFT][XX],
	  s->foot[LEFT][ZZ] - s->foot[LEFT][ZZ] );
  printf( "rthigh: %g %g\n", s->thigh[RIGHT][XX] - s->foot[LEFT][XX],
	  s->thigh[RIGHT][ZZ] - s->foot[LEFT][ZZ] );
  printf( "rknee: %g %g\n", s->knee[RIGHT][XX] - s->foot[LEFT][XX],
	  s->knee[RIGHT][ZZ] - s->foot[LEFT][ZZ] );
  printf( "rcalf: %g %g\n", s->calf[RIGHT][XX] - s->foot[LEFT][XX],
	  s->calf[RIGHT][ZZ] - s->foot[LEFT][ZZ] );
  printf( "rfoot: %g %g\n", s->foot[RIGHT][XX] - s->foot[LEFT][XX],
	  s->foot[RIGHT][ZZ] - s->foot[LEFT][ZZ] );

  exit( 0 );
}

/************************************************************************/
/* Initialize sdfast  */

void init_dynamics( SIM *s )
{
  int i, j;
  double vector[3];
  double inertia[3][3];

  if ( NSTATE > MAX_N_SDFAST_STATE )
    {
      fprintf( stderr, 
	       "Need to increast MAX_N_SDFAST_STATE (%d) to be at least %d\n",
	       MAX_N_SDFAST_STATE, NSTATE );
      exit( -1 );
    }

  s->status = OK;

  // b1gdump();

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

  // b1gdump();

  b1ginit(); /* initialize SDFAST model */

  b1gstab( 2.0*s->sdfast_baumgarte, s->sdfast_baumgarte*s->sdfast_baumgarte ); 

  for( i = 0; i < NSTATE; i++ )
    s->sdfast_state[i] = 0;

  b1gprinterr(stderr);
}

/************************************************************************/

void init_state_one_foot_on_ground( SIM *s )
{
  double min;

  s->status = OK;

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

  set_sdfast_state( s, s->sdfast_state );

  min = s->foot[LEFT][ZZ];
  if ( min < s->foot[RIGHT][ZZ] )
    min = s->foot[RIGHT][ZZ];
  s->sdfast_state[Q_Z] = s->sdfast_state[Q_Z] - (min - s->ground_level);

  set_sdfast_state( s, s->sdfast_state );
}

/************************************************************************/

void init_state_two_feet_on_ground( SIM *s )
{
  double min;
  double offset;

  s->status = OK;

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

  set_sdfast_state( s, s->sdfast_state );

  /* Level the two feet */
  /* Need to handle case where two feet together */
  if ( fabs( s->foot[RIGHT][XX] - s->foot[LEFT][XX] ) > FEET_TOGETHER )
    {
      if ( s->foot[RIGHT][XX] - s->foot[LEFT][XX] > 0 )
	offset = atan2( s->foot[RIGHT][ZZ] - s->foot[LEFT][ZZ],
			s->foot[RIGHT][XX] - s->foot[LEFT][XX] );
      else
	offset = atan2( s->foot[LEFT][ZZ] - s->foot[RIGHT][ZZ],
			s->foot[LEFT][XX] - s->foot[RIGHT][XX] );

      /*
	printf( "two feet on ground: %g %g %g %g: %g\n", 
	s->foot[RIGHT][ZZ], s->foot[LEFT][ZZ],
	s->foot[RIGHT][XX], s->foot[LEFT][XX], offset );
      */

      s->sdfast_state[Q_PITCH] += offset;
      
      set_sdfast_state( s, s->sdfast_state );
    }

  min = s->foot[LEFT][ZZ];
  if ( min < s->foot[RIGHT][ZZ] )
    min = s->foot[RIGHT][ZZ];
  s->sdfast_state[Q_Z] = s->sdfast_state[Q_Z] - (min - s->ground_level);

  set_sdfast_state( s, s->sdfast_state );

  /*
  printf( "%g %g\n", s->foot[RIGHT][ZZ], s->foot[LEFT][ZZ] );
  */
}

/************************************************************************/
/* This is what is called on each integration step */

void integrate_one_time_step( SIM *s )
{ 
  int i, step;
  int err; 
    /* { OK, DERIVATIVE_DISCONTINUITY, SYSTEM_LOCKED, CONSTRAINTS_ERR } */
  double errest;
  int is_bad_number = 0;

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

  // clear outstanding error flags
  b1gclearerr();

  for( step = 0; step < s->sdfast_integration_steps_per_control_step; step++ )
    {
      b1gfmotion( &(s->time), s->sdfast_state, s->sdfast_stated,
		  s->time_step/s->sdfast_integration_steps_per_control_step,
		  s->sdfast_ctol, &(s->sdfast_flag), &errest, &err );
      b1gprinterr(stderr);

      for( i = 0; i < NSTATE; i++ )
	{ // isfinite() should have worked.
	  if ( isnan( s->sdfast_state[i] ) || isinf( s->sdfast_state[i])
	       || isnan( s->sdfast_stated[i] ) || isinf( s->sdfast_stated[i]) ) 
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

  forward_kinematics( s );

  /*
  printf( "Press return to continue.\n" );
  getchar();
  */
}

/****************************************************************/

// Utility function to compute the ground force function.
static 
void compute_ground_force(double contactpos[3],  // contact position wrt world
			  double contact_zero[3], // original contact point.
			  double contactvel[3],  // contact velocity wrt world
			  double bodypos[3],     // contact point wrt body
			  int body,            
			  double force[3]) // resultant force vector wrt world
			  
{
  int i;
  double fvec[3];  // ground force vector in body coordintes

  // force only exists if foot is below ground level
  if ( contactpos[2] >= sim.ground_level )
    {
      for( i = 0; i < 3; i++ )
	{
	  force[i] = 0;
	  contact_zero[i] = contactpos[i];
	}
      return;
    }

  // Compute the desired ground force vector in the world coordinates.

  // Initialize original contact point. */
  if ( contact_zero[ZZ] > sim.ground_level )
    {
      for( i = 0; i < 2; i++ )
	contact_zero[i] = contactpos[i];
      contact_zero[ZZ] = sim.ground_level;
    }

  force[XX] = (contact_zero[XX] - contactpos[XX])*sim.gnd_k_x
    - contactvel[XX] * sim.gnd_b_x;
  force[YY] = 0.0;   // not relevant
  force[ZZ] = (sim.ground_level - contactpos[ZZ]) * sim.gnd_k_z 
    - contactvel[ZZ] * sim.gnd_b_z;

  // ground can't pull down
  if ( force[ZZ] <= 0 ) 
    {
      force[XX] = 0.0;
      force[YY] = 0.0;
      force[ZZ] = 0.0;
      /* Simulate a slip: reset contact point. */
      for( i = 0; i < 3; i++ )
	contact_zero[i] = contactpos[i];
    }
  // check for slipping caused by friction cone violation.
  else if ( fabs( force[XX]/force[ZZ] ) > sim.friction_cone_limit )
    {
      // This slip is probably overly strong. There would be some
      // force and only partial motion in reality.
      force[XX] = 0.0;
      /* Simulate a slip: reset contact point. */
      for( i = 0; i < 3; i++ )
	contact_zero[i] = contactpos[i];
    }

  // transform the vector into body coordinates
  b1gtrans(BODY_WORLD, force, body, fvec);
  
  // apply to the model
  b1gpointf(body, bodypos, fvec);

}

/************************************************************************/
/* SDFAST stuff */
/************************************************************************/
/* This is where the control (joint torques) are applied. May be called many
times per integration step at any time or state. */

void b1guforce( double t, double *q, double *u )
{
  int i;
  double force[3]; // force vector in world coordinates
  double fvec[3];  // force vector in body coordinates
  double bodypos[3];
  double torque[3];
  double l_knee_torque = 0;
  double r_knee_torque = 0;
  double foot[2][3];
  double footd[2][3];

  // find the position and velocity of each foot point in the world frame
  b1gpos( BODY_L_CALF, sim.foot_offset, foot[LEFT] );    
  b1gpos( BODY_R_CALF, sim.foot_offset, foot[RIGHT] );    

  b1gvel( BODY_L_CALF, sim.foot_offset, footd[LEFT] );    
  b1gvel( BODY_R_CALF, sim.foot_offset, footd[RIGHT] );    

  // Apply ground forces to each foot as point forces.
  compute_ground_force( foot[LEFT], sim.foot_zero[LEFT], footd[LEFT],
			sim.foot_offset, BODY_L_CALF, sim.ground_force[LEFT] );
  compute_ground_force( foot[RIGHT], sim.foot_zero[RIGHT], footd[RIGHT],
			sim.foot_offset, BODY_R_CALF, sim.ground_force[RIGHT] );

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

  /* Apply ankle torques */
  torque[XX] = torque[ZZ] = 0;
  torque[YY] = sim.ankle_torque[LEFT];
  b1gbodyt( BODY_L_CALF, torque );
  torque[YY] = sim.ankle_torque[RIGHT];
  b1gbodyt( BODY_R_CALF, torque );
}

/************************************************************************/
/************************************************************************/
/************************************************************************/
/************************************************************************/
