/*****************************************************************************/
/*
  controller.c: control strategy.
*/
/*****************************************************************************/

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "../useful/trajectory/trajectory.h"
#include "main.h"
#include "main2.h"

/*****************************************************************************/

/* Controller states */
#define UNKNOWN_STATE 0
#define WAITING 1 // waiting for launch
#define LAUNCH1 2 // first phase of launch (beginning walking)
#define LAUNCH2 3 // second phase of launch
#define L_SWING 4 // left foot swinging
#define RL_STANCE 5 // double support, left foot leading
#define R_SWING 6 // right foot swinging
#define LR_STANCE 7 // double support, right foot leading
#define STOP 8 // stop walking

/* Servo modes: joint can be position controlled or torque controlled */
#define PD_MODE 0
#define TORQUE_MODE 1

/*****************************************************************************/
/* Trajectories */

DOFS desired_trajectory[N_BIPED_DOFS];
DOFS current_desireds[N_BIPED_DOFS];

/*****************************************************************************/
/*****************************************************************************/

void reset_score( SIM *s )
{
  s->summed_score = 0.0;
  s->discounted_score = 0.0;
  s->discount_to_power = 1.0;
  s->crashed_score = 0; /* amount of time walker is crashed. */
  s->torque_score = 0; /* sum of torque penalties */
  s->knee_score = 0; /* penalty on bending knee wrong way. */
  s->speed_score = 0;
  s->clearance_score = 0;
  s->f_x_score = 0;
  s->f_z_score = 0;
  s->pitchd_score = 0;
}

/*****************************************************************************/
/*
Cost function.

Could normalize torque and other penalties w.r.t. distance
Could penalize deviations from symmetry (if necessary).
*/

double one_step_cost( SIM *s )
{
  int i;
  double total_z_force;

  s->one_step_total_score = 0;

  // sum of squared joint torques
  s->one_step_torque_score = 0;
  s->one_step_torque_score += s->hip_command[LEFT]*s->hip_command[LEFT];
  s->one_step_torque_score += s->hip_command[RIGHT]*s->hip_command[RIGHT];
  s->one_step_torque_score += s->knee_command[LEFT]*s->knee_command[LEFT];
  s->one_step_torque_score += s->knee_command[RIGHT]*s->knee_command[RIGHT];
  s->one_step_torque_score += s->ankle_command[LEFT]*s->ankle_command[LEFT];
  s->one_step_torque_score += s->ankle_command[RIGHT]*s->ankle_command[RIGHT];
  s->one_step_torque_score *= s->torque_penalty_weight;

  // penalty on knee going backwards
  s->one_step_knee_score = 0;
  if ( s->knee_angle_d[LEFT] > 0 )
    s->one_step_knee_score += s->knee_angle_d[LEFT]*s->knee_angle_d[LEFT];
  if ( s->knee_angle_d[RIGHT] > 0 )
    s->one_step_knee_score += s->knee_angle_d[RIGHT]*s->knee_angle_d[RIGHT];

  // penalty on head movement deviating from desired speed
  // could be hipd[XX] instead
  s->one_step_speed_score = (s->desired_speed - s->headd[XX])
    *(s->desired_speed - s->headd[XX])
    *s->speed_penalty_weight;

  // penalty on inadequate ground clearance
  s->one_step_clearance_score = 0;
  if ( s->ground_force[LEFT][ZZ] <= 0 && s->foot[LEFT][ZZ] < s->clearance )
    s->one_step_clearance_score += (s->foot[LEFT][ZZ] - s->clearance)
      *(s->foot[LEFT][ZZ] - s->clearance);
  if ( s->ground_force[RIGHT][ZZ] <= 0 && s->foot[RIGHT][ZZ] < s->clearance )
    s->one_step_clearance_score += (s->foot[RIGHT][ZZ] - s->clearance)
      *(s->foot[RIGHT][ZZ] - s->clearance);
  s->one_step_clearance_score *= s->clearance_penalty_weight;

  // penalty on excessive foot forces
  s->one_step_f_x_score = 0;
  for ( i = LEFT; i <= RIGHT; i++ )
    s->one_step_f_x_score += s->ground_force[i][XX]*s->ground_force[i][XX];
  s->one_step_f_x_score *= s->f_x_penalty_weight;
  total_z_force = s->ground_force[LEFT][ZZ] + s->ground_force[RIGHT][ZZ];
  s->one_step_f_z_score = (total_z_force - s->total_weight_in_newtons)*
    (total_z_force - s->total_weight_in_newtons);
  s->one_step_f_z_score *= s->f_z_penalty_weight;

  // penalty on torso angle
  s->one_step_pitchd_score = s->pitchd*s->pitchd*s->pitchd_penalty_weight;

  // penalty on crashing
  s->one_step_crashed_score = 0;
  if ( s->status == CRASHED )
    s->one_step_crashed_score = 
      (s->duration - s->time)*s->crashed_penalty_weight;

  s->one_step_total_score = 
    s->one_step_torque_score 
    + s->one_step_knee_score
    + s->one_step_speed_score
    + s->one_step_clearance_score
    + s->one_step_f_x_score
    + s->one_step_f_z_score
    + s->one_step_pitchd_score
    + s->one_step_crashed_score;
 // s->one_step_total_score = swing_time
  return s->one_step_total_score;
}

/*****************************************************************************/

void update_score( SIM *s )
{
  double total_score;

  one_step_cost( s );

  s->summed_score += s->one_step_total_score;
  s->discounted_score += s->discount_to_power * s->one_step_total_score;
  s->discount_to_power = s->discount * s->discount_to_power;
  /* This is useful for debugging */
  s->torque_score += s->one_step_torque_score;
  s->knee_score += s->one_step_knee_score;
  s->speed_score += s->one_step_speed_score;
  s->clearance_score += s->one_step_clearance_score;
  s->f_x_score += s->one_step_f_x_score;
  s->f_z_score += s->one_step_f_z_score;
  s->pitchd_score += s->one_step_pitchd_score;
  s->crashed_score += s->one_step_crashed_score;
}

/*****************************************************************************/

double get_score( SIM *s )
{
  printf( "cost: %g, %g; crashed %g (%d); torque %g; speed %g; knee %g; clearance %g; f_x %g; f_z %g; pitchd %g\n",
	  s->summed_score, s->discounted_score, s->crashed_score, s->status,
	  s->torque_score, s->speed_score, s->knee_score, s->clearance_score,
	  s->f_x_score, s->f_z_score, s->pitchd_score );
  return s->summed_score;
}

/*****************************************************************************/
/* call this many times to restart a controller, called by reinint_sim() */

int reinit_controller( SIM *s )
{

  s->controller_state = WAITING;
  s->state_start_time = s->time;
  s->state_elapsed_time = 0.0;

  s->hip_angle_d[LEFT] = s->hip_angle[LEFT];
  s->hip_angle_d[RIGHT] = s->hip_angle[RIGHT];
  s->knee_angle_d[LEFT] = s->knee_angle[LEFT];
  s->knee_angle_d[RIGHT] = s->knee_angle[RIGHT];
  s->ankle_angle_d[LEFT] = s->ankle_angle[LEFT];
  s->ankle_angle_d[RIGHT] = s->ankle_angle[RIGHT];
  s->hip_angled_d[LEFT] = s->hip_angled[LEFT];
  s->hip_angled_d[RIGHT] = s->hip_angled[RIGHT];
  s->knee_angled_d[LEFT] = s->knee_angled[LEFT];
  s->knee_angled_d[RIGHT] = s->knee_angled[RIGHT];
  /* sidestep angle perturbation for now
  s->ankle_angled_d[LEFT] = s->ankle_angled[LEFT];
  s->ankle_angled_d[RIGHT] = s->ankle_angled[RIGHT];
  */
  s->ankle_angled_d[LEFT] = 0;
  s->ankle_angled_d[RIGHT] = 0;

  s->hip_servo_mode[LEFT] = PD_MODE;
  s->hip_servo_mode[RIGHT] = PD_MODE;
  s->knee_servo_mode[LEFT] = PD_MODE;
  s->knee_servo_mode[RIGHT] = PD_MODE;
  s->ankle_servo_mode[LEFT] = PD_MODE;
  s->ankle_servo_mode[RIGHT] = PD_MODE;

  s->hip_command_ff[LEFT] = 0;
  s->hip_command_ff[RIGHT] = 0;
  s->knee_command_ff[LEFT] = 0;
  s->knee_command_ff[RIGHT] = 0;
  s->ankle_command_ff[LEFT] = 0;
  s->ankle_command_ff[RIGHT] = 0;

  clear_all_knots( desired_trajectory, N_BIPED_DOFS );
  clear_all_knots( current_desireds, N_BIPED_DOFS );

  /* add actual position as knot to current desireds */
  add_knot_point( current_desireds, L_HIP, QUINTIC_SPLINE, 0.0, 
		  s->hip_angle_d[LEFT], s->hip_angled_d[LEFT], 0.0 );
  add_knot_point( current_desireds, L_KNEE, QUINTIC_SPLINE, 0.0,
		  s->knee_angle_d[LEFT], s->knee_angled_d[LEFT], 0.0 );
  add_knot_point( current_desireds, L_ANKLE, QUINTIC_SPLINE, 0.0,
		  s->ankle_angle_d[LEFT], s->ankle_angled_d[LEFT], 0.0 );
  add_knot_point( current_desireds, R_HIP, QUINTIC_SPLINE, 0.0, 
		  s->hip_angle_d[RIGHT], s->hip_angled_d[RIGHT], 0.0 );
  add_knot_point( current_desireds, R_KNEE, QUINTIC_SPLINE, 0.0, 
		  s->knee_angle_d[RIGHT], s->knee_angled_d[RIGHT], 0.0 );
  add_knot_point( current_desireds, R_ANKLE, QUINTIC_SPLINE, 0.0, 
		  s->ankle_angle_d[RIGHT], s->ankle_angled_d[RIGHT], 0.0 );

  reset_score( s );

  return 0;
}

/*****************************************************************************/
/* call this many times to restart a simulation */

int reinit_sim( SIM *s )
{
  int i;
  double min;

  srand( s->rand_seed );

  s->time = 0.0;

  s->hip[XX] = 0;
  s->hip[ZZ] = 0;
  s->pitch = 0;
  s->hipd[XX] = 0;
  s->hipd[ZZ] = 0;
  s->pitchd = 0;

  s->hip_angle[LEFT] = 0;
  s->hip_angle[RIGHT] = 0;
  s->knee_angle[LEFT] = 0;
  s->knee_angle[RIGHT] = 0;
  s->hip_angled[LEFT] = 0;
  s->hip_angled[RIGHT] = 0;
  s->knee_angled[LEFT] = 0;
  s->knee_angled[RIGHT] = 0;

  /* Make it fall */
  // s->pitchd = 1.0;

  /* Indicate foot zero positions not set */
  s->foot_zero[LEFT][ZZ] = 1.0;
  s->foot_zero[RIGHT][ZZ] = 1.0;

  /*
  init_state_one_foot_on_ground( s );
  */
  init_state_two_feet_on_ground( s );

  reinit_controller( s );

}

/*****************************************************************************/
/* Call this once to do one time operations like memory allocation */

int init_sim( SIM *s )
{
  init_dynamics( s );
  init_trajectory( desired_trajectory, N_BIPED_DOFS );
  init_trajectory( current_desireds, N_BIPED_DOFS );
  reinit_sim( s );
}

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*
Each joint follows a desired trajectory determined by parameters
*/

void
run_trajectory( SIM *s, double time )
{
  double  position, velocity, acceleration;

  if ( !lookup_spline3( desired_trajectory, L_HIP, time,
			current_desireds,
			&position,
			&velocity,
			&acceleration ) )
    {
      printf( "Lookup failure.\n" );
      exit( -1 );
    }
  /* update current desired position */
  set_first_knot_to( current_desireds, L_HIP, QUINTIC_SPLINE, time,
		     position, velocity, acceleration );
  s->hip_angle_d[LEFT] = position;
  s->hip_angled_d[LEFT] = velocity;

  if ( !lookup_spline3( desired_trajectory, L_KNEE, time,
			current_desireds,
			&position,
			&velocity,
			&acceleration ) )
    {
      printf( "Lookup failure.\n" );
      exit( -1 );
    }
  /* update current desired position */
  set_first_knot_to( current_desireds, L_KNEE, QUINTIC_SPLINE, time,
		     position, velocity, acceleration );
  s->knee_angle_d[LEFT] = position;
  s->knee_angled_d[LEFT] = velocity;
#ifdef KNEE_ZERO_LIMIT
  if ( s->knee_angle_d[LEFT] > 0 )
    {
      s->knee_angle_d[LEFT] = 0;
      if ( s->knee_angled_d[LEFT] > 0 )
	s->knee_angled_d[LEFT] = 0;
    }
#endif    

  if ( !lookup_spline3( desired_trajectory, L_ANKLE, time,
			current_desireds,
			&position,
			&velocity,
			&acceleration ) )
    {
      printf( "Lookup failure.\n" );
      exit( -1 );
    }
  /* update current desired position */
  set_first_knot_to( current_desireds, L_ANKLE, QUINTIC_SPLINE, time,
		     position, velocity, acceleration );
  s->ankle_angle_d[LEFT] = position;
  s->ankle_angled_d[LEFT] = velocity;

  if ( !lookup_spline3( desired_trajectory, R_HIP, time,
			current_desireds,
			&position,
			&velocity,
			&acceleration ) )
    {
      printf( "Lookup failure.\n" );
      exit( -1 );
    }

  /* update current desired position */
  set_first_knot_to( current_desireds, R_HIP, QUINTIC_SPLINE, time,
		     position, velocity, acceleration );
  s->hip_angle_d[RIGHT] = position;
  s->hip_angled_d[RIGHT] = velocity;

  if ( !lookup_spline3( desired_trajectory, R_KNEE, time,
			current_desireds,
			&position,
			&velocity,
			&acceleration ) )
    {
      printf( "Lookup failure.\n" );
      exit( -1 );
    }
  /* update current desired position */
  set_first_knot_to( current_desireds, R_KNEE, QUINTIC_SPLINE, time,
		     position, velocity, acceleration );
  s->knee_angle_d[RIGHT] = position;
  s->knee_angled_d[RIGHT] = velocity;
#ifdef KNEE_ZERO_LIMIT
  if ( s->knee_angle_d[RIGHT] > 0 )
    {
      s->knee_angle_d[RIGHT] = 0;
      if ( s->knee_angled_d[RIGHT] > 0 )
	s->knee_angled_d[RIGHT] = 0;
    }
#endif

  if ( !lookup_spline3( desired_trajectory, R_ANKLE, time,
			current_desireds,
			&position,
			&velocity,
			&acceleration ) )
    {
      printf( "Lookup failure.\n" );
      exit( -1 );
    }
  /* update current desired position */
  set_first_knot_to( current_desireds, R_ANKLE, QUINTIC_SPLINE, time,
		     position, velocity, acceleration );
  s->ankle_angle_d[RIGHT] = position;
  s->ankle_angled_d[RIGHT] = velocity;
}

/*****************************************************************************/
/*
Run either PID servo or torque control for each joint
*/

void
run_servos( SIM *s )
{
  int i;

  s->hip_integrated_error[LEFT] 
    += s->time_step*(s->hip_angle[LEFT] - s->hip_angle_d[LEFT]);
  s->hip_integrated_error[RIGHT] 
    += s->time_step*(s->hip_angle[RIGHT] - s->hip_angle_d[RIGHT]);
  s->knee_integrated_error[LEFT] 
    += s->time_step*(s->knee_angle[LEFT] - s->knee_angle_d[LEFT]);
  s->knee_integrated_error[RIGHT] 
    += s->time_step*(s->knee_angle[RIGHT] - s->knee_angle_d[RIGHT]);
  s->ankle_integrated_error[LEFT] 
    += s->time_step*(s->ankle_angle[LEFT] - s->ankle_angle_d[LEFT]);
  s->ankle_integrated_error[RIGHT] 
    += s->time_step*(s->ankle_angle[RIGHT] - s->ankle_angle_d[RIGHT]);

  if ( s->hip_servo_mode[LEFT] == PD_MODE )
    {
      s->hip_command[LEFT] = -s->pos_gains[L_HIP][L_HIP]
	*( s->hip_angle[LEFT] - s->hip_angle_d[LEFT] )
	- s->vel_gains[L_HIP][L_HIP]
	*( s->hip_angled[LEFT] - s->hip_angled_d[LEFT] )
	- s->int_gains[L_HIP][L_HIP]*s->hip_integrated_error[LEFT]
	+ s->hip_command_ff[LEFT];
    }

  if ( s->hip_servo_mode[RIGHT] == PD_MODE )
    {
      s->hip_command[RIGHT] = -s->pos_gains[R_HIP][R_HIP]
	*( s->hip_angle[RIGHT] - s->hip_angle_d[RIGHT] )
	- s->vel_gains[R_HIP][R_HIP]
	*( s->hip_angled[RIGHT] - s->hip_angled_d[RIGHT] )
	- s->int_gains[R_HIP][R_HIP]*s->hip_integrated_error[RIGHT]
	+ s->hip_command_ff[RIGHT];
    }

  if ( s->knee_servo_mode[LEFT] == PD_MODE )
    {
      s->knee_command[LEFT] = -s->pos_gains[L_KNEE][L_KNEE]
	*( s->knee_angle[LEFT] - s->knee_angle_d[LEFT] )
	- s->vel_gains[L_KNEE][L_KNEE]
	*( s->knee_angled[LEFT] - s->knee_angled_d[LEFT] )
	- s->int_gains[L_KNEE][L_KNEE]*s->knee_integrated_error[LEFT]
	+ s->knee_command_ff[LEFT];
    }

  if ( s->knee_servo_mode[RIGHT] == PD_MODE )
    {
      s->knee_command[RIGHT] = -s->pos_gains[R_KNEE][R_KNEE]
	*( s->knee_angle[RIGHT] - s->knee_angle_d[RIGHT] )
	- s->vel_gains[R_KNEE][R_KNEE]
	*( s->knee_angled[RIGHT] - s->knee_angled_d[RIGHT] )
	- s->int_gains[R_KNEE][R_KNEE]*s->knee_integrated_error[RIGHT]
	+ s->knee_command_ff[RIGHT];
    }

  if ( s->ankle_servo_mode[LEFT] == PD_MODE )
    {
      s->ankle_command[LEFT] = -s->pos_gains[L_ANKLE][L_ANKLE]
	*( s->ankle_angle[LEFT] - s->ankle_angle_d[LEFT] )
	- s->vel_gains[L_ANKLE][L_ANKLE]
	*( s->ankle_angled[LEFT] - s->ankle_angled_d[LEFT] )
	- s->int_gains[L_ANKLE][L_ANKLE]*s->ankle_integrated_error[LEFT]
	+ s->ankle_command_ff[LEFT];
    }

  if ( s->ankle_servo_mode[RIGHT] == PD_MODE )
    {
      s->ankle_command[RIGHT] = -s->pos_gains[R_ANKLE][R_ANKLE]
	*( s->ankle_angle[RIGHT] - s->ankle_angle_d[RIGHT] )
	- s->vel_gains[R_ANKLE][R_ANKLE]
	*( s->ankle_angled[RIGHT] - s->ankle_angled_d[RIGHT] )
	- s->int_gains[R_ANKLE][R_ANKLE]*s->ankle_integrated_error[RIGHT]
	+ s->ankle_command_ff[RIGHT];
      /*
      printf( "%g: %g %g %g\n",
	      s->ankle_command[RIGHT], s->pos_gains[R_ANKLE][R_ANKLE],
	      s->ankle_angle[RIGHT], s->ankle_angle_d[RIGHT] );
      */
    }

  /* Limit ankle torques */
  for ( i = LEFT; i <= RIGHT; i++ )
    {
      if ( s->ground_force <= 0 || s->foot[i][ZZ] > s->ground_level )
	s->ankle_command[i] = 0;
      else if ( s->ankle_command[i]/s->ground_force[i][ZZ] < s->zmp_x_min )
	s->ankle_command[i] = s->zmp_x_min*s->ground_force[i][ZZ];
      else if ( s->ankle_command[i]/s->ground_force[i][ZZ] > s->zmp_x_max )
	s->ankle_command[i] = s->zmp_x_max*s->ground_force[i][ZZ];
    }

  for ( i = LEFT; i <= RIGHT; i++ )
    {
      s->hip_torque[i] = s->hip_command[i];
      s->knee_torque[i] = s->knee_command[i];
      s->ankle_torque[i] = s->ankle_command[i];
    }

  /* Apply random noise */
  /* skip this for now.
  for ( i = LEFT; i <= RIGHT; i++ )
    {
      s->hip_torque[i] += s->rand_scale*((2.0*rand())/RAND_MAX - 1.0);
      s->knee_torque[i] += s->rand_scale*((2.0*rand())/RAND_MAX - 1.0);
      s->ankle_torque[i] += s->rand_scale*((2.0*rand())/RAND_MAX - 1.0);
    }
  */

  /* Check for saturation. */
  for ( i = LEFT; i <= RIGHT; i++ )
    {
      if ( s->hip_torque[i] > s->hip_torque_max[i] )
	s->hip_torque[i] = s->hip_torque_max[i];
      if ( s->hip_torque[i] < s->hip_torque_min[i] )
	s->hip_torque[i] = s->hip_torque_min[i];
      if ( s->knee_torque[i] > s->knee_torque_max[i] )
	s->knee_torque[i] = s->knee_torque_max[i];
      if ( s->knee_torque[i] < s->knee_torque_min[i] )
	s->knee_torque[i] = s->knee_torque_min[i];
      if ( s->ankle_torque[i] > s->ankle_torque_max[i] )
	s->ankle_torque[i] = s->ankle_torque_max[i];
      if ( s->ankle_torque[i] < s->ankle_torque_min[i] )
	s->ankle_torque[i] = s->ankle_torque_min[i];
    }

  /* ZMP test */
  for ( i = LEFT; i <= RIGHT; i++ )
    {
      if ( s->ankle_torque[i] > s->ankle_to_heel*s->ground_force[i][ZZ] )
	s->ankle_torque[i] = s->ankle_to_heel*s->ground_force[i][ZZ];
      if ( s->ankle_torque[i] < -s->ankle_to_toe*s->ground_force[i][ZZ] )
	s->ankle_torque[i] = -s->ankle_to_toe*s->ground_force[i][ZZ];
    }
}

/*****************************************************************************/
/*
Trajectory control: reset current_desireds to current desired position
and velocity.
*/

void
reset_desireds( SIM *s )
{
  /* update current DESIRED positions */
  set_first_knot_to( current_desireds, L_HIP, QUINTIC_SPLINE, s->time,
		     s->hip_angle_d[LEFT], s->hip_angled_d[LEFT], 0.0 );
  set_first_knot_to( current_desireds, L_KNEE, QUINTIC_SPLINE, s->time,
		     s->knee_angle_d[LEFT], s->knee_angled_d[LEFT], 0.0 );
  set_first_knot_to( current_desireds, L_ANKLE, QUINTIC_SPLINE, s->time,
		     s->ankle_angle_d[LEFT], s->ankle_angled_d[LEFT], 0.0 );
  set_first_knot_to( current_desireds, R_HIP, QUINTIC_SPLINE, s->time,
		     s->hip_angle_d[RIGHT], s->hip_angled_d[RIGHT], 0.0 );
  set_first_knot_to( current_desireds, R_KNEE, QUINTIC_SPLINE, s->time,
		     s->knee_angle_d[RIGHT], s->knee_angled_d[RIGHT], 0.0 );
  set_first_knot_to( current_desireds, R_ANKLE, QUINTIC_SPLINE, s->time,
		     s->ankle_angle_d[RIGHT], s->ankle_angled_d[RIGHT], 0.0 );
}

/*****************************************************************************/
/*
Trajectory control: reset current_desireds to current desired position
with zero velocity.
*/

void
zero_velocity_desireds( SIM *s, int side )
{
  /* When the leg has an impact, adjust DESIRED positions to reflect that. */
  if ( side == LEFT )
    {
      set_first_knot_to( current_desireds, L_HIP, QUINTIC_SPLINE, s->time,
			 s->hip_angle_d[LEFT], 0.0, 0.0 );
      set_first_knot_to( current_desireds, L_KNEE, QUINTIC_SPLINE, s->time,
			 s->knee_angle_d[LEFT], 0.0, 0.0 );
    }
  if ( side == RIGHT )
    {
      set_first_knot_to( current_desireds, R_HIP, QUINTIC_SPLINE, s->time,
			 s->hip_angle_d[RIGHT], 0.0, 0.0 );
      set_first_knot_to( current_desireds, R_KNEE, QUINTIC_SPLINE, s->time,
			 s->knee_angle_d[RIGHT], 0.0, 0.0 );
    }
}

/*****************************************************************************/

void setup_swing_knots( SIM *s, int swing_hip, int swing_knee, double start_time )
{

  add_knot_point( desired_trajectory, swing_hip, QUINTIC_SPLINE, 
		  s->swing_time + start_time,
		  s->swing_hip_target + s->pitch_d, s->swing_hv1, s->swing_ha1 );

  add_knot_point( desired_trajectory, swing_knee, QUINTIC_SPLINE, 
		  s->swing_knee1_time*s->swing_time + start_time,
		  s->swing_knee1, s->swing_kv1, s->swing_ka1 );
  add_knot_point( desired_trajectory, swing_knee, QUINTIC_SPLINE, 
		  s->swing_time + start_time,
		  s->swing_knee_target, s->swing_kv2, s->swing_ka2 );
}

/*****************************************************************************/

void setup_stance_knots( SIM *s, int stance_hip, int stance_knee, double start_time )
{

  /* A little bogosity here */
  s->stance_time = s->swing_time;

  add_knot_point( desired_trajectory, stance_hip, QUINTIC_SPLINE, 
		  s->stance_time + start_time,
		  s->stance_hip_target + s->pitch_d, s->stance_hv1, s->stance_ha1 );

  add_knot_point( desired_trajectory, stance_knee, QUINTIC_SPLINE, 
		  s->pushoff_time*s->stance_time + start_time,
		  s->thrust1, s->stance_kv1, s->stance_ka1 );
  add_knot_point( desired_trajectory, stance_knee, QUINTIC_SPLINE, 
		  s->stance_time + start_time,
		  s->stance_knee_target, s->stance_kv2, s->stance_ka2 );
}

/*****************************************************************************/

void
run_state_machine( SIM *s )
{

  /* Debounce state transitions */
  if ( s->time - s->state_start_time < 0.001 )
    return;
  s->state_elapsed_time = s->time - s->state_start_time;

  /* Do actions and transitions */
  switch( s->controller_state )
    {
    case WAITING:
      // printf( "WAITING: %g %g\n", s->time, s->wait_duration );
      if ( s->time >= s->wait_duration )
	{ // do transition
	  s->controller_state = LAUNCH1;
	  s->state_start_time = s->time;
	  if ( s->controller_print )
	    printf( "LAUNCH1 %g\n", s->time );
	  clear_all_knots( desired_trajectory, N_BIPED_DOFS );
	  /* Lift and swing left leg */
	  add_knot_point( desired_trajectory, L_HIP, QUINTIC_SPLINE, 
			  s->l1_duration + s->time,
			  s->l1_lhip_target, 0.0, 0.0 );
	  add_knot_point( desired_trajectory, L_KNEE, QUINTIC_SPLINE, 
			  s->l1_duration + s->time,
			  s->l1_lknee_target, 0.0, 0.0 );
	  s->ankle_servo_mode[LEFT] = TORQUE_MODE;
	  s->ankle_command[LEFT] = 0.0;
	  s->ankle_servo_mode[RIGHT] = TORQUE_MODE;
	  s->ankle_command[RIGHT] = 0.0;

	  /* Swing left leg */
	  add_knot_point( desired_trajectory, L_HIP, QUINTIC_SPLINE, 
			  s->l1_duration + 
			  s->l2_duration + s->time,
			  s->l2_lhip_target, 0.0, 0.0 );
	  add_knot_point( desired_trajectory, L_KNEE, QUINTIC_SPLINE, 
			  s->l1_duration + 
			  s->l2_duration + s->time,
			  s->l2_lknee_target, 0.0, 0.0 );
	  add_knot_point( desired_trajectory, R_HIP, QUINTIC_SPLINE, 
			  s->l1_duration + 
			  s->l2_duration + s->time,
			  s->l2_rhip_target, 0.0, 0.0 );
	  add_knot_point( desired_trajectory, R_KNEE, QUINTIC_SPLINE, 
			  s->l1_duration + 
			  s->l2_duration + s->time,
			  s->l2_rknee_target, 0.0, 0.0 );
	}
      break;
    case LAUNCH1:
      /* need to delay this slightly */
      s->ankle_command[RIGHT] = s->l1_rankle_torque;
      /*
      printf( "LAUNCH1: %g %g %g\n",
	      s->state_elapsed_time, s->l1_duration, s->time );
      */
      if ( s->state_elapsed_time >= s->l1_duration )
	{
	  s->controller_state = LAUNCH2;
	  s->state_start_time = s->time;
	  if ( s->controller_print )
	    printf( "LAUNCH2 %g\n", s->time );
	  /* Don't need to do anything here. */
	  /* Eventually delete a launch state */
	}
      break;
    case LAUNCH2: 
      if ( s->state_elapsed_time >= s->l2_duration
	   && s->ground_force[RIGHT][ZZ] <= 0.001 )

	{
	  s->controller_state = R_SWING;
	  s->state_start_time = s->time;
	  if ( s->controller_print )
	    printf( "R_SWING %g\n", s->time );
	  /* Swing right leg while moving left leg back. */
	  reset_desireds( s );
	  clear_knots( desired_trajectory, L_HIP );
	  clear_knots( desired_trajectory, L_KNEE );
	  setup_stance_knots( s, L_HIP, L_KNEE, s->time );
	  setup_swing_knots( s, L_HIP, L_KNEE, s->time + s->stance_time );
	  clear_knots( desired_trajectory, R_HIP );
	  clear_knots( desired_trajectory, R_KNEE );
	  setup_swing_knots( s, R_HIP, R_KNEE, s->time );
	  setup_stance_knots( s, R_HIP, R_KNEE, s->time + s->swing_time );
	  s->ankle_command[LEFT] = s->stance_ankle_torque;
	  s->ankle_command[RIGHT] = 0.0;
	}
      break;
    case L_SWING:
      if ( s->ground_force[LEFT][ZZ] > 0.0 && s->state_elapsed_time > s->swing_time/2 )
	{
	  s->controller_state = RL_STANCE;
	  s->state_start_time = s->time;
	  if ( s->controller_print )
	    printf( "RL_STANCE %g\n", s->time );
	  reset_desireds( s );
	  zero_velocity_desireds( s, LEFT );
	  clear_knots( desired_trajectory, L_HIP );
	  clear_knots( desired_trajectory, L_KNEE );
	  setup_stance_knots( s, L_HIP, L_KNEE, s->time );
	  setup_swing_knots( s, L_HIP, L_KNEE, s->time + s->stance_time );
	  s->ankle_command[LEFT] = 0.0;
	  s->ankle_command[RIGHT] = 0.0;
	}
      break;
    case RL_STANCE:
      if ( s->ground_force[RIGHT][ZZ] <= 0.001 )
	{
	  s->controller_state = R_SWING;
	  s->state_start_time = s->time;
	  if ( s->controller_print )
	    printf( "R_SWING %g\n", s->time );
	  reset_desireds( s );
	  clear_knots( desired_trajectory, R_HIP );
	  clear_knots( desired_trajectory, R_KNEE );
	  setup_swing_knots( s, R_HIP, R_KNEE, s->time );
	  setup_stance_knots( s, R_HIP, R_KNEE, s->time + s->swing_time );
	  s->ankle_command[LEFT] = s->stance_ankle_torque;
	  s->ankle_command[RIGHT] = 0.0;
	}
      break;
    case R_SWING:
      if ( s->ground_force[RIGHT][ZZ] > 0.0 
	   && s->state_elapsed_time > s->swing_time/2 )
	{
	  s->controller_state = LR_STANCE;
	  s->state_start_time = s->time;
	  if ( s->controller_print )
	    printf( "LR_STANCE %g\n", s->time );
	  reset_desireds( s );
	  zero_velocity_desireds( s, RIGHT );
	  clear_knots( desired_trajectory, R_HIP );
	  clear_knots( desired_trajectory, R_KNEE );
	  setup_stance_knots( s, R_HIP, R_KNEE, s->time );
	  setup_swing_knots( s, R_HIP, R_KNEE, s->time + s->stance_time );
	  s->ankle_command[LEFT] = 0.0;
	  s->ankle_command[RIGHT] = 0.0;
	}
      break;
    case LR_STANCE:
      if ( s->ground_force[LEFT][ZZ] <= 0.001 )
	{
	  s->controller_state = L_SWING;
	  s->state_start_time = s->time;
	  if ( s->controller_print )
	    printf( "L_SWING %g\n", s->time );
	  reset_desireds( s );
	  clear_knots( desired_trajectory, L_HIP );
	  clear_knots( desired_trajectory, L_KNEE );
	  setup_swing_knots( s, L_HIP, L_KNEE, s->time );
	  setup_stance_knots( s, L_HIP, L_KNEE, s->time + s->swing_time );
	  s->ankle_command[LEFT] = 0.0;
	  s->ankle_command[RIGHT] = s->stance_ankle_torque;
	}
      break;
    case STOP:
      break;
    default:
      fprintf( stderr, "Unknown state %d in run_state_machine.\n",
	       s->controller_state );
      exit( -1 );
    }
}

/*****************************************************************************/

int controller( SIM *s )
{
  int i;

  /* Kinematics already computed by SDFAST */
  /* Forces already computed by SDFAST */

  run_state_machine( s );

  run_trajectory( s, s->time );

  run_servos( s );

  update_score( s );

  return 0;
}

/*****************************************************************************/
