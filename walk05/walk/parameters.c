/*****************************************************************************/
/*
  parameters.c: manage simulation parameters
*/
/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <malloc.h>
#include <string.h>

#include "main.h"
#include "main2.h"

/*****************************************************************************/

void
init_default_parameters( SIM *s )
{
  int i;

  /* Overall parameters */
  s->duration = 20.0;
  s->time_step = 0.001;

  /* Objective function */
  s->desired_speed = 1.0;
  s->clearance = 0.02;
  s->speed_penalty_weight = 1e-3;
  s->torque_penalty_weight = 1e-7;
  s->clearance_penalty_weight = 1e1;
  s->f_x_penalty_weight = 1e-6;
  s->f_z_penalty_weight = 1e-6;
  s->pitchd_penalty_weight = 1e-1;
  s->crashed_penalty_weight = 1e2;
  s->summed_score = 0.0;
  s->discount = 0.99954;
  s->discount_to_power = 1.0;
  s->discounted_score = 0;

  s->rand_scale = 0.0;
  s->rand_seed = 1;

  s->pos_gains[L_HIP][L_HIP] = 2000.0;
  s->vel_gains[L_HIP][L_HIP] = 100.0;
  s->int_gains[L_HIP][L_HIP] = 0.0;
  s->pos_gains[L_KNEE][L_KNEE] = 2000.0;
  s->vel_gains[L_KNEE][L_KNEE] = 100.0;
  s->int_gains[L_KNEE][L_KNEE] = 0.0;
  s->pos_gains[L_ANKLE][L_ANKLE] = 0.0;
  s->vel_gains[L_ANKLE][L_ANKLE] = 0.0;
  s->int_gains[L_ANKLE][L_ANKLE] = 0.0;
  s->pos_gains[R_HIP][R_HIP] = 2000.0;
  s->vel_gains[R_HIP][R_HIP] = 100.0;
  s->int_gains[R_HIP][R_HIP] = 0.0;
  s->pos_gains[R_KNEE][R_KNEE] = 2000.0;
  s->vel_gains[R_KNEE][R_KNEE] = 100.0;
  s->int_gains[R_KNEE][R_KNEE] = 0.0;
  s->pos_gains[R_ANKLE][R_ANKLE] = 0.0;
  s->vel_gains[R_ANKLE][R_ANKLE] = 0.0;
  s->int_gains[R_ANKLE][R_ANKLE] = 0.0;

  // contact impedance parameters
  s->gnd_k_z = 300000.0;
  s->gnd_b_z = 7000.0;
  s->gnd_k_x = 1000000.0;
  s->gnd_b_x = 200000.0;
  s->ground_level = 0.0;

  s->knee_limit_k = 1000.0;
  s->knee_limit_b = 2.0;
  s->knee_limit = -0.01; // Where is the hard knee stop?

  s->wait_duration = 0.0;

  s->l1_duration = 0.2;
  s->l1_lhip_target = 0.35;
  s->l1_lknee_target = -0.3;
  s->l1_rankle_torque = 0;

  s->l2_duration = 0.2;
  s->l2_lhip_target = s->l1_lhip_target;
  s->l2_lknee_target = -0.1;
  s->l2_rhip_target = -0.15;
  s->l2_rknee_target = -0.01;

  /* Steady state gait parameters */
  s->swing_time = 0.89;
  s->stance_time = s->swing_time; /* Reset in setup_stance_knots */

  s->thrust1 = -0.047;

  s->swing_hip_target = 0.33;
  s->swing_hv1 = -0.37;
  s->swing_ha1 = -0.10;

  s->swing_knee1_time = 0.7; /* FIXED */
  s->swing_knee1 = -0.43;
  s->swing_kv1 = 0.29;
  s->swing_ka1 = 0.26;

  s->swing_knee_target = -0.080;
  s->swing_kv2 = 0.11;
  s->swing_ka2 = -0.14;

  s->stance_hip_target = -0.22;
  s->stance_hv1 = -0.031;
  s->stance_ha1 = 0.28;

  s->pitch_d = 0.085;

  s->pushoff_time = 0.5; /* FIXED */
  s->stance_kv1 = 0.41;
  s->stance_ka1 = -0.38;

  s->stance_knee_target = -0.046;
  s->stance_kv2 = -0.29;
  s->stance_ka2 = -0.36;

  s->stance_ankle_torque = -0.12;

  s->torso_perturbation = 0;

  /*
Assume biped is standing.
Torso CoM is measured from hip.
Thigh CoM is measured from hip.
Calf CoM is measured from knee.

Torso (T)
Left Thigh (LT)
Left Calf (LC)
Right Thigh (RT)
Right Calf (RC)

     |
     |
     | T
     |
   __|__
LT |   | RT
   |   |
   *   *
   |   |
LC |   | RC

The feet are point feet, which can exert a limited torque while on the
ground (+/-74 Newton-meters).

Knees bend both ways.

lengths:
head to hip: 0.8m
hip to knee: 0.3918m
knee to ankle: 0.3810m

calf inertial parameters includes foot.

mass:
torso: 50kg
thigh: 5.676kg
calf: 6.8952kg

moment of inertia (about center of mass):
torso: 1.5kg-m^2
thigh: 0.09592kg-m^2
calf: 0.1535kg-m^2

center of mass
torso: 0.2868m above hip
thigh: 0.1898m below hip
calf: 0.2384m below knee

body COM location relative to hip:
(0.2868*50 - 0.1898*5.676*2 - (0.3918+ 0.2384)*6.8952*2)/(50+5.676*2+6.8952*2)
= 0.0465
a little low relative to a human, should be around 0.1
*/

  s->torso_length = 0.8;
  s->thigh_length = 0.3918;
  s->calf_length = 0.3810;
  s->torso_mass = 50;
  s->thigh_mass = 5.676;
  s->calf_mass = 6.8952;
  s->torso_I = 1.5;
  s->thigh_I = 0.09592;
  s->calf_I = 0.1535;
  s->torso_cm = 0.2868;
  s->thigh_cm = 0.1898;
  s->calf_cm = 0.2384;

  s->ankle_to_toe = 0.1;
  s->ankle_to_heel = 0.1;

  for( i = 0; i < 3; i++ )
    {
      s->head_offset[i] = 0;
      s->hip_offset[i] = 0;
      s->knee_offset[i] = 0;
      s->foot_offset[i] = 0;
    }
  s->head_offset[ZZ] = s->torso_length - s->torso_cm;
  s->hip_offset[ZZ] = -s->torso_cm;
  s->knee_offset[ZZ] = s->calf_cm;
  s->foot_offset[ZZ] = -(s->calf_length - s->calf_cm);

  s->in_air_not_allowed = 1;

  s->hip_min = 1.5*s->calf_length;
  s->hip_max = 1.1*(s->calf_length + s->thigh_length);

  /* Assume symmetric foot */
  s->zmp_x_min = -0.1;
  s->zmp_x_max = 0.1;

  s->friction_cone_limit = 2.0;

  s->sdfast_flag = 1;
  s->sdfast_integration_steps_per_control_step = 20;
  s->sdfast_integration_steps_per_control_step_impact = 1;
  s->sdfast_ctol = 1e-1; /* constraint tolerance */
  s->sdfast_baumgarte = 100.0; /* could be 10.0 - 1000.0 */

  s->n_func_calls_per_eval = 1;
  s->all_time_low_cost = 1e20;
  s->debug_criterion = 1;

  s->total_mass = (s->torso_mass + 2*s->thigh_mass + 2*s->calf_mass);
  s->total_weight_in_newtons = s->total_mass*9.81;

  for( i = LEFT; i <= RIGHT; i++ )
    {
      s->hip_torque_min[i] = -100;
      s->hip_torque_max[i] = 100;
      s->knee_torque_min[i] = -100;
      s->knee_torque_max[i] = 100;
      // this is a hack that assumes single support
      // ankle torque is further limited by ZMP test in controller.c
      s->ankle_torque_min[i] =
	-s->ankle_to_toe*s->total_weight_in_newtons;
      s->ankle_torque_max[i] =
	s->ankle_to_heel*s->total_weight_in_newtons;
    }

  /*
  printf( "Torque limits: %g %g %g %g %g %g\n",
	  s->hip_torque_min[LEFT],
	  s->hip_torque_max[LEFT],
	  s->knee_torque_min[LEFT],
	  s->knee_torque_max[LEFT],
	  s->ankle_torque_min[LEFT],
	  s->ankle_torque_max[LEFT] );
  */

  s->controller_print = 0;
}

/*****************************************************************************/
/******************************************************************/

PARAMETER *malloc_parameter()
{
  PARAMETER *result;

  result = (PARAMETER *) malloc( sizeof( PARAMETER ) );
  if ( result == NULL )
    {
      fprintf( stderr, "No memory to allocate PARAMETER.\n" );
      exit( -1 );
    }
  result->name = NULL;
  result->value = 0;
  result->optimize = 0;
  result->regularize = 0;
  result->nominal_value = 0;
  result->regularize_weight = 0;
  result->next = NULL;
  result->pointer = NULL;
  return result;
}

/******************************************************************/

PARAMETER *read_parameter_file( char *filename )
{
  FILE *stream;
  char buffer[10000];
  PARAMETER *p, *top, *bottom;

  top = NULL;
  bottom = NULL;
  p = NULL;
  stream = fopen( filename, "r" );
  if ( stream == NULL )
    {
      fprintf( stderr, "Can't open %s\n", filename );
      exit( -1 );
    }
  for(;;)
    {
      /* get name of variable */
      if ( fscanf( stream, "%s", buffer ) < 1 )
	break;
      p = malloc_parameter();
      p->name = strdup( buffer );
      if ( fscanf( stream, "%lf", &(p->value) ) < 1 )
	{
	  fprintf( stderr, "Missing value for %s in %s\n",
		   p->name, filename );
	  exit( -1 );
	}

      /* read various commands */
      for( ; ; )
	{
	  if ( fscanf( stream, "%s", buffer ) < 1 )
	    {
	      fprintf( stderr, "Missing end for %s in %s\n",
		       p->name, filename );
	      exit( -1 );
	    }
	  if ( strcmp( buffer, "end" ) == 0 )
	    break;
	  if ( strcmp( buffer, "opt" ) == 0 )
	    p->optimize = 1;
	}

      if ( top == NULL )
	top = p;
      if ( bottom != NULL )
	bottom->next = p;
      bottom = p;
    }
  fclose( stream );
  return top;
}

/******************************************************************/
/******************************************************************/

int process_parameters( PARAMETER *p, SIM *s, int verbose )
{
  int total = 0;

  for( ; ; )
    {
      if ( p == NULL )
	break;
      if ( strcmp( p->name, "swing_time" ) == 0 )
	p->pointer = &(s->swing_time);
      else if ( strcmp( p->name, "thrust1" ) == 0 )
	p->pointer = &(s->thrust1);
      else if ( strcmp( p->name, "swing_hip_target" ) == 0 )
	p->pointer = &(s->swing_hip_target);
      else if ( strcmp( p->name, "swing_hv1" ) == 0 )
	p->pointer = &(s->swing_hv1);
      else if ( strcmp( p->name, "swing_ha1" ) == 0 )
	p->pointer = &(s->swing_ha1);
      else if ( strcmp( p->name, "swing_knee1" ) == 0 )
	p->pointer = &(s->swing_knee1);
      else if ( strcmp( p->name, "swing_kv1" ) == 0 )
	p->pointer = &(s->swing_kv1);
      else if ( strcmp( p->name, "swing_ka1" ) == 0 )
	p->pointer = &(s->swing_ka1);
      else if ( strcmp( p->name, "swing_knee_target" ) == 0 )
	p->pointer = &(s->swing_knee_target);
      else if ( strcmp( p->name, "swing_kv2" ) == 0 )
	p->pointer = &(s->swing_kv2);
      else if ( strcmp( p->name, "swing_ka2" ) == 0 )
	p->pointer = &(s->swing_ka2);
      else if ( strcmp( p->name, "stance_hip_target" ) == 0 )
	p->pointer = &(s->stance_hip_target);
      else if ( strcmp( p->name, "stance_hv1" ) == 0 )
	p->pointer = &(s->stance_hv1);
      else if ( strcmp( p->name, "stance_ha1" ) == 0 )
	p->pointer = &(s->stance_ha1);
      else if ( strcmp( p->name, "pitch_d" ) == 0 )
	p->pointer = &(s->pitch_d);
      else if ( strcmp( p->name, "stance_kv1" ) == 0 )
	p->pointer = &(s->stance_kv1);
      else if ( strcmp( p->name, "stance_ka1" ) == 0 )
	p->pointer = &(s->stance_ka1);
      else if ( strcmp( p->name, "stance_knee_target" ) == 0 )
	p->pointer = &(s->stance_knee_target);
      else if ( strcmp( p->name, "stance_kv2" ) == 0 )
	p->pointer = &(s->stance_kv2);
      else if ( strcmp( p->name, "stance_ka2" ) == 0 )
	p->pointer = &(s->stance_ka2);
      else if ( strcmp( p->name, "pushoff_time" ) == 0 )
	p->pointer = &(s->pushoff_time);
      else if ( strcmp( p->name, "swing_knee1_time" ) == 0 )
	p->pointer = &(s->swing_knee1_time);
      else if ( strcmp( p->name, "stance_ankle_torque" ) == 0 )
	p->pointer = &(s->stance_ankle_torque);
      else if ( strcmp( p->name, "speed_penalty_weight" ) == 0 )
	p->pointer = &(s->speed_penalty_weight);
      else if ( strcmp( p->name, "l1_duration" ) == 0 )
	p->pointer = &(s->l1_duration);
      else if ( strcmp( p->name, "l1_lhip_target" ) == 0 )
	p->pointer = &(s->l1_lhip_target);
      else if ( strcmp( p->name, "l1_lknee_target" ) == 0 )
	p->pointer = &(s->l1_lknee_target);
      else if ( strcmp( p->name, "l1_rankle_torque" ) == 0 )
	p->pointer = &(s->l1_rankle_torque);
      else if ( strcmp( p->name, "l2_duration" ) == 0 )
	p->pointer = &(s->l2_duration);
      else if ( strcmp( p->name, "l2_lhip_target" ) == 0 )
	p->pointer = &(s->l2_lhip_target);
      else if ( strcmp( p->name, "l2_lknee_target" ) == 0 )
	p->pointer = &(s->l2_lknee_target);
      else if ( strcmp( p->name, "l2_rhip_target" ) == 0 )
	p->pointer = &(s->l2_rhip_target);
      else if ( strcmp( p->name, "l2_rknee_target" ) == 0 )
	p->pointer = &(s->l2_rknee_target);

      /*
      else if ( strcmp( p->name, "" ) == 0 )
	p->pointer = &(s->);
      */

      else 
	{
	  fprintf( stderr, "Don't recognize %s in process_parameters()\n", p->name );
	  exit( -1 );
	}
      *(p->pointer) = p->value;
      if ( p->optimize )
	{
	  if ( verbose )
	    printf( "%d: %s\n", total, p->name );
	  total++;
	}
      p = p->next;
    }

  if ( verbose )
    printf( "Optimizing %d parameters.\n", total );
  return total;
}

/************************************************************************/

void
parameters_to_vector( PARAMETER *p, float *v )
{
  int indx = 0;

  printf( "Parameters_to_vector:\n" );
  for( ; ; )
    {
      if ( p == NULL )
	break;
      if ( p->optimize )
	{
	  v[indx++] = *(p->pointer);
	  printf( "%d: %g\n", indx-1, v[indx-1] );
	}
      p = p->next;
    }
}

/************************************************************************/

void
parameters_to_dvector( PARAMETER *p, double *v )
{
  int indx = 0;

  printf( "Parameters_to_vector:\n" );
  for( ; ; )
    {
      if ( p == NULL )
	break;
      if ( p->optimize )
	{
	  v[indx++] = *(p->pointer);
	  printf(p->name);
	  printf( " %d: %lg\n", indx-1, v[indx-1] );
	}
      p = p->next;
    }
}

/************************************************************************/

void
vector_to_sim( float *v, int n_parameters, PARAMETER *p )
{
  int indx;
  
  for( indx = 0; indx < n_parameters; indx++ )
    {
      /* Find next parameter to optimize */
      for( ; ; )
	{
	  if ( p == NULL )
	    {
	      fprintf( stderr, "Failure in vector_to_parameters: %d\n",
		       indx );
	      exit( - 1 );
	    }
	  if ( p->optimize )
	    break;
	  p = p->next;
	}
      *(p->pointer) = v[indx];
      
      printf( "%d: %g\n", indx, v[indx] );
      
      p = p->next;
    }
}

/************************************************************************/

void
dvector_to_sim( double *v, int n_parameters, PARAMETER *p )
{
  int indx;
  for( indx = 0; indx < n_parameters; indx++ )
    {
      /* Find next parameter to optimize */
      for( ; ; )
	{
	  if ( p == NULL )
	    {
	      fprintf( stderr, "Failure in vector_to_parameters: %d\n",
		       indx );
	      exit( - 1 );
	    }
	  if ( p->optimize )
	    break;
	  p = p->next; 
	}
      *(p->pointer) = v[indx];
      
      printf( "k %d: %g\n", indx, v[indx] );
      
      p = p->next;
    }
}

/******************************************************************/

int write_param_file( char *filename, PARAMETER *p )
{
  FILE *stream;

  stream = fopen( filename, "w" );
  if ( stream == NULL )
    {
      fprintf( stderr, "Can't open %s for write.\n", filename );
      exit( -1 );
    }

  for( ; ; )
    {
      if ( p == NULL )
	break;
      fprintf( stream, "%s %25.20lg", p->name, *(p->pointer) );
      if ( p->optimize )
	fprintf( stream, " opt" );
      fprintf( stream, " end\n" );
      p = p->next;
    }

  fclose( stream );

  return 0;
}

/*****************************************************************************/

