/******************************************************************/
/* 
Defines, typedefs, and structs
*/
/******************************************************************/

/* Indices */
#define LEFT 0
#define RIGHT 1

/* These are handy constants. XX rather than X to avoid math.h conflicts */
#define XX 0
#define YY 1
#define ZZ 2

/* Status flags */
#define OK 0
#define CRASHED 1

/* May have N_CONTROLS != N_CTRL_INPUTS eventually
   include COM_X, COM_Y, TORSO_PITCH, or other variables in N_CTRL_INPUTS */
/* Joints (for trajectories and gains) */
#define L_HIP 0
#define L_KNEE 1
#define L_ANKLE 2
#define R_HIP 3
#define R_KNEE 4
#define R_ANKLE 5
#define N_BIPED_DOFS 6
#define N_CONTROLS 6
#define N_CTRL_INPUTS 6

/* This is used to allocate parameter vectors etc. */
#define MAX_N_PARAMETERS 100

/* This is used to allocate sdfast state */
#define MAX_N_SDFAST_STATE 100

// foot status
#define FOOT_ON_GROUND 0
#define FOOT_SLIDING 1
#define FOOT_IN_AIR 2

/******************************************************************/

/* For each parameter read in from a file we have this information */
typedef struct parameter
{
  char *name;
  double value;
  int optimize;
  int regularize;
  double nominal_value;
  double regularize_weight;
  struct parameter *next;
  double *pointer;
} PARAMETER;

/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/
/******************************************************************/
// Overall simulation structure.

typedef struct {

  // *** COMMON VARIABLES ACROSS ALL SIMULATIONS

  double time_step; // controller time step    
  double time;      // current time in simulation
  int status;       // Are we CRASHED? 

  // Joint angles
  double pitch; /* angle of torso wrt vertical */
  double hip_angle[2]; // 3D: hip_yaw[2], hip_roll[2], hip_pitch[2]
  double knee_angle[2];
  double ankle_angle[2];

  // commanded torques
  double hip_command[2]; // 3D: hip_yaw_command[2], ...
  double knee_command[2];
  double ankle_command[2];

  // torques (actually applied)
  double hip_torque[2];
  double knee_torque[2];
  double ankle_torque[2];

  // torques limits
  double hip_torque_min[2];
  double hip_torque_max[2];
  double knee_torque_min[2];
  double knee_torque_max[2];
  double ankle_torque_min[2];
  double ankle_torque_max[2];

  // Location of various body parts. */
  double head[3];
  double hip[3];
  double knee[2][3];
  double foot[2][3];
  // These are link COM locations
  double torso[3];
  double thigh[2][3];
  double calf[2][3];
  double com[3]; // total body COM

  // absolute angles
  double torso_abs_angle;
  double thigh_abs_angle[2];
  double calf_abs_angle[2];

  // Angular velocities
  double pitchd;
  double hip_angled[2];
  double knee_angled[2];
  double ankle_angled[2];

  // Linear velocities
  double headd[3];
  double hipd[3];
  double kneed[2][3];
  double footd[2][3];
  // These are COM velocities
  double torsod[3];
  double thighd[2][3];
  double calfd[2][3];
  double comd[3];

  // *** KINEMATIC AND DYNAMIC PARAMETERS

  double torso_length;
  double torso_cm;
  double torso_mass;
  double torso_I;
  double thigh_length;
  double thigh_cm;
  double thigh_mass;
  double thigh_I;
  double calf_length;
  double calf_cm;
  double calf_mass;
  double calf_I;

  double ankle_to_toe;
  double ankle_to_heel;

  double head_offset[3]; // head relative to torso COM
  double hip_offset[3];  // hip relative to torso COM
  double knee_offset[3]; // knee relative to calf COM
  double foot_offset[3]; // foot relative to calf COM

  // *** DYNAMICS **************

  // Control bits
  /* Flag to indicate both feet off the ground not allowed */
  int in_air_not_allowed; 

  // SDFAST
  int sdfast_flag; /* Flag to indicate dxt needs to be recalculated */
  double sdfast_state[MAX_N_SDFAST_STATE]; // sdfast state
  double sdfast_stated[MAX_N_SDFAST_STATE]; // derivative of sdfast state
  int sdfast_integration_steps_per_control_step;
  double sdfast_ctol;
  double sdfast_baumgarte;
  double x_offset;

  // Stuff for impact model
  int sdfast_model; /* Which model is multiple model system using? */
  int sdfast_integration_steps_per_control_step_impact;
  double ds_state[MAX_N_SDFAST_STATE];
  double ss_state[MAX_N_SDFAST_STATE];
  double ss_stated[MAX_N_SDFAST_STATE];
  double stashed_ss_state[MAX_N_SDFAST_STATE];
  int ss_foot_down;
  int foot_status[2];

  // ground model
  double ground_level;

  // ground spring damper parameters
  double gnd_k_z;
  double gnd_k_x;
  double gnd_b_z;
  double gnd_b_x;

  // original ground contact point.
  double foot_zero[2][3];

  // ground forces
  double ground_force[2][3];

  // knee limit
  double knee_limit;
  double knee_limit_k;
  double knee_limit_b;

  // crash detection
  double hip_min;
  double hip_max;

  /* Ankle torque limits */
  double zmp_x_min;
  double zmp_x_max;

  /* force limits */
  double friction_cone_limit;

  // *** CONTROLLER **************

  // controller variables
  int controller_state;
  double state_start_time;
  double state_elapsed_time;

  // gains xxx[torque][sensor]
  double pos_gains[N_CONTROLS][N_CTRL_INPUTS];
  double vel_gains[N_CONTROLS][N_CTRL_INPUTS];
  double int_gains[N_CONTROLS][N_CTRL_INPUTS];

  // desired positions
  double head_d[3];
  double hip_d[3];
  double knee_d[2][3];
  double foot_d[2][3];
  double pitch_d;
  double hip_angle_d[2];
  double knee_angle_d[2];
  double ankle_angle_d[2];

  // desired velocities
  double headd_d[3];
  double hipd_d[3];
  double kneed_d[2][3];
  double footd_d[2][3];
  double pitchd_d;
  double hip_angled_d[2];
  double knee_angled_d[2];
  double ankle_angled_d[2];

  // integrated errors
  double hip_integrated_error[2];
  double knee_integrated_error[2];
  double ankle_integrated_error[2];

  // Torque feedforwards
  double hip_command_ff[2];
  double knee_command_ff[2];
  double ankle_command_ff[2];

  /* Servo modes */
  int hip_servo_mode[2];
  int knee_servo_mode[2];
  int ankle_servo_mode[2];

  // *** PATTERN GENERATION ****************

  /* Initial posture */
  double wait_duration;
  double wait_l_hip;
  double wait_l_knee;
  double wait_l_ankle;
  double wait_r_hip;
  double wait_r_knee;
  double wait_r_ankle;

  /* Launch1 parameters: lift left leg */
  double l1_duration;
  double l1_lhip_target;
  double l1_lknee_target;
  double l1_rankle_torque;

  /* Launch2 parameters: swing left leg */
  double l2_duration;
  double l2_lhip_target;
  double l2_lknee_target;
  double l2_rhip_target;
  double l2_rknee_target;
  
  /* Steady state gait parameters */
  double swing_time; /* how long it takes to swing leg (sec) */
  double stance_time; /* how long stance leg is on the ground (sec) */
  double thrust1; /* knee angle before pushoff (radians) */
  double swing_hip_target; /* Where to swing leg to */
  double swing_hv1; /* velocity of knot */
  double swing_ha1; /* acceleration of knot */
  double swing_knee1_time;  /* Some intermediate knee targets and times */
  double swing_knee1;
  double swing_kv1; /* velocity of knot */
  double swing_ka1; /* acceleration of knot */
  double swing_knee_target; /* Where to swing leg to */
  double swing_kv2; /* velocity of knot */
  double swing_ka2; /* acceleration of knot */
  double stance_hip_target; /* Where to move stance hip to */
  double stance_hv1; /* velocity of knot */
  double stance_ha1; /* acceleration of knot */
  /* double pitch_d; see above */ /* Desired pitch angle */
  double pushoff_time; /* When to push off */
  double stance_kv1; /* velocity of knot */
  double stance_ka1; /* acceleration of knot */
  double stance_knee_target;
  double stance_kv2; /* velocity of knot */
  double stance_ka2; /* acceleration of knot */
  double stance_ankle_torque; /* How much to torque at stance angle */

  // *** OPTIMIZATION PARAMETERS ***************

  /* Objective function */
  double desired_speed;
  double clearance;
  double crashed_penalty_weight;
  double speed_penalty_weight;
  double torque_penalty_weight;
  double clearance_penalty_weight;
  double f_x_penalty_weight;
  double f_z_penalty_weight;
  double pitchd_penalty_weight;
  double summed_score;
  double discount;
  double discount_to_power;
  double discounted_score;
  double crashed_score;
  double torque_score;
  double knee_score;
  double speed_score;
  double clearance_score;
  double f_x_score;
  double f_z_score;
  double pitchd_score;
  double one_step_torque_score;
  double one_step_knee_score;
  double one_step_speed_score;
  double one_step_clearance_score;
  double one_step_f_x_score;
  double one_step_f_z_score;
  double one_step_pitchd_score;
  double one_step_crashed_score;
  double one_step_total_score;

  /* Useful variables for optimization */
  int n_parameters;
  double all_time_low_cost;
  double debug_criterion;
  int func_calls;
  int n_func_calls_per_eval;
  PARAMETER *params;
  char output_file[10000];
  int iter;

  // *** MISCELLANEOUS ***********************

  // simulation duration
  double duration;

  // random stuff
  // Make this thread safe: random number generator seed for rand_r() and rand()
  int rand_seed;
  double rand_scale;

  /* Perturbations */
  double torso_perturbation;

  /* debugging message flags */
  int controller_print;

  double total_mass;
  double total_weight_in_newtons;

} SIM;

/******************************************************************/
