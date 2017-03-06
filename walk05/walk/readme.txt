*****************************************************************

Quick start:
Run a simulation and look at result.
on linux:
make
./simulate p0
./animate

******************************************************************
Files:

animate.c: animate a data file.
animate.vcproj: file for Microsoft Windows Visual Studio 2008 to compile
                animate.c

controller.c: example walking policy

data.c: saving data in mrdplot format.

drawstuff.dll: file for Windows version of drawstuff.

dynamics.c: dynamics using spring/damper ground model

dynamics-impact.c: multiple model dynamics

last_data: keep track of data file number for file name generation

learn.c: version of simulate.c to show how to do multiple simulations.

lu.c: LU solver: solve for x in matrix equation Ax = b

main.h: main include file.
main2.h: function declarations

Makefile: instructions for compiling on Linux

p0: example parameter file.

parameters.c: handle simulation parameters.

readme.txt: this file.

sdfast: sdfast dynamics stuff, see www.sdfast.com for info

simulate.c: main program, run the simulation.
simulate.vcproj: file for Microsoft Windows Visual Studio 2008 to compile
                 simulate.c

******************************************************************

******************************************************************
KINEMATICS
******************************************************************

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


*******************************************************************
*******************************************************************

Cost function is defined in one_step_cost() in controller.c

*******************************************************************
*******************************************************************
CONTROLLER:

controller.c: walking policy. This is the controller state machine. It
is called on each tick, with the simulation data structure s as the
argument.  Based on s->controller_state, the appropriate actions are
performed.  There is some additional housekeeping at the beginning and
the end of the subroutine.

Joints can be PD servoed to a desired position or trajectory,
or torque controlled (run_servos()).

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

run_state_machine()
has actions and transition actions of all states

Before starting, the biped is in the WAITING state.
The machine is set to the following configuration:
  s->hip[XX] = 0;
  s->hip[ZZ] = 0;
  s->pitch = 0;
  s->hip_angle[LEFT] = s->wait_l_hip;
  s->hip_angle[RIGHT] = s->wait_r_hip;
  s->knee_angle[LEFT] = s->wait_l_knee;
  s->knee_angle[RIGHT] = s->wait_r_knee;

The state machine works as follows:
To start, the state is set to LAUNCH1.
LAUNCH1: Bend knees. After enough time, go to LAUNCH2.
LAUNCH2: Push off with the right leg. When the right foot leaves the ground,
                 go to LAUNCH3.
LAUNCH3: Swing right leg forward. When right foot hits the ground, go to
                 LR_STANCE.
L_SWING: Swing left foot, Go to RL_STANCE when left foot touches ground.
RL_STANCE: Double support. Zero hip torques. When R foot lifts off, go to
                 R_SWING.
R_SWING: Swing right foot, Go to LR_STANCE when right foot touches ground.
LR_STANCE: Double support. Zero hip torques. When L foot lifts off, go to
                 L_SWING.

// transitions to R_SWING, L_SWING
setup_swing_knots()
setup_stance_knots()

*****************************************************************
PARAMETERS YOU CAN MODIFY
*****************************************************************

  /* Steady state gait parameters */
  s->swing_time = 0.89;
  s->stance_time = s->swing_time; /* Reset in setup_stance_knots */

  s->thrust1 = -0.047;

  // knot point for swing hip
  s->swing_hip_target = 0.33;
  s->swing_hv1 = -0.37;
  s->swing_ha1 = -0.10;

  // first knot point for swing knee
  s->swing_knee1 = -0.43;
  s->swing_kv1 = 0.29;
  s->swing_ka1 = 0.26;

  // second knot point for swing knee
  s->swing_knee_target = -0.080;
  s->swing_kv2 = 0.11;
  s->swing_ka2 = -0.14;

  // one knot point for stance hip
  s->stance_hip_target = -0.22;
  s->stance_hv1 = -0.031;
  s->stance_ha1 = 0.28;

  s->pitch_d = 0.085;

  // first knot point for stance knee
  s->stance_kv1 = 0.41;
  s->stance_ka1 = -0.38;

  // second knot point for stance knee
  s->stance_knee_target = -0.046;
  s->stance_kv2 = -0.29;
  s->stance_ka2 = -0.36;

  s->stance_ankle_torque = -0.12; // set on transitions to X_SWING

Parameters you could also vary:
  s->swing_knee1_time = 0.7;
  s->pushoff_time = 0.5;

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

  // Launch 1 parameters: see transition out of WAITING
  s->l1_duration = 0.2;
  s->l1_lhip_target = 0.35;
  s->l1_lknee_target = -0.3;
  s->l1_rankle_torque = 0; // see LAUNCH1 action

  // Launch 2 parameters: see transition out of WAITING
  s->l2_duration = 0.2;
  s->l2_lhip_target = s->l1_lhip_target;
  s->l2_lknee_target = -0.1;
  s->l2_rhip_target = -0.15;
  s->l2_rknee_target = -0.01;

*************************************************************************
PERTURBATION

sim.torso_perturbation // can be used to apply a perturbation force on the torso

*************************************************************************
************************************************************************
