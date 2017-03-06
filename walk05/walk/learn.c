/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "main2.h"

/*****************************************************************************/
/*****************************************************************************/

extern SIM sim;

/*****************************************************************************/
/*****************************************************************************/
// Example of optimizing a value
/*****************************************************************************/
/*****************************************************************************/

double run_sim( SIM *sim )
{
  int i;

  for( i = 0; sim->time < sim->duration; i++ )
    {
      controller( sim );
      save_data( sim );
      if ( sim->status == CRASHED )
	break;
      integrate_one_time_step( sim );
    }

  // write_the_mrdplot_file( sim );
  return get_score( sim );
}

/*****************************************************************************/

main( int argc, char **argv )
{
  int i;
  PARAMETER *params;
  int n_parameters;
  double score, new_score;

  init_default_parameters( &sim );
  sim.rand_scale = 0;
  sim.controller_print = 1;

  /* Parameter file argument? */
  if ( argc > 1 )
    {
      params = read_parameter_file( argv[1] );
      n_parameters = process_parameters( params, &sim, 1 );
      if ( n_parameters > MAX_N_PARAMETERS )
	{
	  fprintf( stderr, "Too many parameters %d > %d\n",
		   n_parameters, MAX_N_PARAMETERS );
	  exit( -1 );
	}
    }

  init_sim( &sim );
  init_data( &sim );

  sim.controller_print = 0;
  score = run_sim( &sim );

  // show how to run simulation multiple times.
  for ( i = 0; i < 10; i++ )
    {
      reinit_sim( &sim );
      sim.thrust1 -= 0.001;
      new_score = run_sim( &sim );
      if ( new_score > score )
	break;
      score = new_score;
    }
}

/*****************************************************************************/
