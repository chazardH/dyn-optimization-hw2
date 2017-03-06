/*****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "main.h"
#include "main2.h"

/*****************************************************************************/
/*****************************************************************************/

extern SIM sim;

#include "cmaes_interface.h"


void runCMA(SIM *sim, double* seedParams, int nparams, int maxIterations,
	int populationSize, double initialStdDev, double solveFunctionValue, double solveHistToleranceValue, int numTrials, double maxExternalForce);

/*****************************************************************************/
/*****************************************************************************/
/*****************************************************************************/
/*
double run_sim(SIM *sim,int numTrials,double maxExternalForce)
{
	double* forces = (double*)malloc(numTrials*sizeof(double));
	int j;
	for (j = 0; j < numTrials; j++)
	{
		if (numTrials == 1)
			forces[j] = maxExternalForce;
		else
			forces[j] = 2* maxExternalForce*j/(numTrials-1)- maxExternalForce;
		printf("force %f\n", forces[j]);
	}

	double totalScore = 0;

	//these two variables control how long the force is applied to the torso
	double startTime = 1.0;
	double duration = .10;
	for (j = 0; j < numTrials; j++)
	{
		int i;
		reinit_sim(sim);
		
		for (i = 0; sim->time < sim->duration; i++)
		{
			//set the torso perturbation to only act on a given time interval
			if(sim->time>startTime&&sim->time-startTime<duration)
				sim->torso_perturbation = forces[j];
			else sim->torso_perturbation = 0;

			controller(sim);
			save_data(sim);
			if (sim->status == CRASHED)
				break;
			integrate_one_time_step(sim);
		}
		totalScore += get_score(sim);
		printf("done trial %i\n", j);
	}
	free(forces);
	// write_the_mrdplot_file( sim );
	return totalScore/numTrials;
}
*/

//this is the faster version of run sim that does it all in one trial--num trials just has to be less than or equal to 6
double run_sim(SIM *sim, int numTrials, double maxExternalForce)
{
	if (numTrials % 2 != 0)
		numTrials++;
	double* forces = (double*)malloc(numTrials*sizeof(double));
	int j;
	for (j = 0; j < numTrials; j+=2)
	{
		forces[j] = maxExternalForce*(j+1) / (numTrials-1) ;
		forces[j + 1] = -forces[j];
		printf("force %f\n", forces[j]); printf("force %f\n", forces[j+1]);
	}

	double totalScore = 0;

	//these two variables control how long the force is applied to the torso
	double startTimes[] = {2.5,5.5,8.5,11.5,14.5,17.5};
	double duration = .05;
	
		int i;
		reinit_sim(sim);

		for (i = 0; sim->time < sim->duration; i++)
		{
			for (j = 0; j < numTrials; j++)
			{
				//set the torso perturbation to only act on a given time interval
				if (sim->time > startTimes[j] && sim->time - startTimes[j] < duration) {
					sim->torso_perturbation = forces[j];
					break;
				}
				else sim->torso_perturbation = 0;
			}
			controller(sim);
			save_data(sim);
			if (sim->status == CRASHED)
				break;
			integrate_one_time_step(sim);
		}
		totalScore += get_score(sim);
	
	free(forces);
	// write_the_mrdplot_file( sim );
	return totalScore;
}


main( int argc, char **argv )
{
  PARAMETER *params;
  int nparams;
  int i; 
  char* seedFile = "part2successparams"; //automatically read the default param file   //"part1optimizedparams"
  int singleRun = 1; //boolean value

  int numTrials = 6;   //number of trials per simulation if singleRun is false--their scores get averaged together
  double maxExternalForce = 25; //maximum external force applied in simulations (if numTrials>1)
  //20 is the maximum force it can deal with

  init_default_parameters( &sim );
  sim.rand_scale = 0;
  sim.controller_print = 1;

  params = read_parameter_file(seedFile);
  nparams = process_parameters(params, &sim, 1);
 /*
  // Parameter file argument? 
  if ( argc > 1 )
    {
      params = read_parameter_file( argv[1] );
      nparams = process_parameters( params, &sim, 1 );
      if ( nparams > MAX_nparams )
	{
	  printf( stderr, "Too many parameters %d > %d\n",
		   nparams, MAX_nparams );
	  exit( -1 );
	}
    }
  */

  init_sim( &sim );
  init_data( &sim );

  //single run test case--for just running one simulation parameter set
  if (singleRun) {
	  double score = run_sim(&sim, numTrials, maxExternalForce);
	  write_the_mrdplot_file(&sim);
	  printf("Final score %f\n", score);
  }

  //multiple run test case (for the CMA)
  else {
	  sim.controller_print = 0;
	  double* seedParams = (double*)malloc(nparams*sizeof(double));

	  parameters_to_dvector(params, seedParams);

	  int maxIterations = 20;//20
	  int populationSize = 70;//50 
	  double initialStdDev = .03;//.03
	  double solveFunctionValue = 25;
	  double solveHistToleranceValue = 1e-13;
	  
	  
	  runCMA(&sim,seedParams, nparams, maxIterations,populationSize, initialStdDev, solveFunctionValue, solveHistToleranceValue, numTrials, maxExternalForce);
	  free(seedParams);
  }
  
}


//the objective function is in the controller

//set the bounds in order (print them out just to verify the order is right)
void setMinAndMaxBounds(double* pMin,double* pMax,int nparams) {
	for (int i = 0; i < nparams; i++) {
		pMin[i] = 0.0;
		pMax[i] = 1.0;
	}
	/*
	(indexed from 0)
1	thrust1
2		swing_hip_target
8		swing_knee_target
12		stance_hv1
14		pitch_d
17		stance_knee_target
	decrease sensitivity for these
		*/
	double max = .1;
	double min = 0.0;
	pMin[1] = min;
	pMax[1] = max;
	pMin[2] = min;
	pMax[2] = max;
	pMin[8] = min;
	pMax[8] = max;
	pMin[12] = min;
	pMax[12] = max;
	pMin[14] = min;
	pMax[14] = max;
	pMin[17] = min;
	pMax[17] = max;
}

//CMA function
void runCMA(SIM *sim, double* seedParams,int nparams, int maxIterations, 
	int populationSize, double initialStdDev, double solveFunctionValue, double solveHistToleranceValue, int numTrials, double maxExternalForce){

	reinit_sim(sim);
	PARAMETER *pms = read_parameter_file("part1optimizedparams");//just a dummy, this variable gets overwritten
	process_parameters(pms, sim, 0);

	double* pMin = (double*)malloc(nparams*sizeof(double));
	double* pMax = (double*)malloc(nparams*sizeof(double));

	setMinAndMaxBounds(pMin, pMax, nparams);


	double functionValue = 0;
	double* params = (double*)malloc(nparams*sizeof(double));
	double* stdDev = (double*)malloc(nparams*sizeof(double));
	
	for (int i = 0; i < nparams; i++) {
		stdDev[i] = initialStdDev;
		params[i] = seedParams[i];
	}
	

	// Initialize CMA with the current parameters, scaled to [0,1].
	double* pScaled = (double*)malloc(nparams*sizeof(double));

	for (int i = 0; i<nparams; i++) {
		pScaled[i] = (params[i] - pMin[i]) / (pMax[i] - pMin[i]);
	}
	
	double initialFunctionValue = run_sim(sim,numTrials,maxExternalForce);

	// Initialize the CMA blackbox optimization
	cmaes_t evo;
	double *arFunVals = cmaes_init(&evo, nparams, &pScaled[0], &stdDev[0], 0, populationSize, "non");


	evo.sp.stopMaxFunEvals = 1e299;
	evo.sp.stStopFitness.flg = 1;
	evo.sp.stStopFitness.val = solveFunctionValue;
	evo.sp.stopMaxIter = maxIterations;
	evo.sp.stopTolFun = 1e-9;
	evo.sp.stopTolFunHist = solveHistToleranceValue;
	evo.sp.stopTolX = 1e-11;
	evo.sp.stopTolUpXFactor = 1e3;
	evo.sp.seed = 0;

	for (int i = 0; i < nparams; i++)
		evo.rgxbestever[i] = pScaled[i];
	evo.rgxbestever[nparams] = initialFunctionValue;
	evo.rgxbestever[nparams + 1] = 1;

//										assert(IS_EQUAL(cmaes_Get(&evo, "fbestever"), initialFunctionValue));

	
	double* pi = (double*)malloc(nparams*sizeof(double));
	for (int i = 0; i < nparams; i++) {
		pi[i] = 0.0;
	}

	int iter = 0;
	for (; !cmaes_TestForTermination(&evo); iter++)
	{
		// Sample the parameter space
		double *const *pop = cmaes_SamplePopulation(&evo);
		int popSize = (int)cmaes_Get(&evo, "popsize");

		
		printf("Starting iteration %d\n", iter);
		

		for (int popIdx = 0; popIdx<popSize; popIdx++) {
			printf("pop member %d\n", popIdx);
			for (int i = 0; i < nparams; i++) {
				pi[i] = (1 - pop[popIdx][i])*pMin[i] + (pop[popIdx][i])*pMax[i];
				//for physical practicality, can't go below the min
//				if (pi[i] < pMin[i])
//					pi[i] = pMin[i];
			}
			// Evaluate the objective for each sampling point.
			dvector_to_sim(pi, nparams, pms);
			arFunVals[popIdx] = run_sim(sim,numTrials, maxExternalForce);

			//save the best one so far in case we quit early
			double* paras = (double*)malloc(nparams*sizeof(double));
			double* unscaled = (double*)malloc(nparams*sizeof(double));
			cmaes_GetInto(&evo, "xbestever", paras);
			for (int i = 0; i<nparams; i++)
				unscaled[i] = (1 - paras[i])*pMin[i] + (paras[i])*pMax[i];
			dvector_to_sim(unscaled, nparams, pms);
			write_param_file("CMA_bestsofar", pms);
			free(paras);
			free(unscaled);
		}
		// Update the distribution
		cmaes_UpdateDistribution(&evo, arFunVals);

/*		// Print output
		if (printLevel >= 2)
		{
			dVector pTmp((int)p.size(), 0);

			cmaes_GetInto(&evo, "xmean", &pTmp[0]);
			for (int i = 0; i<p.size(); i++)
				pi[i] = (1 - pTmp[i])*pMin[i] + (pTmp[i])*pMax[i];
			double mean = function->computeValue(pi);

			cmaes_GetInto(&evo, "xbest", &pTmp[0]);
			for (int i = 0; i<p.size(); i++)
				pi[i] = (1 - pTmp[i])*pMin[i] + (pTmp[i])*pMax[i];
			double best = function->computeValue(pi);

			cmaes_GetInto(&evo, "xbestever", &pTmp[0]);
			for (int i = 0; i<p.size(); i++)
				pi[i] = (1 - pTmp[i])*pMin[i] + (pTmp[i])*pMax[i];
			double bestEver = function->computeValue(pi);

			Logger::logPrint("       Mean function value: %.6lf\n", mean);
			Logger::logPrint("       Best function value: %.6lf\n", best);
			Logger::logPrint("   Bestever function value: %.6lf\n", bestEver);
		}
		*/
	}

	// Obtain the result and scale it back
	cmaes_GetInto(&evo, "xbestever", &pScaled[0]);
	for (int i = 0; i<nparams; i++)
		params[i] = (1 - pScaled[i])*pMin[i] + (pScaled[i])*pMax[i];
	dvector_to_sim(params, nparams, pms); 
	functionValue = run_sim(sim, numTrials, maxExternalForce);

	
		printf("CMA ended in %d/%d iterations\n", iter, maxIterations);
		printf("   Function value improved from % .5lf\n", initialFunctionValue);
		printf("                             to % .5lf\n", functionValue);

		printf("CMA ended in %d/%d iterations\n", iter, maxIterations);

	//save the evolved parameters
	write_param_file("CMA_output", pms);
	cmaes_exit(&evo);
	free(pMin);
	free(pMax);
	free(params);
	free(stdDev);
	free(pScaled);
	free(pi);
	free(pms);
	return iter < maxIterations;
}

