/*
 * ga.h
 *
 *  Created on: May 28, 2025
 *      Author: User
 */

#ifndef INC_GA_H_
#define INC_GA_H_

#include "stm32f4xx_hal.h"
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include <float.h>
#include <string.h>

#define NV 3
#define POP_SIZE 100
#define MAX_ITER 200
#define PI 3.141592653589793

typedef struct {
    float x[NV];
    float fitness[2];
} Solution;

typedef struct {
    Solution solutions[POP_SIZE * 3];
    int size;
} Population;

extern float lb[NV];
extern float ub[NV];
extern float crossover_prob;
extern float mutation_prob;
extern int rate_local_search;
extern float step_size;

// Utility functions
float rand01();
float rand_range(float min, float max);

// Core NSGA-II functions
void random_population(Population *pop);
void evaluate(Solution *sol);
void crossover(Population *pop, Population *offspring);
void mutation(Population *pop, Population *offspring);
void local_search(Population *pop, Population *offspring);
bool dominates(Solution *sol1, Solution *sol2);
void find_pareto_front(Solution *solutions, int size, int *front_indices, int *front_size);
void crowding_distance(Solution *front, int front_size, float *distances);
void select_by_crowding(Solution *solutions, int size, int num_to_select, Solution *selected);
void selection(Population *pop, Population *selected);

#endif /* INC_GA_H_ */
