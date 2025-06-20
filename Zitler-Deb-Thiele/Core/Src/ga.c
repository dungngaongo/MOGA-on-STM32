/*
 * ga.c
 *
 *  Created on: Jun 9, 2025
 *      Author: User
 */

#include "ga.h"

extern RNG_HandleTypeDef hrng;

float lb[NV];
float ub[NV];
float crossover_prob = 0.6;
float mutation_prob = 0.05;
int rate_local_search = 30;
float step_size = 0.02;

void initialize_bounds() {
    for (int i = 0; i < NV; i++) {
        lb[i] = 0.0;
        ub[i] = 1.0;
    }
}

uint32_t trng_rand() {
    uint32_t value;
    HAL_RNG_GenerateRandomNumber(&hrng, &value);
    return value;
}

float rand01() {
    return (float)trng_rand() / (float)0xFFFFFFFF;
}

float rand_range(float min, float max) {
    return min + (max - min) * rand01();
}

void random_population(Population *pop) {
    for (int i = 0; i < POP_SIZE; i++) {
        for (int j = 0; j < NV; j++) {
            pop->solutions[i].x[j] = rand_range(lb[j], ub[j]);
        }
    }
    pop->size = POP_SIZE;
}

void evaluate(Solution *sol) {
    float f1 = sol->x[0];

    float g = 1.0;
    float sum = 0.0;
    for (int i = 1; i < NV; i++) {
        sum += sol->x[i];
    }
    g += 9.0 * sum / (NV - 1);

    float ratio = f1 / g;
    float h = 1.0 - sqrt(ratio) - ratio * sin(10.0 * PI * f1);

    sol->fitness[0] = f1;
    sol->fitness[1] = g * h;
}

void crossover(Population *pop, Population *offspring) {
    offspring->size = 0;
    for (int i = 0; i < pop->size / 2; i++) {
        if (rand01() < crossover_prob) {
            int r1 = trng_rand() % pop->size;
            int r2 = trng_rand() % pop->size;
            while (r1 == r2) {
                r2 = trng_rand() % pop->size;
            }

            int cutting_point = trng_rand() % (NV - 1) + 1;

            for (int j = 0; j < NV; j++) {
                if (j < cutting_point) {
                    offspring->solutions[offspring->size].x[j] = pop->solutions[r1].x[j];
                    offspring->solutions[offspring->size + 1].x[j] = pop->solutions[r2].x[j];
                } else {
                    offspring->solutions[offspring->size].x[j] = pop->solutions[r2].x[j];
                    offspring->solutions[offspring->size + 1].x[j] = pop->solutions[r1].x[j];
                }
            }

            evaluate(&offspring->solutions[offspring->size]);
            evaluate(&offspring->solutions[offspring->size + 1]);

            offspring->size += 2;
        }
    }
}

void mutation(Population *pop, Population *offspring) {
    offspring->size = 0;
    for (int i = 0; i < pop->size; i++) {
        if (rand01() < mutation_prob) {
            offspring->solutions[offspring->size] = pop->solutions[i];
            int mutation_point = trng_rand() % NV;
            offspring->solutions[offspring->size].x[mutation_point] =
                rand_range(lb[mutation_point], ub[mutation_point]);
            evaluate(&offspring->solutions[offspring->size]);
            offspring->size++;
        }
    }
}

void local_search(Population *pop, Population *offspring) {
    offspring->size = rate_local_search;
    for (int i = 0; i < rate_local_search; i++) {
        int r1 = trng_rand() % pop->size;
        offspring->solutions[i] = pop->solutions[r1];
        int r2 = trng_rand() % NV;
        offspring->solutions[i].x[r2] += rand_range(-step_size, step_size);

        if (offspring->solutions[i].x[r2] < lb[r2])
            offspring->solutions[i].x[r2] = lb[r2];
        if (offspring->solutions[i].x[r2] > ub[r2])
            offspring->solutions[i].x[r2] = ub[r2];

        evaluate(&offspring->solutions[i]);
    }
}

bool dominates(Solution *sol1, Solution *sol2) {
    bool better = false;
    for (int i = 0; i < 2; i++) {
        if (sol1->fitness[i] > sol2->fitness[i]) {
            return false;
        }
        if (sol1->fitness[i] < sol2->fitness[i]) {
            better = true;
        }
    }
    return better;
}

void find_pareto_front(Solution *solutions, int size, int *front_indices, int *front_size) {
    *front_size = 0;
    for (int i = 0; i < size; i++) {
        bool is_dominated = false;
        for (int j = 0; j < size; j++) {
            if (i == j) continue;
            if (dominates(&solutions[j], &solutions[i])) {
                is_dominated = true;
                break;
            }
        }
        if (!is_dominated) {
            front_indices[(*front_size)++] = i;
        }
    }
}

void crowding_distance(Solution *front, int front_size, float *distances) {
    if (front_size == 0) return;

    for (int i = 0; i < front_size; i++) {
        distances[i] = 0.0;
    }

    for (int obj = 0; obj < 2; obj++) {
        int indices[POP_SIZE * 3];
        for (int i = 0; i < front_size; i++) indices[i] = i;

        for (int i = 0; i < front_size - 1; i++) {
            for (int j = i + 1; j < front_size; j++) {
                if (front[indices[i]].fitness[obj] > front[indices[j]].fitness[obj]) {
                    int temp = indices[i];
                    indices[i] = indices[j];
                    indices[j] = temp;
                }
            }
        }

        distances[indices[0]] = DBL_MAX;
        distances[indices[front_size - 1]] = DBL_MAX;

        float fmin = front[indices[0]].fitness[obj];
        float fmax = front[indices[front_size - 1]].fitness[obj];
        float range = fmax - fmin;

        if (range > 0) {
            for (int i = 1; i < front_size - 1; i++) {
                distances[indices[i]] +=
                    (front[indices[i + 1]].fitness[obj] - front[indices[i - 1]].fitness[obj]) / range;
            }
        }
    }
}

void select_by_crowding(Solution *solutions, int size, int num_to_select, Solution *selected) {
    float distances[POP_SIZE * 3];
    crowding_distance(solutions, size, distances);

    for (int i = 0; i < num_to_select; i++) {
        int best = 0;
        for (int j = 1; j < size; j++) {
            if (distances[j] > distances[best]) {
                best = j;
            }
        }
        selected[i] = solutions[best];
        distances[best] = -1.0;
    }
}

void selection(Population *pop, Population *selected) {
    int remaining_indices[pop->size];
    int remaining_size = pop->size;
    for (int i = 0; i < pop->size; i++) remaining_indices[i] = i;

    selected->size = 0;

    while (selected->size < POP_SIZE) {
        int front_indices[remaining_size];
        int front_size;
        find_pareto_front(pop->solutions, remaining_size, front_indices, &front_size);

        if (selected->size + front_size > POP_SIZE) {
            int needed = POP_SIZE - selected->size;
            Solution temp_front[front_size];
            for (int i = 0; i < front_size; i++) {
                temp_front[i] = pop->solutions[front_indices[i]];
            }

            Solution selected_from_front[needed];
            select_by_crowding(temp_front, front_size, needed, selected_from_front);

            for (int i = 0; i < needed; i++) {
                selected->solutions[selected->size++] = selected_from_front[i];
            }
            break;
        } else {
            for (int i = 0; i < front_size; i++) {
                selected->solutions[selected->size++] = pop->solutions[front_indices[i]];
            }

            int new_remaining_size = 0;
            for (int i = 0; i < remaining_size; i++) {
                bool in_front = false;
                for (int j = 0; j < front_size; j++) {
                    if (remaining_indices[i] == front_indices[j]) {
                        in_front = true;
                        break;
                    }
                }
                if (!in_front) {
                    remaining_indices[new_remaining_size++] = remaining_indices[i];
                }
            }
            remaining_size = new_remaining_size;
        }
    }
}
