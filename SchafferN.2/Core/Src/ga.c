/*
 * ga.c
 *
 *  Created on: May 28, 2025
 *      Author: User
 */


#include "ga.h"

extern RNG_HandleTypeDef hrng;

float lb[NV] = {-5.0};
float ub[NV] = {10.0};
float crossover_prob = 0.6;
float mutation_prob = 0.05;
int rate_local_search = 30;
float step_size = 0.02;

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
    float x = sol->x[0];

    if (x <= 1) {
        sol->fitness[0] = -x;
    } else if (x > 1 && x <= 3) {
        sol->fitness[0] = x - 2;
    } else if (x > 3 && x <= 4) {
        sol->fitness[0] = 4 - x;
    } else {
        sol->fitness[0] = x - 4;
    }

    sol->fitness[1] = pow(x - 5, 2);
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

            for (int j = 0; j < NV; j++) {
                offspring->solutions[offspring->size].x[j] =
                    (pop->solutions[r1].x[j] + pop->solutions[r2].x[j]) / 2.0;
                offspring->solutions[offspring->size + 1].x[j] =
                    (pop->solutions[r1].x[j] + pop->solutions[r2].x[j]) / 2.0;
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
            offspring->solutions[offspring->size].x[0] = rand_range(lb[0], ub[0]);
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
        offspring->solutions[i].x[0] += rand_range(-step_size, step_size);

        if (offspring->solutions[i].x[0] < lb[0]) offspring->solutions[i].x[0] = lb[0];
        if (offspring->solutions[i].x[0] > ub[0]) offspring->solutions[i].x[0] = ub[0];

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

void update_archive(Population *pop, Archive *archive) {
    int front_indices[pop->size];
    int front_size;
    find_pareto_front(pop->solutions, pop->size, front_indices, &front_size);

    for (int i = 0; i < front_size; i++) {
        if (archive->size < MAX_ARCHIVE) {
            archive->solutions[archive->size++] = pop->solutions[front_indices[i]];
        }
    }

    if (archive->size > MAX_ARCHIVE) {
        Solution temp[MAX_ARCHIVE];
        select_by_crowding(archive->solutions, archive->size, MAX_ARCHIVE, temp);
        for (int i = 0; i < MAX_ARCHIVE; i++) {
            archive->solutions[i] = temp[i];
        }
        archive->size = MAX_ARCHIVE;
    }
}

void send_archive(UART_HandleTypeDef *huart, Archive *archive) {
    char msg[64];
    for (int i = 0; i < archive->size; i++) {
        snprintf(msg, sizeof(msg), "archive,%f,%f\r\n",
                 archive->solutions[i].fitness[0],
                 archive->solutions[i].fitness[1]);
        HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
    }
}
