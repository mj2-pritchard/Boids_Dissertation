#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>   // for mkdir
#include <errno.h>
#include "boidUpdate.h"
#include "messaging.h"

#define NUM_BOIDS 3000
#define NUM_STEPS 200

// Data structure for collecting terrain samples.
typedef struct {
    double *data;  // Each sample has 3 doubles: x, y, ground z.
    int size;      // Number of samples stored.
    int capacity;  // Capacity in samples.
} TerrainData;

void initTerrainData(TerrainData *td) {
    td->capacity = 100;
    td->size = 0;
    td->data = (double*)malloc(td->capacity * 3 * sizeof(double));
    if (!td->data) {
        fprintf(stderr, "Error allocating terrainData\n");
        exit(1);
    }
}

void appendTerrainData(TerrainData *td, double x, double y, double z) {
    if (td->size >= td->capacity) {
        td->capacity *= 2;
        td->data = (double*)realloc(td->data, td->capacity * 3 * sizeof(double));
        if (!td->data) {
            fprintf(stderr, "Error reallocating terrainData\n");
            exit(1);
        }
    }
    int idx = td->size * 3;
    td->data[idx + 0] = x;
    td->data[idx + 1] = y;
    td->data[idx + 2] = z;
    td->size++;
}

void freeTerrainData(TerrainData *td) {
    free(td->data);
}

void writeCSVFilesDistr(const double *positions, const double *statuses, 
                          int localNumBoids, int numSteps,
                          const TerrainData *terrainData,
                          const double bounds[3],
                          int rank) {
    char filename[256];

    if (mkdir("output", 0777) != 0 && errno != EEXIST) {
        perror("mkdir");
        exit(1);
    }

    sprintf(filename, "output/positions_distr_rank%d.csv", rank);
    FILE *fp = fopen(filename, "w");
    if (!fp) { perror(filename); exit(1); }
    fprintf(fp, "boid,step,x,y,z\n");
    for (int boid = 0; boid < localNumBoids; boid++) {
        for (int step = 0; step < numSteps; step++) {
            int idx = (step * localNumBoids + boid) * 3;
            fprintf(fp, "%d,%d,%.6f,%.6f,%.6f\n", boid, step, 
                    positions[idx + 0], positions[idx + 1], positions[idx + 2]);
        }
    }
    fclose(fp);

    sprintf(filename, "output/statuses_distr_rank%d.csv", rank);
    fp = fopen(filename, "w");
    if (!fp) { perror(filename); exit(1); }
    fprintf(fp, "boid,step,status\n");
    for (int boid = 0; boid < localNumBoids; boid++) {
        for (int step = 0; step < numSteps; step++) {
            int idx = step * localNumBoids + boid;
            fprintf(fp, "%d,%d,%.0f\n", boid, step, statuses[idx]);
        }
    }
    fclose(fp);

    sprintf(filename, "output/terrainData_distr_rank%d.csv", rank);
    fp = fopen(filename, "w");
    if (!fp) { perror(filename); exit(1); }
    fprintf(fp, "x,y,z\n");
    for (int i = 0; i < terrainData->size; i++) {
        int idx = i * 3;
        fprintf(fp, "%.6f,%.6f,%.6f\n", terrainData->data[idx + 0],
                terrainData->data[idx + 1], terrainData->data[idx + 2]);
    }
    fclose(fp);

    sprintf(filename, "output/bounds_distr_rank%d.txt", rank);
    fp = fopen(filename, "w");
    if (!fp) { perror(filename); exit(1); }
    fprintf(fp, "%.6f, %.6f, %.6f\n", bounds[0], bounds[1], bounds[2]);
    fclose(fp);
}

int main(int argc, char *argv[])
{
    int nProcs = 1;
    char *env_nprocs = getenv("NPROCS");
    if (env_nprocs) {
        nProcs = atoi(env_nprocs);
    }
    int rank = 0;
    char *env_rank = getenv("RANK");
    if (env_rank) {
        rank = atoi(env_rank);
    } else if (argc >= 3) {
        rank = atoi(argv[1]);
        nProcs = atoi(argv[2]);
    } else {
        printf("Usage: %s <rank> <nProcs>\nDefaulting to rank=0, nProcs=1\n", argv[0]);
    }
    printf("Using RANK=%d, NPROCS=%d\n", rank, nProcs);

    // Enable messaging.
    const char *host = getenv("RABBITMQ_HOST");
    if (!host) host = "localhost";
    printf("Using RabbitMQ host: %s\n", host);
    if (initMessaging(host) != 0) {
        fprintf(stderr, "initMessaging: Failed to open TCP socket to %s\n", host);
        return -1;
    }

    // Set up the consumer queue on every process.
    if (setupConsumerQueue() != 0) {
        fprintf(stderr, "Failed to set up consumer queue\n");
        exit(1);
    }

    // Allow a short delay for all processes to finish setting up their queues.
    sleep(2);

    BoidParams params;
    initParameters(&params, 124);

    double *allStates = malloc(NUM_BOIDS * BOID_STATE_SIZE * sizeof(double));
    if (!allStates) {
        fprintf(stderr, "Memory allocation failed for boid states\n");
        exit(1);
    }

    int boidsPerProc = NUM_BOIDS / nProcs;
    int startIdx = rank * boidsPerProc;
    int endIdx = (rank == nProcs - 1) ? NUM_BOIDS : startIdx + boidsPerProc;
    int localNumBoids = endIdx - startIdx;

    // --- Initialization ---
    if (rank == 0) {
        // Rank 0 initializes the full global state.
        for (int i = 0; i < NUM_BOIDS; i++) {
            double x = ((double)rand() / RAND_MAX) * params.bounds[0];
            double y = ((double)rand() / RAND_MAX) * params.bounds[1];
            double ground = getTerrainHeight(x, y, &params);
            double z = ground + params.margin + (((double)rand() / RAND_MAX) * (params.bounds[2] - ground));
            double vx = (((double)rand() / RAND_MAX) - 0.5) * params.maxSpeed;
            double vy = (((double)rand() / RAND_MAX) - 0.5) * params.maxSpeed;
            double vz = (((double)rand() / RAND_MAX) - 0.5) * params.maxSpeed;
            allStates[i * BOID_STATE_SIZE + 0] = x;
            allStates[i * BOID_STATE_SIZE + 1] = y;
            allStates[i * BOID_STATE_SIZE + 2] = z;
            allStates[i * BOID_STATE_SIZE + 3] = vx;
            allStates[i * BOID_STATE_SIZE + 4] = vy;
            allStates[i * BOID_STATE_SIZE + 5] = vz;
            allStates[i * BOID_STATE_SIZE + 6] = 1.0;
        }
        // Publish the entire initial global state.
        if (publishGlobalState(allStates, NUM_BOIDS * BOID_STATE_SIZE * sizeof(double), rank) != 0) {
            fprintf(stderr, "Failed to publish global state\n");
            exit(1);
        }
    } else {
        // Other ranks consume the initial global state.
        if (consumeGlobalState(allStates, NUM_BOIDS * BOID_STATE_SIZE * sizeof(double)) != 0) {
            fprintf(stderr, "Failed to consume global state\n");
            exit(1);
        }
    }

    // --- Allocate history arrays and terrain data ---
    double *positionsHistory_local = malloc(NUM_STEPS * localNumBoids * 3 * sizeof(double));
    double *statusesHistory_local = malloc(NUM_STEPS * localNumBoids * sizeof(double));
    if (!positionsHistory_local || !statusesHistory_local) {
        fprintf(stderr, "Memory allocation failed for local history arrays\n");
        exit(1);
    }
    TerrainData terrainData;
    initTerrainData(&terrainData);

    // Record initial state (step 0) for local boids.
    for (int i = startIdx; i < endIdx; i++) {
        int localIndex = i - startIdx;
        int posIdx = localIndex * 3;
        positionsHistory_local[posIdx + 0] = allStates[i * BOID_STATE_SIZE + 0];
        positionsHistory_local[posIdx + 1] = allStates[i * BOID_STATE_SIZE + 1];
        positionsHistory_local[posIdx + 2] = allStates[i * BOID_STATE_SIZE + 2];
        statusesHistory_local[localIndex] = allStates[i * BOID_STATE_SIZE + 6];
        double ground = getTerrainHeight(allStates[i * BOID_STATE_SIZE + 0],
                                         allStates[i * BOID_STATE_SIZE + 1],
                                         &params);
        appendTerrainData(&terrainData, allStates[i * BOID_STATE_SIZE + 0],
                                    allStates[i * BOID_STATE_SIZE + 1],
                                    ground);
    }

    // --- Simulation loop using an all-gather approach ---
    for (int step = 1; step < NUM_STEPS; step++) {
        // 1. Update local boids.
        stepBoidsSubset(allStates, NUM_BOIDS, startIdx, endIdx, &params);

        // 2. Publish local update.
        int localMsgSize = localNumBoids * BOID_STATE_SIZE * sizeof(double);
        double *localBuffer = malloc(localMsgSize);
        if (!localBuffer) {
            perror("malloc for localBuffer");
            exit(1);
        }
        memcpy(localBuffer, &allStates[startIdx * BOID_STATE_SIZE], localMsgSize);
        if (publishGlobalState(localBuffer, localMsgSize, rank) != 0) {
            fprintf(stderr, "Failed to publish local state from rank %d\n", rank);
            exit(1);
        }
        // Save a copy of our own update for merging.
        double *myLocalCopy = malloc(localMsgSize);
        if (!myLocalCopy) {
            perror("malloc for myLocalCopy");
            exit(1);
        }
        memcpy(myLocalCopy, localBuffer, localMsgSize);
        free(localBuffer);

        // 3. Gather updates from all ranks.
        // For each rank r, expect a message of size computed from that rank's boids.
        for (int r = 0; r < nProcs; r++) {
            int boidsForRank = (r == nProcs - 1) ? (NUM_BOIDS - r * boidsPerProc) : boidsPerProc;
            size_t expectedSize = boidsForRank * BOID_STATE_SIZE * sizeof(double);
            if (r == rank) {
                // Use our local copy.
                memcpy(&allStates[r * boidsPerProc * BOID_STATE_SIZE], myLocalCopy, expectedSize);
            } else {
                double *tempBuffer = malloc(expectedSize);
                if (!tempBuffer) {
                    perror("malloc for tempBuffer");
                    exit(1);
                }
                if (consumeGlobalState(tempBuffer, expectedSize) != 0) {
                    fprintf(stderr, "Failed to consume state message from rank %d\n", r);
                    exit(1);
                }
                memcpy(&allStates[r * boidsPerProc * BOID_STATE_SIZE], tempBuffer, expectedSize);
                free(tempBuffer);
            }
        }
        free(myLocalCopy);

        // 4. Record updated state for local boids.
        for (int i = startIdx; i < endIdx; i++) {
            int localIndex = i - startIdx;
            int posIdx = (step * localNumBoids + localIndex) * 3;
            positionsHistory_local[posIdx + 0] = allStates[i * BOID_STATE_SIZE + 0];
            positionsHistory_local[posIdx + 1] = allStates[i * BOID_STATE_SIZE + 1];
            positionsHistory_local[posIdx + 2] = allStates[i * BOID_STATE_SIZE + 2];
            statusesHistory_local[step * localNumBoids + localIndex] = allStates[i * BOID_STATE_SIZE + 6];

            double ground = getTerrainHeight(allStates[i * BOID_STATE_SIZE + 0],
                                             allStates[i * BOID_STATE_SIZE + 1],
                                             &params);
            appendTerrainData(&terrainData, allStates[i * BOID_STATE_SIZE + 0],
                                        allStates[i * BOID_STATE_SIZE + 1],
                                        ground);
        }
    }

    writeCSVFilesDistr(positionsHistory_local, statusesHistory_local, localNumBoids, NUM_STEPS, &terrainData, params.bounds, rank);
    printf("Distributed simulation complete on rank %d. Output files saved in the 'output' folder.\n", rank);

    free(positionsHistory_local);
    free(statusesHistory_local);
    freeTerrainData(&terrainData);
    free(allStates);
    return 0;
}
