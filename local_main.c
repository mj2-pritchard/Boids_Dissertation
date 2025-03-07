#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>   // for mkdir
#include <errno.h>
#include "boidUpdate.h"

// Define simulation dimensions
#define NUM_BOIDS 6000
#define NUM_STEPS 200
#define PROGRESS_BAR_WIDTH 50

// Data structure for collecting terrain samples
typedef struct {
    double *data;  // Each sample has 3 doubles: x, y, ground z
    int size;      // Number of samples stored
    int capacity;  // Capacity in samples
} TerrainData;

// Initialize the TerrainData structure.
void initTerrainData(TerrainData *td) {
    td->capacity = 100; // initial capacity; will grow as needed
    td->size = 0;
    td->data = (double*)malloc(td->capacity * 3 * sizeof(double));
    if (!td->data) {
        fprintf(stderr, "Error allocating terrainData\n");
        exit(1);
    }
}

// Append one terrain sample (x, y, z) to td.
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

// Free the terrain data.
void freeTerrainData(TerrainData *td) {
    free(td->data);
}

// Save CSV files into the "output" folder.
void writeCSVFiles(const double *positions, const double *statuses, 
    int numBoids, int numSteps,
    const TerrainData *terrainData,
    const double bounds[3]) {
    // Create the output folder if it doesn't exist.
    if (mkdir("output", 0777) != 0 && errno != EEXIST) {
        perror("mkdir");
        exit(1);
    }

    // Write positions.csv: output all time steps for a given boid, then next boid.
    FILE *fp = fopen("output/positions.csv", "w");
    if (!fp) { perror("fopen positions.csv"); exit(1); }
    fprintf(fp, "boid,step,x,y,z\n");
    for (int boid = 0; boid < numBoids; boid++) {
        for (int step = 0; step < numSteps; step++) {
            // positions is stored as [step][boid][3] flattened:
            int idx = (step * numBoids + boid) * 3;
            fprintf(fp, "%d,%d,%.6f,%.6f,%.6f\n", boid, step, 
                    positions[idx + 0], positions[idx + 1], positions[idx + 2]);
        }
    }
    fclose(fp);

    // Write statuses.csv: output all time steps for each boid.
    fp = fopen("output/statuses.csv", "w");
    if (!fp) { perror("fopen statuses.csv"); exit(1); }
    fprintf(fp, "boid,step,status\n");
    for (int boid = 0; boid < numBoids; boid++) {
        for (int step = 0; step < numSteps; step++) {
            int idx = step * numBoids + boid;
            fprintf(fp, "%d,%d,%.0f\n", boid, step, statuses[idx]);
        }
    }
    fclose(fp);

    // Write terrainData.csv (order remains the same).
    fp = fopen("output/terrainData.csv", "w");
    if (!fp) { perror("fopen terrainData.csv"); exit(1); }
    fprintf(fp, "x,y,z\n");
    for (int i = 0; i < terrainData->size; i++) {
        int idx = i * 3;
        fprintf(fp, "%.6f,%.6f,%.6f\n", terrainData->data[idx + 0], 
                terrainData->data[idx + 1], terrainData->data[idx + 2]);
    }
    fclose(fp);

    // Write bounds.txt.
    fp = fopen("output/bounds.txt", "w");
    if (!fp) { perror("fopen bounds.txt"); exit(1); }
    fprintf(fp, "%.6f, %.6f, %.6f\n", bounds[0], bounds[1], bounds[2]);
    fclose(fp);
}

int main(void)
{
    BoidParams params;
    initParameters(&params, 124);  // use seed 123

    // Allocate the global boid state array.
    // Each boid state is 7 doubles: x,y,z, vx,vy,vz, flag.
    double *allStates = (double*)malloc(NUM_BOIDS * BOID_STATE_SIZE * sizeof(double));
    if (!allStates) {
        fprintf(stderr, "Memory allocation failed for boid states\n");
        exit(1);
    }
    
    // Allocate history arrays:
    double *positionsHistory = (double*)malloc(NUM_STEPS * NUM_BOIDS * 3 * sizeof(double));
    double *statusesHistory  = (double*)malloc(NUM_STEPS * NUM_BOIDS * sizeof(double));
    if (!positionsHistory || !statusesHistory) {
        fprintf(stderr, "Memory allocation failed for history arrays\n");
        exit(1);
    }
    
    // Terrain data structure.
    TerrainData terrainData;
    initTerrainData(&terrainData);
    
    // Initialize boids.
    for (int i = 0; i < NUM_BOIDS; i++) {
        double x = ((double)rand() / RAND_MAX) * params.bounds[0];
        double y = ((double)rand() / RAND_MAX) * params.bounds[1];
        double ground = getTerrainHeight(x, y, &params);
        // Choose z as ground + margin plus a random fraction of the remaining depth.
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
        allStates[i * BOID_STATE_SIZE + 6] = 1.0;  // active
    }
    
    // Record initial state (step 0)
    for (int i = 0; i < NUM_BOIDS; i++) {
        int posIdx = i * 3;
        positionsHistory[posIdx + 0] = allStates[i * BOID_STATE_SIZE + 0];
        positionsHistory[posIdx + 1] = allStates[i * BOID_STATE_SIZE + 1];
        positionsHistory[posIdx + 2] = allStates[i * BOID_STATE_SIZE + 2];
        statusesHistory[i] = allStates[i * BOID_STATE_SIZE + 6];
        
        // Record a terrain sample for each boid (optional)
        double ground = getTerrainHeight(allStates[i * BOID_STATE_SIZE + 0],
                                         allStates[i * BOID_STATE_SIZE + 1],
                                         &params);
        appendTerrainData(&terrainData, allStates[i * BOID_STATE_SIZE + 0],
                                    allStates[i * BOID_STATE_SIZE + 1],
                                    ground);
    }
    
    // Simulation loop: for each time step from 1 to NUM_STEPS-1,
    // update all boids and record positions and statuses.
    for (int step = 1; step < NUM_STEPS; step++) {
        // Update all boids using our shared step function.
        stepBoidsSubset(allStates, NUM_BOIDS, 0, NUM_BOIDS, &params);
        
        // Record state after update.
        for (int i = 0; i < NUM_BOIDS; i++) {
            int posIdx = (step * NUM_BOIDS + i) * 3;
            positionsHistory[posIdx + 0] = allStates[i * BOID_STATE_SIZE + 0];
            positionsHistory[posIdx + 1] = allStates[i * BOID_STATE_SIZE + 1];
            positionsHistory[posIdx + 2] = allStates[i * BOID_STATE_SIZE + 2];
            statusesHistory[step * NUM_BOIDS + i] = allStates[i * BOID_STATE_SIZE + 6];
            
            // Record terrain sample for this boid update.
            double ground = getTerrainHeight(allStates[i * BOID_STATE_SIZE + 0],
                                             allStates[i * BOID_STATE_SIZE + 1],
                                             &params);
            appendTerrainData(&terrainData, allStates[i * BOID_STATE_SIZE + 0],
                                        allStates[i * BOID_STATE_SIZE + 1],
                                        ground);
        }
        
        // Update and display progress bar
        float progress = (float)step / (NUM_STEPS - 1);
        int barComplete = (int)(progress * PROGRESS_BAR_WIDTH);
        printf("%3d%% [", (int)(progress * 100));
        for (int i = 0; i < PROGRESS_BAR_WIDTH; i++) {
            if (i < barComplete)
                printf("=");
            else if (i == barComplete)
                printf(">");
            else
                printf(" ");
        }
        printf("]\r");
        fflush(stdout);
    }
    
    // Ensure progress bar shows 100% complete.
    printf("100%% [");
    for (int i = 0; i < PROGRESS_BAR_WIDTH; i++) {
        printf("=");
    }
    printf("]\n");
    
    // Write simulation outputs to CSV and TXT files.
    writeCSVFiles(positionsHistory, statusesHistory, NUM_BOIDS, NUM_STEPS, &terrainData, params.bounds);
    
    // Optionally, print a message.
    printf("Simulation complete. Output files saved in the 'output' folder.\n");
    
    // Free allocated memory.
    free(allStates);
    free(positionsHistory);
    free(statusesHistory);
    freeTerrainData(&terrainData);
    
    return 0;
}
