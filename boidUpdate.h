#ifndef BOIDUPDATE_H
#define BOIDUPDATE_H

#ifdef __cplusplus
extern "C" {
#endif

// Each boid state is stored as 7 doubles:
// [posX, posY, posZ, velX, velY, velZ, flag]
#define BOID_STATE_SIZE 7

// Structure to hold all simulation parameters.
typedef struct {
    double bounds[3];          // [width, height, depth]
    double maxSpeed;
    double visualRange;
    double matchingFactor;
    double centeringFactor;
    double avoidFactor;
    double minDistance;
    double speedLimit;
    double margin;
    double turnFactor;
    double targetPoint[3];
    double navigationGain;
    double terrainBuffer;
    double terrainAvoidFactor;
    // Terrain parameters
    double terrainAmplitude;
    double terrainScale;
    double terrainBase;
} BoidParams;

// Initializes simulation parameters and random seed.
void initParameters(BoidParams *params, int seed);

// Updates one boid (index i) given the full array of boid states.
void updateOneBoid(double *allStates, int i, int numBoids, const BoidParams *p);

// Updates a subset of boids (from startIdx to endIdx) in one simulation step.
void stepBoidsSubset(double *allStates, int numBoids, int startIdx, int endIdx, const BoidParams *p);

// Exposes the terrain–height function so that other modules can query it.
double getTerrainHeight(double x, double y, const BoidParams *p);

#ifdef __cplusplus
}
#endif

#endif // BOIDUPDATE_H
