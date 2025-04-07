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
    double bounds[3];           // Simulation Boundaries [x, y, z]
    double maxSpeed;            // Maximum speed of boids
    double visualRange;         // Boid visual range
    double matchingFactor;      // Alignment scaling factor
    double centeringFactor;     // Cohesion scaling factor
    double avoidFactor;         // Separation scaling factor
    double minDistance;         // Minimum distance for separation
    double speedLimit;          // Speed limit for boids
    double margin;              // Boundary margin
    double turnFactor;          // Factor for turning near boundaries
    double targetPoint[3];      // Target Point [x, y, z]
    double navigationGain;      // Navigation gain
    double terrainBuffer;       // Vertical distance to begin pushing up
    double terrainAvoidFactor;  // Strength of upward push
    double terrainAmplitude;    // Random terrain amplitude
    double terrainScale;        // Horizontal scale for sin/cos
    double terrainBase;         // Base offset
} BoidParams;

// Initialises simulation parameters and random seed.
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
