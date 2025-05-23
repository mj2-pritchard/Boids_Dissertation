#include <math.h>
#include <stdlib.h>
#include "boidUpdate.h"

// - myInvSqrt Function - //

// Inputs:
//   - x,    double, [1x1], Value for which to compute 1/sqrt(x)

static double myInvSqrt(double x)
{
    if (x <= 0.0)
        return 0.0;
    double out = 1.0;
    for (int i = 0; i < 5; i++) {
        out = 0.5 * (out + 1.0 / (x * out));
    }
    return out;
}

// - End of myInvSqrt Function - //

// ----------------------------- //

// - terrainHeight Function - //

// Inputs:

//   - x,           double, [1x1],  X-coordinate
//   - y,           double, [1x1],  Y-coordinate
//   - BoidParams,  struct,      ,  Terrain parameters 

// Internal function that Computes a wave-like terrain based on amplitude, scale, base stored in params.

static double terrainHeight(double x, double y, const BoidParams *p)
{
    return p->terrainAmplitude * sin(x / p->terrainScale) * cos(y / p->terrainScale) + p->terrainBase;
}

// - End of terrainHeight Function - //

// ----------------------------- //

// - getTerrainHeight Function - //

// Exported wrapper to allow other modules to call terrainHeight.

double getTerrainHeight(double x, double y, const BoidParams *p)
{
    return terrainHeight(x, y, p);
}

// - End of getTerrainHeight Function - //

// ----------------------------- //

// - initParameters Function - //

// Initialise simulation parameters and set a random seed.

void initParameters(BoidParams *p, int seed)
{
    srand(seed);
    // Set simulation parameters.
    p->bounds[0]            = 2000.0;   // Simulation Boundary (x)
    p->bounds[1]            = 2000.0;    // Simulation Boundary (y)
    p->bounds[2]            = 400.0;    // Simulation Boundary (z)
    p->maxSpeed             = 5.0;      // Maximum speed of boids
    p->visualRange          = 50.0;     // Boid visual range
    p->matchingFactor       = 0.05;     // Alignment scaling factor
    p->centeringFactor      = 0.01;     // Cohesion scaling factor
    p->avoidFactor          = 0.01;     // Separation scaling factor
    p->minDistance          = 5.0;      // Minimum distance for separation
    p->speedLimit           = 3.0;      // Speed limit for boids
    p->margin               = 0.5;      // Boundary margin
    p->turnFactor           = 1.0;      // Factor for turning near boundaries
    p->targetPoint[0]       = 100.0;    // Target Point (x)
    p->targetPoint[1]       = 150.0;    // Target Point (y)
    p->targetPoint[2]       = 20.0;     // Target Point (z)
    p->navigationGain       = 0.05;     // Navigation gain
    p->terrainBuffer        = 10.0;     // Vertical distance to begin pushing up
    p->terrainAvoidFactor   = 0.6;      // Strength of upward push
    p->terrainAmplitude     = ((double)rand() / RAND_MAX) * 80.0; // Random terrain amplitude
    p->terrainScale         = 50.0;     // Horizontal scale for sin/cos
    p->terrainBase          = 0.0;      // Base offset
}

// - End of initParameters Function - //

// ----------------------------- //

// - updateOneBoid Function - //

// Core function: update the state of one boid (index i) based on its neighbours.

void updateOneBoid(double *allStates, int i, int numBoids, const BoidParams *p)
{
    double *myState = &allStates[i * BOID_STATE_SIZE];
    if (myState[6] == 0.0) {
        // Already crashed.
        return;
    }
    
    // Extract current position and velocity.
    double pos[3] = { myState[0], myState[1], myState[2] };
    double vel[3] = { myState[3], myState[4], myState[5] };
    
    // Initialise sum arrays for cohesion, alignment, and separation.
    double cohesionSum[3]   = { 0.0, 0.0, 0.0 };
    double alignmentSum[3]  = { 0.0, 0.0, 0.0 };
    double separationVec[3] = { 0.0, 0.0, 0.0 };
    int neighbourCount      = 10;
    
    // Loop over all other boids.
    for (int j = 0; j < numBoids; j++) {
        if (j == i)
            continue;
        double *nbr     = &allStates[j * BOID_STATE_SIZE];
        double diff[3]  = { pos[0] - nbr[0], pos[1] - nbr[1], pos[2] - nbr[2] };
        double distSq   = diff[0]*diff[0] + diff[1]*diff[1] + diff[2]*diff[2];
        
        // If within visual range, add for cohesion and alignment.
        if (distSq < p->visualRange * p->visualRange) {
            cohesionSum[0]  += nbr[0];
            cohesionSum[1]  += nbr[1];
            cohesionSum[2]  += nbr[2];
            alignmentSum[0] += nbr[3];
            alignmentSum[1] += nbr[4];
            alignmentSum[2] += nbr[5];
            neighbourCount++;
        }
        // If too close, add for separation.
        if (distSq < p->minDistance * p->minDistance) {
            separationVec[0] += diff[0];
            separationVec[1] += diff[1];
            separationVec[2] += diff[2];
        }
    }
    
    double cohesionForce[3]  = { 0.0, 0.0, 0.0 };
    double alignmentForce[3] = { 0.0, 0.0, 0.0 };
    if (neighbourCount > 0) {
        double center[3]  = { cohesionSum[0] / neighbourCount,
                              cohesionSum[1] / neighbourCount,
                              cohesionSum[2] / neighbourCount };
        cohesionForce[0]  = (center[0] - pos[0]) * p->centeringFactor;
        cohesionForce[1]  = (center[1] - pos[1]) * p->centeringFactor;
        cohesionForce[2]  = (center[2] - pos[2]) * p->centeringFactor;
        
        double avgVel[3]  = { alignmentSum[0] / neighbourCount,
                              alignmentSum[1] / neighbourCount,
                              alignmentSum[2] / neighbourCount };
        alignmentForce[0] = (avgVel[0] - vel[0]) * p->matchingFactor;
        alignmentForce[1] = (avgVel[1] - vel[1]) * p->matchingFactor;
        alignmentForce[2] = (avgVel[2] - vel[2]) * p->matchingFactor;
    }
    
    double separationForce[3] = { separationVec[0] * p->avoidFactor,
                                  separationVec[1] * p->avoidFactor,
                                  separationVec[2] * p->avoidFactor };
    
    // Navigation force: steer toward target point.
    double toTarget[3]  = { p->targetPoint[0] - pos[0],
                            p->targetPoint[1] - pos[1],
                            p->targetPoint[2] - pos[2] };
    double distSqTarget = toTarget[0]*toTarget[0] + toTarget[1]*toTarget[1] + toTarget[2]*toTarget[2];
    double navForce[3]  = { 0.0, 0.0, 0.0 };
    if (distSqTarget > 0.0) {
        double invDist       = myInvSqrt(distSqTarget);
        double desiredVel[3] = { toTarget[0] * invDist * p->maxSpeed,
                                 toTarget[1] * invDist * p->maxSpeed,
                                 toTarget[2] * invDist * p->maxSpeed };
        navForce[0] = (desiredVel[0] - vel[0]) * p->navigationGain;
        navForce[1] = (desiredVel[1] - vel[1]) * p->navigationGain;
        navForce[2] = (desiredVel[2] - vel[2]) * p->navigationGain;
    }
    
    // Terrain avoidance force.
    double ground           = terrainHeight(pos[0], pos[1], p);
    double distAboveGnd     = pos[2] - ground;
    double terrainAvoid[3]  = { 0.0, 0.0, 0.0 };
    if (distAboveGnd < p->terrainBuffer) {
        terrainAvoid[2] = (p->terrainBuffer - distAboveGnd) * p->terrainAvoidFactor;
    }
    
    // Combine all forces.
    double totalForce[3] = { 
        cohesionForce[0] + alignmentForce[0] + separationForce[0] + navForce[0] + terrainAvoid[0],
        cohesionForce[1] + alignmentForce[1] + separationForce[1] + navForce[1] + terrainAvoid[1],
        cohesionForce[2] + alignmentForce[2] + separationForce[2] + navForce[2] + terrainAvoid[2]
    };
    
    // Update velocity.
    vel[0] += totalForce[0];
    vel[1] += totalForce[1];
    vel[2] += totalForce[2];
    
    // Enforce speed limit.
    double speedSq = vel[0]*vel[0] + vel[1]*vel[1] + vel[2]*vel[2];
    if (speedSq > p->speedLimit * p->speedLimit) {
        double scale = p->speedLimit * myInvSqrt(speedSq);
        vel[0] *= scale;
        vel[1] *= scale;
        vel[2] *= scale;
    }
    
    // Compute new position.
    double newPos[3] = { pos[0] + vel[0], pos[1] + vel[1], pos[2] + vel[2] };
    
    // Boundary enforcement for X and Y.
    if (newPos[0] < p->margin)
        vel[0] += p->turnFactor;
    else if (newPos[0] > (p->bounds[0] - p->margin))
        vel[0] -= p->turnFactor;
    
    if (newPos[1] < p->margin)
        vel[1] += p->turnFactor;
    else if (newPos[1] > (p->bounds[1] - p->margin))
        vel[1] -= p->turnFactor;
    
    // Recompute new position after boundary adjustment.
    newPos[0] = pos[0] + vel[0];
    newPos[1] = pos[1] + vel[1];
    newPos[2] = pos[2] + vel[2];
    
    // Final terrain collision check.
    double newGround = terrainHeight(newPos[0], newPos[1], p);
    if (newPos[2] < newGround + p->margin) {
        // Mark boid as crashed.
        myState[0] = newPos[0];
        myState[1] = newPos[1];
        myState[2] = newPos[2];
        myState[3] = 0.0;
        myState[4] = 0.0;
        myState[5] = 0.0;
        myState[6] = 0.0;
    } else {
        // Update boid state.
        myState[0] = newPos[0];
        myState[1] = newPos[1];
        myState[2] = newPos[2];
        myState[3] = vel[0];
        myState[4] = vel[1];
        myState[5] = vel[2];
        myState[6] = 1.0;
    }
}

// - End of updateOneBoid Function - //

// ----------------------------- //

// Update a subset of boids (from startIdx to endIdx) in one simulation step.
void stepBoidsSubset(double *allStates, int numBoids, int startIdx, int endIdx, const BoidParams *p)
{
    for (int i = startIdx; i < endIdx; i++) {
        updateOneBoid(allStates, i, numBoids, p);
    }
}
