%
                            %% - runBoids - %%
%
% Inputs:
%   - numBoids,             int32,  [1x1],  Number of boids
%   - numSteps,             int32,  [1x1],  Number of simulation steps
%   - bounds,               double, [1x3],  Simulation boundaries [w, h, d]
%   - maxSpeed,             double, [1x1],  Maximum speed of boids
%   - visualRange,          double, [1x1],  Boid visual range
%   - matchingFactor,       double, [1x1],  Alignment scaling factor
%   - centringFactor,       double, [1x1],  Cohesion scaling factor
%   - avoidFactor,          double, [1x1],  Separation scaling factor
%   - minDistance,          double, [1x1],  Minimum distance for separation
%   - speedLimit,           double, [1x1],  Speed limit for boids
%   - margin,               double, [1x1],  Boundary margin and terrain 
%                                           margin
%
%   - turnFactor,           double, [1x1],  Factor for turning near 
%                                           boundaries
%
%   - targetPoint,          double, [1x3],  Target point [w, h, d]
%   - navigationGain,       double, [1x1],  Navigation gain
%   - terrainBuffer,        double, [1x1],  Vertical distance to begin 
%                                           pushing up
%
%   - terrainAvoidFactor,   double, [1x1],  Strength of upward push
%   - seed,                 int32,  [1x1],  Random seed for reproducibility
%
% Outputs:
%   - positions,    double, [numBoids x 3 x numSteps],  Boid positions 
%                                                       over time
%
%   - bounds,       double, [1x3],                      Simulation 
%                                                       boundaries
%
%   - statuses,     double, [numBoids x numSteps],      Boid status flags 
%                                                       (1=OK, 0=crashed)
%
%   - terrainData,  double, [Nx3],                      Collected terrain 
%                                                       samples [x,y,z]
%
%
function [positions, bounds, statuses, terrainData] = runBoids(numBoids,...
                                    numSteps, bounds, ...
                                    maxSpeed, visualRange, ...
                                    matchingFactor, centringFactor, ...
                                    avoidFactor, minDistance, ...
                                    speedLimit, margin, turnFactor, ...
                                    targetPoint, navigationGain, ...
                                    terrainBuffer, terrainAvoidFactor, ...
                                    seed)
%
%#codegen
%
    % 1. Random seed setting
    rng(seed);
%
    % 2. Parameters structure construction
    params.bounds             = bounds;
    params.maxSpeed           = maxSpeed;
    params.visualRange        = visualRange;
    params.matchingFactor     = matchingFactor;
    params.centeringFactor    = centringFactor;
    params.avoidFactor        = avoidFactor;
    params.minDistance        = minDistance;
    params.speedLimit         = speedLimit;
    params.margin             = margin;
    params.turnFactor         = turnFactor;
    params.targetPoint        = targetPoint;
    params.navigationGain     = navigationGain;
    params.terrainBuffer      = terrainBuffer;
    params.terrainAvoidFactor = terrainAvoidFactor;
%
    % 3. Terrain parameters generation
    params.terrainAmplitude = rand * 80;   % Random amplitude
    params.terrainScale     = 50;          % Horizontal scale for sin/cos
    params.terrainBase      = 0;           % Base offset
%
    % 4. Terrain samples data initialisation
    terrainData = zeros(0,3); 
    coder.varsize("terrainData",[50000,3],[1,0])
%
    % 5. Run the boid simulation
    [positions, statuses, terrainData] = simulateBoids(numBoids, ...
                                                       numSteps, params,...
                                                       terrainData);
%
    % 6. Return final bounds for plotting
    bounds = params.bounds;
%
end
%
                        %%% - END OF FUNCTION - %%%
%
