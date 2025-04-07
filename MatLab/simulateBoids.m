%
                    %% - simulateBoids - %%
%
% Inputs:
%   - numBoids,     int32,  [1x1], Number of boids
%   - numSteps,     int32,  [1x1], Number of simulation timesteps
%   - params,       struct, [1x1], Simulation parameters
%   - terrainData,  double, [Nx3], Existing terrain samples
%
% Outputs:
%   - positions,    double, [numBoids x 3 x numSteps],  Boid positions
%   - statuses,     double, [numBoids x numSteps],      Boid status flags 
%                                                       (1=OK, 0=crashed)
%
%   - terrainData,  double, [Nx3],                      Updated terrain 
%                                                       samples
%
function [positions, statuses, terrainData] = simulateBoids(numBoids, ...
                                                numSteps, params, ...
                                                terrainData)
%
%#codegen
%
    % Each boid state: [posX, posY, posZ, velX, velY, velZ, flag]
    boidStates = zeros(numBoids, 7);
    positions  = zeros(numBoids, 3, numSteps);
    statuses   = zeros(numBoids, numSteps);
%
    % --- 1. Initialise boids ---
    for i = 1:numBoids
        x = params.bounds(1) * rand;
        y = params.bounds(2) * rand;
%
        % Compute terrain height, store sample
        groundLevel = terrainHeight(x, y, params);
        terrainData = storeTerrainSample(terrainData, x, y, groundLevel);
%
        zMin = groundLevel + params.margin;
        if zMin < 0
            zMin = 0;
        end
        z = zMin + rand * (params.bounds(3) - zMin);
%
        vx = randn * params.maxSpeed;
        vy = randn * params.maxSpeed;
        vz = randn * params.maxSpeed;
%
        boidStates(i,:) = [x, y, z, vx, vy, vz, 1];  % flag=1 => OK
        positions(i,:,1) = [x, y, z];
        statuses(i,1)    = 1;
    end
%
    % --- 2. Main simulation loop ---
    for step = 2:numSteps
        newStates = zeros(numBoids, 7);
        for i = 1:numBoids
            % Build neighbour list
            neighbourStates  = zeros(numBoids-1, 7);
            idx             = 0;
            for j = 1:numBoids
                if j ~= i
                    idx                     = idx + 1;
                    neighbourStates(idx,:)   = boidStates(j,:);
                end
            end
            neighbourCount = idx;
%
            % Update single boid (includes terrain calls)
            [newStates(i,:), terrainData] = updateBoidState( ...
                boidStates(i,:), neighbourStates, neighbourCount, ...
                params, terrainData);
        end
%
        % Copy new states
        boidStates = newStates;
        for i = 1:numBoids
            positions(i, :, step) = boidStates(i, 1:3);
            statuses(i,     step) = boidStates(i, 7);
        end
    end
%
end
%
                    %%% - END OF FUNCTION - %%%
%
