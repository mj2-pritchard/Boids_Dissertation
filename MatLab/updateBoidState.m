%
                        %% - updateBoidState - %%
%
% Inputs:
%   - oldState,         double, [1x7],  Boid state: [posX, posY, posZ, 
%                                                  velX, velY, velZ, flag]
%
%   - neighbourStates,  double, [Mx7],  States of neighbouring boids
%   - neighbourCount,   int32,  [1x1],  Number of valid neighbours
%   - params,           struct, [1x1],  Simulation parameters
%   - terrainData,      double, [Nx3],  Current terrain sample array
%
% Outputs:
%   - newState,         double, [1x7],  Updated boid state
%   - terrainData,      double, [Nx3],  Updated terrain samples
%
% If a boid is crashed (flag=0), it continues to be simulated. If it hits 
% terrain (z < ground + margin), it is marked crashed. Otherwise, it can 
% avoid terrain using an upward push (terrainBuffer, terrainAvoidFactor).
%
function [newState, terrainData] = updateBoidState(oldState, ...
                                    neighbourStates, neighbourCount, ...
                                    params, terrainData)
%
%#codegen
%
    % 0. If boid is already crashed, do nothing
    if oldState(7) == 0
        newState = oldState;
        return;
    end
%
    % Extract position & velocity
    pos = oldState(1:3);
    vel = oldState(4:6);
%
    %% 1. Compute Cohesion, Alignment, Separation
    cohesionSum     = [0, 0, 0];
    alignmentSum    = [0, 0, 0];
    separationVec   = [0, 0, 0];
    cohesionCount   =  0;
    alignmentCount  =  0;
%
    for i = 1:neighbourCount
        nState = neighbourStates(i,:);
        nPos   = nState(1:3);
        nVel   = nState(4:6);
%
        diffPos = pos - nPos;
        distSq  = diffPos(1)^2 + diffPos(2)^2 + diffPos(3)^2;
%
        % Cohesion + alignment if within visualRange
        if distSq < params.visualRange^2
            cohesionSum     = cohesionSum + nPos;
            alignmentSum    = alignmentSum + nVel;
            cohesionCount   = cohesionCount + 1;
            alignmentCount  = alignmentCount + 1;
        end
%
        % Separation if closer than minDistance
        if distSq < params.minDistance^2
            separationVec = separationVec + diffPos;
        end
    end
%
    % Cohesion force
    if cohesionCount > 0
        centerPos     = cohesionSum / cohesionCount;
        cohesionForce = (centerPos - pos) * params.centeringFactor;
    else
        cohesionForce = [0, 0, 0];
    end
%
    % Alignment force
    if alignmentCount > 0
        avgVel         = alignmentSum / alignmentCount;
        alignmentForce = (avgVel - vel) * params.matchingFactor;
    else
        alignmentForce = [0, 0, 0];
    end
%
    % Separation force
    separationForce = separationVec * params.avoidFactor;
%
    %% 2. Navigation toward target
    toTarget       = params.targetPoint - pos;
    distSqToTarget = sum(toTarget.^2);
    if distSqToTarget > 0
        invDist    = myInvSqrt(distSqToTarget);
        dir        = toTarget * invDist;
        desiredVel = dir * params.maxSpeed;
        steer      = desiredVel - vel;
        navForce   = params.navigationGain * steer;
    else
        navForce = [0, 0, 0];
    end
%
    %% 2.5. Terrain Avoidance Force
    groundHere   = terrainHeight(pos(1), pos(2), params);
    terrainData  = storeTerrainSample(terrainData, pos(1), pos(2), ...
                                      groundHere);
%
    distAboveGnd = pos(3) - groundHere;
    if distAboveGnd < params.terrainBuffer
        pushUp            = (params.terrainBuffer - distAboveGnd) ...
                            * params.terrainAvoidFactor;
        terrainAvoidForce = [0, 0, pushUp];
    else
        terrainAvoidForce = [0, 0, 0];
    end
%
    %% 3. Combine forces
    totalForce = cohesionForce + alignmentForce + separationForce ...
                 + navForce + terrainAvoidForce;
    newVel     = vel + totalForce;
%
    % Enforce speed limit
    speedSq = sum(newVel.^2);
    if speedSq > params.speedLimit^2
        scale  = params.speedLimit * myInvSqrt(speedSq);
        newVel = newVel * scale;
    end
%
    % Preliminary new position
    newPos = pos + newVel;
%
    %% 4. Boundary Enforcement (X, Y)
    if newPos(1) < params.margin
        newVel(1) = newVel(1) + params.turnFactor;
    elseif newPos(1) > (params.bounds(1) - params.margin)
        newVel(1) = newVel(1) - params.turnFactor;
    end
%
    if newPos(2) < params.margin
        newVel(2) = newVel(2) + params.turnFactor;
    elseif newPos(2) > (params.bounds(2) - params.margin)
        newVel(2) = newVel(2) - params.turnFactor;
    end
%
    % Recompute newPos after boundary changes
    newPos = pos + newVel;
%
    %% 5. Final terrain collision check
    groundLevel  = terrainHeight(newPos(1), newPos(2), params);
    terrainData  = storeTerrainSample(terrainData, newPos(1), newPos(2),...
                                      groundLevel);
%
    if newPos(3) < (groundLevel + params.margin)
        % boid crashes
        newState = [newPos, 0, 0, 0, 0];
    else
        % optional final speed re-check
        speedSq = sum(newVel.^2);
        if speedSq > params.speedLimit^2
            scale  = params.speedLimit * myInvSqrt(speedSq);
            newVel = newVel * scale;
            newPos = pos + newVel;
        end
        newState = [newPos, newVel, 1];
    end
%
end
%
                    %%% - END OF FUNCTION - %%%
%
