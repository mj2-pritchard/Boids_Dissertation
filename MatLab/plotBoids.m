%
                    %% - plotBoids - %%
%
% Inputs:
%   - positions,    double, [numBoids x 3 x numSteps],  Boid positions
%   - statuses,     double, [numBoids x numSteps],      Boid flags (1=OK, 
%                                                       0=crashed)
%
%   - bounds,       double, [1x3],                      Simulation 
%                                                       boundaries
%
%   - terrainData,  double, [Nx3],                      Sampled terrain 
%                                                       points
%
% Outputs:
%   - 3D Plot Animation
%
function plotBoids(positions, statuses, bounds, terrainData)
%
    [numBoids, ~, numSteps] = size(positions);
%
    % Build a terrain surface from scattered data
    if ~isempty(terrainData)
        F = scatteredInterpolant(terrainData(:,1), terrainData(:,2), terrainData(:,3), ...
                                 'natural', 'none');
        xGrid   = linspace(0, bounds(1), 50);
        yGrid   = linspace(0, bounds(2), 50);
        [X, Y]  = meshgrid(xGrid, yGrid);
        Z       = F(X, Y);
    end
%
    figure;
    for step = 1:numSteps
        clf;
        hold on;
        if ~isempty(terrainData)
            surf(X, Y, Z, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        end
%
        for i = 1:numBoids
            if statuses(i, step) == 1
                col = 'g';  % alive
            else
                col = 'r';  % crashed
            end
            plot3(positions(i,1,step), positions(i,2,step), positions(i,3,step), ...
                'o', 'MarkerFaceColor', col, 'MarkerEdgeColor', col);
        end
%
        xlim([0, bounds(1)]);
        ylim([0, bounds(2)]);
        zlim([0, bounds(3)]);
        grid on;
        title(['3D Simulation - Step ' num2str(step)]);
        xlabel('X'); ylabel('Y'); zlabel('Z');
        view(3);
        drawnow;
        pause(0.05);
    end
end
%
                    %%% - END OF FUNCTION - %%%
%
