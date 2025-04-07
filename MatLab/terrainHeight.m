%
                    %% - terrainHeight - %%
%
% Inputs:
%   - x,        double, [1x1],  X-coordinate
%   - y,        double, [1x1],  Y-coordinate
%   - params,   struct, [1x1],  Terrain parameters 
%
% Outputs:
%   - z,        double, [1x1],  Terrain height at (x,y)
%
% Computes a wave-like terrain based on amplitude, scale, base stored in
% params.
%
function z = terrainHeight(x, y, params)
%
%#codegen
%
    amplitude   = params.terrainAmplitude;
    scale       = params.terrainScale;
    base        = params.terrainBase;
%
    z           = amplitude * sin(x/scale) * cos(y/scale) + base;
%
end
%
                    %%% - END OF FUNCTION - %%%
%
