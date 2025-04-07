%
                %% - storeTerrainSample - %%
%
% Inputs:
%   - terrainData, double, [Nx3], Existing terrain sample array
%   - x,           double, [1x1], X-coordinate
%   - y,           double, [1x1], Y-coordinate
%   - z,           double, [1x1], Computed terrain height
%
% Outputs:
%   - terrainData, double, [Nx3], Updated terrain samples with the row 
%                                 [x,y,z] appended
%
function terrainData = storeTerrainSample(terrainData, x, y, z)
%#codegen
    terrainData = [terrainData; x, y, z];
%
end
%
                %%% - END OF FUNCTION - %%%
%
