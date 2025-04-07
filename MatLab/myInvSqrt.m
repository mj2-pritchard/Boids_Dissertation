%
                        %% - myInvSqrt - %%
%
% Inputs:
%   - x,    double, [1x1], Value for which to compute 1/sqrt(x)
%
% Outputs:
%   - out,  double, [1x1], Approximation of 1/sqrt(x)
%
function out = myInvSqrt(x)
%#codegen
    if x <= 0
        out = 0;
        return;
    end
    out = 1; % initial guess
    for k = 1:5
        out = 0.5 * (out + 1.0./(x * out));
    end
%    
                    %%% - END OF FUNCTION - %%%
%
