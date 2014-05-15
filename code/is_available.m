function [is_available] = is_available(Grid, r, i_new, j_new)
%==========================================================================
% This function verifies wether the updating position is free or occupied
% (with a pedestrian of a grid point)
% ATTENTION: This function strongly deaccelerates the updating speed! If
% possible use is_available2().
%--------------------------------------------------------------------------
% Input:    > Grid
%           > radius of the pedestrian r
%           >updating indices (i_new, j_new)
% Output:   > is_available (1: position is available, 0: position is
%           not available)
%==========================================================================

global N M Nx Ny

is_available = 1;
security = 2;

% iterate over a square with width 4*r and middle point (i_new, j_new)
for i=i_new-2*r*security/Ny : i_new+2*r*security/Ny
    for j=j_new-2*r*security/Nx : j_new+2*r*security/Nx
        if i>0 && j>0 && i<N && j<M % get sure that position exists
            % take care of neighbour pedestria
            if (Grid(i, j) > 0) && Grid(i,j)~=inf
                % calculate vector pointing from updating position to
                % neighbour position
                vec = [j*Nx; i*Ny] - [j_new*Nx; i_new*Ny];
                dist = norm(vec);
                if dist<(2*r)*security
                    is_available = 0;
                    %disp('warning: update was not possible -- pedestrian crossing')
                    return
                end
            % take care of boundary points
            elseif i>i_new-r/Ny && i<i_new+r/Ny && j>j_new-r/Nx && j<j_new+r/Nx && Grid(i, j)==inf
                is_available = 0;
                %disp('warning: update was not possible -- boundary crossing')
                return
            end            
        end
    end
end



end

