function [availability, position_velocity] = is_available2(potential_blocking_indices, i_new, j_new, v)
%==========================================================================
% This function check whether the updating position is available or or not.
%--------------------------------------------------------------------------
% Input:    > potential_blocking_indices: indices of neighbour pedestrians
%           > i_new, j_new: indices of updating position
%           > velocity v
% Output:   > availability (true or false)
%==========================================================================

global Nx Ny r

security = 1.1;
local_availability = true;
availability = true;
position_velocity = [];

[n, m] = size(potential_blocking_indices);
if m==0 || v==0
    return
end

for i=1:m
    % Extract indicex and velocity of neighbour pedestrian
    i_blocking = potential_blocking_indices(1, i);
    j_blocking = potential_blocking_indices(2, i);
    v_blocking = potential_blocking_indices(3:4,i);
    
    % Check if the updating position is around the neighbour location
    r12 = norm([(j_blocking-j_new)*Nx; (i_blocking-i_new)*Ny]);
    local_availability = r12 > 2*r*security;
    % Exlude boundary points from this procedure: Sometimes pedestrian
    % move backward after its generating and an other pedestrian will
    % generated on the current one.
    if r12-2*r<0
        continue
    elseif local_availability==false
        availability = false;   % remember that normal uptading is not possible
        % save position and velocity of "critical" neithbour
        position_velocity = [position_velocity, [j_blocking*Nx; i_blocking*Ny; v_blocking] ];
        local_availability = true; % make ready for next iteration
    end
end


end

