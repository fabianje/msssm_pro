function [is_collision, Delta_t] = is_collision(x1, x2, v1, v2, distance0)
%==========================================================================
% is_collision() predicts a collision (0 or 1) between two pedestrians
%based on the current positions and velocities.
%--------------------------------------------------------------------------
% Input:    > position and velocity of pedestrian 1 and pedestrian 2
%           > distance0: distance between 1 and 2
% Output:   > is_collision: 1 if collision is expected, 0 if collision is
%           excluded
%           > Delta_t: time to the expected collision
%==========================================================================
global r

dt = 0.1; %[s]
Delta_t = 0;
is_collision=0;
distance = 0;
k=0;

if distance0 <= 2*r || norm(v1) == 0 || distance0>12 % collision occured
    return % leave the function without executing something
end

% start "mini"-simulation 
while distance < distance0
    k=k+1;    
    %update position
    x1 = x1+dt*v1;
    x2 = x2+dt*v2;
    
    distance = distance + dt*norm(v1) + dt*norm(v2);

    % check if collision occured
    if norm(x1-x2)<2*r
        is_collision = 1;
        Delta_t = dt*k;
        break
    end
end
% Of course, the numerical effort for this simulation is much more bigger
% than compute a potential collision point. Since the colission time
% Delta_t have a big influence on the force in our model and the exact
% tedermination of this time is complicated (many special cases have to be
% considered), we decided for this conclusive solution


end














