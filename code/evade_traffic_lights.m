function [x, v] = evade_traffic_lights(traffic_lights, distance, x_old, x_new, v)
%==========================================================================
% evade_traffic_lights() checks wheter the updating position is available
% or if there is an other traffic light that prevents from update. If the
% update is not possible, the velocity is set to zero.
%--------------------------------------------------------------------------
% > Input:   > traffic_lights: location of the traffic lights
%            > distance: distance from pedestrian to traffic light
%            > x_old (current position), x_new (updating poistion), v 
%            (current velocity)
% > Output:  > corrected position x and corrected velocity v
%==========================================================================

global r
security = 1.3;
x = x_new;
dist_crit = traffic_lights;
    
% exclude all cases, where no touching of boundary points expecte
if ~traffic_lights
    return % leave this function
end

cos_alpha = dist_crit'*distance/(norm(dist_crit)*norm(distance));

if cos_alpha*norm(distance) < norm(dist_crit)-security*r || cos_alpha<=0
    % mos_alpha might be NaN, but this does not matter
    return
end

% total deacceleration it traffic light is arrived
x = x_old;
v = [0;0];

     






end

