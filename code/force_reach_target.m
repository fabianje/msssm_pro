function [f, location] = force_reach_target(p, location, Grid)
%==========================================================================
% First of all, the pedestrian should understand the situation in which he
% is, i.e he has to to do the following things (in order of importante)
%   (1) Check whether the next target point is visible. If yes, go to this
%   point.
%   (2) Check whether the current target is visible (the pedestrian might be
%   walkt backwards). If yes, go back to the previous target point
% pedestrian tries to reach his target with a preferable velocity v0
% (which is restricted with vmax and vmin). The force that
% correct the velocity direction points from the pedestrian to the target.
%--------------------------------------------------------------------------
%Input:     > number of pedestrian p
%           > location: counter in the vector that saves the target points
%           > Grid 
% Output:   > firc t
%           > location
%==========================================================================

global target_radius with_middle_line with_roundabout ax bx Nx Ny r ...
    with_traffic_lights

% time constant (reaction time) for the "approach-target"-force
% This parameter can be also interpreted as an agressivity indicator
tau = 0.005;

vmin = p.v_boundaries(1);
vmax = p.v_boundaries(2);

if norm(p.target_position(:,location)-p.target_position(:,end)) ~= 0
    % check if target point is visible
    if Grid(p.target_indices(1,location), p.target_indices(2,location)) == inf
        error('target point inside boundary') 
    end
    
    % Decide how much the pedestrian should approach the target point
    % the following values are estimated using graphical interpretation
    if with_middle_line && norm(p.target_position(:,location) - p.target_position(:,end-1))==0
        approach_target_distance = (bx-ax)*Nx/4; % approach second last target as near as possible
    elseif with_roundabout && location~=1 && norm(p.target_position(:,location) - p.target_position(:,end-1))~=0 && p.density>0
        % do not walk to the center of the circle. This will cause a total blocking situation.
        approach_target_distance = 1;
    elseif with_traffic_lights && location==1
        approach_target_distance = (bx-ax)*Nx/4;
    else
        approach_target_distance = target_radius;
    end
else
    approach_target_distance = target_radius;
end


% take care of multiple updating of the target position
if norm(p.target_position(:,location)-p.target_position(:,end))~=0 && location~=1
    loc_current = norm(p.position - p.target_position(:,location));     % distance to current target
    visibility = is_visible(p.indices(1), p.indices(2),p.target_indices(1, location+1), p.target_indices(2,location+1), Grid);
    ready_to_update = (loc_current<approach_target_distance) && visibility;
elseif location==1 && norm(p.target_position(:,location)-p.target_position(:,end))~=0
    loc_current = norm(p.position - p.target_position(:,location));     % distance to current target
    visibility = is_visible(p.indices(1), p.indices(2),p.target_indices(1, location+1), p.target_indices(2,location+1), Grid);
    ready_to_update = loc_current<approach_target_distance && visibility;
else % location = last target
    ready_to_update = false; % no more targets
end

% Check if the pedestrian can focus the the next target point
if with_roundabout && location~=1 && norm(p.target_position(:,location)-p.target_position(:,end))~=0 ...
        && norm(p.target_position(:,location)-p.target_position(:,end-1))~=0 ...
        && norm(p.target_position(:,location)-p.target_position(:,end-2))~=0
    loc_next = norm(p.position - p.target_position(:,location+1));      % distance to next target
    ready_to_update = ready_to_update && (loc_next<2*approach_target_distance);
end
 
% approach the current target as near possible. If the target visible, than
% update to the next target.
if ready_to_update
    location = location+1;
end

% Sometimes the pedestrian walks backwards and the current target becomes
% suddenly unvisible. In that case, switch the previous target.
% Include also the critical cases, where the center of the pedestrian can
% see the target but some border-points of the pedestrian do not (this
% special case casues blocking pedestrian on edge points)
if location>1
    if is_visible(p.indices(1), p.indices(2), p.target_indices(1, location), p.target_indices(2,location), Grid)==0
        location = max(location-1,1); % pedestrian can not see current target
    elseif norm(p.velocity)<0.1 % now check if pedestrian is blocked
        visibility = true; % first estimation
        % 4 iterations around the center point (be a little bit
        % conservative and add the factor 1.4)
        for i= round([-1:2:1].*r/Ny*1.4)
            for j= round([-1:2:2].*r/Nx*1.4)
                if is_visible(p.indices(1)+i, p.indices(2)+j, p.target_indices(1, location), p.target_indices(2,location), Grid)==0
                    visibility = false; % corrected estimation
                end
            end
        end
        if ~visibility % pedestrian can see target but something prevent him from moving 
            location = max(location-1,1); % go back and try again
        end
    end
end

% get current target
r_target = p.target_position(:,location);

% If no other forces act on the pedestrian, he will walk with its
% preferable velocity v0. He tries to keep his velocity constant at this
% value.
v0 = p.initial_velocity;
v_opt = v0*(r_target - p.position)/norm(r_target - p.position);

% get sure that the pedestrian is not borred (restriction v_opt > vmin)
% and not overstrained (restriction v_opt < vmax)
v_opt = v_opt/norm(v_opt) * max(min(norm(v_opt), vmax), vmin);

f = (v_opt - p.velocity)/tau;

if isnan(norm(f)) && norm(p.velocity)==0
    if isnan(norm(f))
        warning('force is NaN')
    end
    f=[0;0] ;
end

end

