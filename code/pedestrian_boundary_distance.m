function [dist_min, dist_critical, traffic_lights] = pedestrian_boundary_distance(x)
%--------------------------------------------------------------------------
% Input:    > pedestrian and Grid
% Output:   > dist_critical: cell array with dist_crit (vector
%           pointing from center of current pedestrian to nearest boundary
%           point)and boundary type (string with type of boundary point)
%           > distance: vector pointing from nearest boundary point to
%           pedestrian (center)
%--------------------------------------------------------------------------

global r Nx Ny N M with_roundabout with_middle_line with_traffic_lights ...
    with_4_roundabouts
distance_saver = zeros(3, 4);
load('variables', 'bounds') % load struct profile

dist_critical = cell(1,3);

C = [round(x(2)/Ny); round(x(1)/Nx)]; % indices of current pedestrian

%--------------------------------------------------------------------------
% (1) CONSIDER PROFILE BOUNDS
profile = bounds.profile;
[np, mp] = size(profile);
distance = [N; M];

for k=1:mp-1
    A = profile(:,k);
    B = profile(:,k+1);
    distance = find_min_distance(A,B,C,distance,false); 
end

if with_traffic_lights
    traffic_lights = bounds.traffic_lights;
    [nt, mt] = size(traffic_lights);
    for k = 1:2:mt-1
        B = traffic_lights(:,k);
        A = traffic_lights(:,k+1);
        [distance] = find_min_distance(A,B,C,distance,false);
    end 
end

distance_saver(:,1) = [norm(distance); distance];


%--------------------------------------------------------------------------
% (2) CONSIDER OUTER CIRCLE BOUNDS
if with_roundabout || with_4_roundabouts
    distance = [N; M];
    outer_circle = bounds.outer_circle;
    [ns, ms] = size(outer_circle);
    for k=1:ms
        circle = outer_circle(:,k);
        R = circle(1)/Nx;       % radius
        Mid = circle(2:3);      % center point (indices)
        dist = (norm(C-Mid)-R)*(C-Mid)/norm(C-Mid); % vector pointing from boundary to pedestrian
        if norm(dist)<norm(distance)
            distance = dist;
        end
    end
    distance_saver(:,2) = [norm(distance); distance];
else
    distance_saver(:,2) = [norm([N; M]); [N; M]];
end

%--------------------------------------------------------------------------
% (4) CONSIDER INNER CIRCLE
% This step is not neccessary since the profile desribes the circle well
% (for big sufficiently big radius)

%--------------------------------------------------------------------------
% (3) CONSIDER CORNER BOUNDS
corner  = bounds.corner;
[nc, mc] = size(corner);
distance = [N; M];
for k=1:mc
    dist = C - corner(:,k); % vector pointing from boundary point to pedestrian
    % find local minimum
    if norm(dist) < norm(distance)
        distance = dist;
    end 
end
distance_saver(:,3) = [norm(distance); distance];

%--------------------------------------------------------------------------
% (4) CONSIDER STRAIGHT LINES
if with_middle_line
    straight_lines = bounds.straight_lines;
    [nl, ml] = size(straight_lines);
    distance = [N; M];
    for k = 1:2:ml-1
        line = straight_lines(:,k:k+1);
        A = line(:,1);
        B = line(:,2);
        [distance] = find_min_distance(A,B,C,distance,true);
    end
    distance_saver(:,4) = [norm(distance); distance];
end

%--------------------------------------------------------------------------
% (5) CONSIDER TRAFFIC LIGHTS
if with_traffic_lights
    distance = [N; M];
    for k = 1:2:mt-1
        A = traffic_lights(:,k);
        B = traffic_lights(:,k+1);
        [distance] = find_min_distance(A,B,C,distance,false);
    end
    
    if norm(distance) == norm([N; M]) % nothing found
        traffic_lights = false;
    else
        traffic_lights = -[distance(2)*Nx; distance(1)*Ny];
    end
    
else
    traffic_lights = false;
end



%--------------------------------------------------------------------------
% FIND GLOBAL MINIMUM
dist_min = [N; M];
for k=1:4
    distance = distance_saver(1,k);
    if distance>0 && distance<norm(dist_min);
        dist_min = distance_saver(2:3,k);
        p = k;
    end
end

if isempty(p)
    dist_type = 'empty';
    dist_critical{1} = 'empty';
elseif p==1
    dist_type = 'profile';
elseif p==2
    dist_type = 'outer_circle';
elseif p==3
    dist_type = 'corner';
elseif p==4
    dist_type = 'straight_lines';
end

% vector pointing from boundary point to pedestrian
critical = false;
dist_min = [dist_min(2)*Nx; dist_min(1)*Ny]; 
if norm(dist_min)<=3*r
    critical = true;
end 

if critical
    dist_critical{2} = -dist_min; % vector pointing from pedestrian to boundary point
    dist_critical{1} = dist_type;
else
    dist_critical{1} = 'empty';
end



end

