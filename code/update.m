function [Grid, pedestrian_saver, deleted_pedestrians, measurements_1, measurements_2, measurements_3, time_effort] ...
    = update(Grid, pedestrian_saver, p)
%==========================================================================
% This fnction updates the position if the velocity is given.
%--------------------------------------------------------------------------
% Input:    > Grid
%           > pedestrian_saver
%           > p: number of pedestrians
% Output:   > updated Grid, updated pedestrian_saver
%           > deleted_pedestrians: array that contains indices of
%           pedestrians which have left the grid
%           > avaraged measurements (1, 2 and 3)
%==========================================================================

global N M Nx Ny with_traffic_lights
load('variables', 'time');
deleted_pedestrians = [];
measurements_1 = zeros(p,3);
measurements_2 = [];
measurements_3 = zeros(N,M);
time_effort = [];

% Note: Since some pedestrian can react on decisions of other pedestrians
% (the react on the forces), this function can not be involved in the loop 
% below (However, this would would be much more efficient)

[pedestrian_saver, potential_blocking_indices] = update_force(pedestrian_saver, Grid);

if with_traffic_lights
    traffic_lights_regulator()
end


for k=1:p
    current = pedestrian_saver{k}; % extract current pedestrian
    i = current.indices(1);        % extract its position
    j = current.indices(2);
    % compute the force that acts on the current pedestrian
    f = current.force;
    [x, v] = update_position(f, current.position, current.velocity, current.mass); % update position
    distance = x-current.position; % vector pointing from current position to updating position
    if ~strcmp(current.dist_critical{1},'empty') 
        % check if current postion is critical (any boundary points around?)
        [x1, v1] = evade_boundary(current.dist_critical, distance, current.position,x, v);
    else
        x1 = x;
        v1 = v;
    end
    % Take care of traffic lights
    if with_traffic_lights
        [x, v] = evade_traffic_lights(current.traffic_lights, distance, current.position, x1, v1);
    else
        x = x1;
        v = v1;
    end
    
    % check if updating position is available
    [availability_pedestrian, neighbour] = is_available2(potential_blocking_indices{k}, floor(x(2)/Ny), floor(x(1)/Nx), norm(v)); 
    if ~availability_pedestrian
        [x, v] = evade_pedestrian(x,current.position, neighbour(1:2,:), v1, neighbour(3:4,:));      
    end
    
    % new grid coordinates
    i_new = floor(x(2)/Ny);
    j_new = floor(x(1)/Nx);
    
    % is the pedestrian located on the grid?
    if (i_new <= N && j_new <= M) && (i_new > 0 && j_new >0 )
        % save id on the grid
        Grid(i_new,j_new) = k; 
        % save new state
        pedestrian_saver{k}.indices = [i_new; j_new];
        pedestrian_saver{k}.position = x;
        pedestrian_saver{k}.velocity = v;
        % release old position
        Grid(i,j) = 0; 
       
        % take some "measurements" and save them
        measurements_1(k,1) = norm(v);
        measurements_1(k,2) = norm(current.initial_velocity);
        measurements_1(k,3) = norm(v)<0.2;
        if current.initial_velocity>=1.5 % fast pedestrians
            measurements_2 = [measurements_2; norm(v)];
        end
        measurements_3(i_new, j_new) = norm(v);
    % Special case for pedestrians that have reached its target
    elseif ((i_new > N) || (j_new > M) || (i_new <= 0) || (j_new <= 0))
        deleted_pedestrians = [deleted_pedestrians k];
        Grid(i,j) = 0;
        % take measuremetns here. Get also sure that the pedestrian do not
        % walk  backwards after beeing generated (this distorts the
        % measurements). Exclude those cases.
        if (time-current.generation_time>4)
            time_effort = [time_effort, time-current.generation_time];
        end
    else
        warning('updating position is NaN') % tell me we have problems here
    end   
    % If grid point is occupied then do not update. This, of course, sould 
    % influence the velocity. Since this situation rarely occures,
    % the effect of deacceleration is not considered here.
end


if p~= 0
    % avarage_velocity(1): avarage velocity
    % avarage_velocity(2): avarage of initial velocity (preferable velocity)
    measurements_1 = sum(measurements_1)/p;
else
    measurements_1 = [inf;inf;inf];   % mark this case with infinity values
end

if ~isempty(measurements_2)
    measurements_2 = sum(measurements_2)/length(measurements_2);
else 
    measurements_2 = inf;       % mark this case with infintiy value
end

if ~isempty(time_effort)
    time_effort = sum(time_effort)/length(time_effort);
else
    time_effort=0;              % mark this case with zero value
end

