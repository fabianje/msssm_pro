function [pedestrian_saver, Grid] = update_pedestrians(pedestrian_saver, deleted_pedestrians, Grid)
%==========================================================================
% This function delets pedestrians that have left the grid and generates
% new pedestrians on the boundary points.
%--------------------------------------------------------------------------
% Input:    > pedestrian_saver: cell array that contains all pedestrians
%           > deleted_pedestrians: array that contains indices of
%           pedestrians which have left the grid
%           > Grid
% Output:   > updated pedestrian_saver
%==========================================================================

global intersetction_type p_max ay by ax bx w Nx Ny r N M
load('variables', 'time');

%--------------------------------------------------------------------------
% delete all pedestrians that left the grid
%--------------------------------------------------------------------------
if ~isempty(deleted_pedestrians) % check if any pedestrians left the grid
    % make sure that the order of deleting elements is correct
    % (delete great numbers first)
    deleted_pedestrians = sort(deleted_pedestrians,'descend');
    for k=1:length(deleted_pedestrians)
        pedestrian_saver(deleted_pedestrians(k)) = [];
        % notice that the id is changed here -> requiers a new update of id
    end
end
p_new = length(pedestrian_saver);

%--------------------------------------------------------------------------
% add new pedestrians which enter into the grid
%--------------------------------------------------------------------------
p = 0;   % number of created pedestrians
switch(intersetction_type)
    case {1,2} % left brach (1)
    for i = ay+floor((by-ay)/2)+1 : by-floor(r/Ny)-1 
        if  (w>rand) && length(pedestrian_saver)<p_max
            % note that this function requires a lot of iterations and
            % should therfore only be used if necessairy
            if is_available(Grid, r, i, 1)==1 
            p = p+1;
            pedestrian = generate_pedestrian();
            % allocate pedestrian:
            pedestrian.gen_location = 1; % number of the branch where pedestrian is generated on
            pedestrian.velocity = pedestrian.initial_velocity*[1;0];
            pedestrian.indices = [i;1];
            pedestrian.position = [1*Nx; i*Ny];
            pedestrian.generation_time = time;
            % target indices 
            [target] = final_target(1);
            pedestrian.target_indices = [target(1,:); target(2,:)];
            pedestrian.target_position = [target(2,:).*Nx; target(1,:).*Ny];
            pedestrian_saver{p_new + p} = pedestrian; % save pedestrian behavior
            Grid(i,1) = p_new+p; % save id
            % each pedestrian has an unique id for each time step.
            end
        end
    end
end

switch(intersetction_type) 
    case {1, 2} % right branch (2)
    for i = ay+floor(r/Nx)+1 : by-floor((by-ay)/2)-1
       
        if  (w>rand) && length(pedestrian_saver)<p_max
            if is_available(Grid, r, i, M)==1
            p = p+1;
            pedestrian = generate_pedestrian();
            pedestrian.gen_location = 2;
            pedestrian.velocity = pedestrian.initial_velocity*[-1;0];
            pedestrian.indices = [i;M];
            pedestrian.position = [M*Nx; i*Ny];
            pedestrian.generation_time = time;
            [target] = final_target(2);
            pedestrian.target_indices = [target(1,:); target(2,:)];
            pedestrian.target_position = [target(2,:).*Nx; target(1,:).*Ny];
            pedestrian_saver{p_new + p} = pedestrian;
            Grid(i,1) = p_new+p;
            end
        end
    end 
end

switch(intersetction_type)
    case{2}  % upper branch (3)
    for j = ax+floor(r/Ny)+1 : bx-floor((bx-ax)/2)-1
        if  (w>rand) && length(pedestrian_saver)<p_max
            if is_available(Grid, r, 1, j)==1
            p = p+1;
            pedestrian = generate_pedestrian();
            pedestrian.gen_location = 3;
            pedestrian.velocity = pedestrian.initial_velocity*[-1;0];
            pedestrian.indices = [1;j];
            pedestrian.position = [j*Nx; Ny];
            pedestrian.generation_time = time;
            [target] = final_target(3);
            pedestrian.target_indices = [target(1,:); target(2,:)];
            pedestrian.target_position = [target(2,:).*Nx; target(1,:).*Ny];
            pedestrian_saver{p_new + p} = pedestrian;
            Grid(1,j) = p_new+p;
            end
        end
    end 
end

switch(intersetction_type)   
    case{2} % lower branch (4)
    for j = ax+floor((by-ax)/2)+1 : bx-floor(r/Ny)-1
        if  (w>rand) && length(pedestrian_saver)<p_max
            if is_available(Grid, r, N, j)==1
            p = p+1;
            pedestrian = generate_pedestrian();
            pedestrian.gen_location = 4;
            pedestrian.velocity = pedestrian.initial_velocity*[-1;0];
            pedestrian.indices = [N; j];
            pedestrian.position = [j*Nx; N*Ny];
            pedestrian.generation_time = time;
            [target] = final_target(4);
            pedestrian.target_indices = [target(1,:); target(2,:)];
            pedestrian.target_position = [target(2,:).*Nx; target(1,:).*Ny];
            pedestrian_saver{p_new + p} = pedestrian;
            Grid(N,j) = p_new+p;
            end
        end
    end 
end 


end







