function [measurements, velocity_distribution] = simulate(with_graphic, with_video, video_name)
%==========================================================================
% simulate() calls all function for each iteration step.
% Input:    > with_graphic (1, 0)
%           > with_video (1, 0)
%           > video_name (1, 0)
% Output:   > measurements
%           > velocity_distribution
%==========================================================================

global dt t_end intersetction_type N M

iter = t_end/dt;        % number of iterations
Grid = generate_grid(intersetction_type); % prepare grid
p=0;                    % number of created pedestrians
pedestrian_saver = [];
measurements = zeros(iter, 5);
velocity_distribution = zeros(N,M);

if with_video
    hFig = figure;
    set(hFig, 'Position', [500 60 740 740])
else
    figure
end


disp('========================================')
disp('Start simulation')
disp('----------------------------------------')

fprintf('\n')
disp(['progress: ',num2str(0), 's of ', num2str(t_end), 's' ]) 


for it=1:iter
    time = it*dt;
    save('variables.mat', 'time','-append')
    [Grid, pedestrian_saver, deleted_pedestrians, m1, m2, m3, time_effort] = update(Grid, pedestrian_saver, p);
    % update measurements
    measurements(it,1:3) = m1;
    measurements(it,4) = m2;
    measurements(it,5) = time_effort;
    velocity_distribution = velocity_distribution + m3;
    
    if with_graphic || with_video % with graphical output
        
        draw_obstacle()
   
        % check if pedestrians left the grid and allocate new pedestrians
        [pedestrian_saver, Grid] = update_pedestrians(pedestrian_saver, deleted_pedestrians, Grid);
        p = length(pedestrian_saver);
   
        for k=1:p % draw all pedestrians
            current =  pedestrian_saver{k};
            draw_pedestrian(current.indices(1), current.indices(2), current.velocity, current.gen_location)
        end

        if with_video
            % store video frames (may rise a warning in MATLAB 2011a)
            A(it)=getframe(gcf);
        end
        
        pause(dt)
        
    else % without graphical output
        [pedestrian_saver, Grid] = update_pedestrians(pedestrian_saver, deleted_pedestrians, Grid);
        p = length(pedestrian_saver);
    end
    
    % Show progress after each 100 timesteps
    if mod(it, 100) == 0
       disp(['progress: ',num2str(time), 's of ', num2str(t_end), 's' ]) 
    end
   
end

if with_video
    fprintf('\n')
    disp('Save vidoe frame to file. This may take some minutes...')
    movie2avi(A, video_name,'compression','None', 'fps',7, 'quality', 100);
    fprintf('\n')
    disp('Video sucessfully generated.')
end


% take the avarge of all measurements
fprintf('\n')
disp('Compute measurements')

% measuremetns 2
m2 = measurements(:,4);
m2_new = [];
for i=1:length(m2)
    if m2(i)~=inf % find non marked values
        m2_new = [m2_new m2(i)];
    end
end
m2 = sum(m2_new)/length(m2_new); % avarage the values

% time effort
time_effort = measurements(:,5);
number_of_zeros = sum(time_effort==0); % exclude zero-marked values
time_effort = sum(time_effort)/(iter-number_of_zeros);

% measurements 1
m1 = measurements(:,1:3);
m1_new = [];
for i=1:size(measurements,1)
    if m1(i,1)~=inf % find non marked values
        m1_new = [m1_new; m1(i,:)];
    end
end
m1 = sum(m1_new)/size(m1_new,1); % avarage the values

% save all measurements
measurements = [m1'; m2; time_effort; p]; % reallocation

% velocity distribution
velocity_distribution = velocity_distribution/iter; % avarage distribution


fprintf('\n')
disp('Measurements succesfully stored')

fprintf('\n')
disp('----------------------------------------')
disp('Simulation is done')
disp('========================================')


end

