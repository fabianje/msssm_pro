clear all
close all
clc
format compact






%==========================================================================
% Parameters
%==========================================================================

global p_max interaction_distance dt t_end update_type intersetction_type ...
    w target_radius Nx Ny r color_set roundabout_radius intersection_radius ...
    with_roundabout with_inner_radius with_middle_line W_line traffic_light_time ...
    with_traffic_lights traffic_light__time_closed number_of_traffic_lights_red ...
    with_4_roundabouts super_radius roundabout_4_radius


% INTERSECTION
%--------------------------------------------------------------------------
% grid resolution (distance between two grid points)
Nx = 0.05;                  % [m] in x-direction
Ny = 0.05;                  % [m] in y-direction

% Width of lines
W_line = 0.1;               % [m]

 
% SIMULATION
%--------------------------------------------------------------------------
% Two updating distinct types are available:
%   1: euler forward method: efficient
%   2: low-storage runga-kutta method: more exactly
update_type = 1;

dt          = 0.05;         % [s] integration step
t_end       = 50;          % [s] simulation time

with_graphic = true;        % if true a graphical output is obtained
with_video   = true;        % if true a video called 'pedestrian_simulation.avi is generated


% PEDESTRIANS
%--------------------------------------------------------------------------
r = 0.2;                    % "radius" of each pedestrian
w = 0.003;                  % possibility that pedestrian is generatated

% Each pedestrian is located in a cell with size interaction_distance^2.
% All pedestrian recognize neighbour in this cell and in 8 neighbour cells.
% Increase this value if just a few pedestrians are located on the grid,
% decrease this value if the pedestrian densitiy is high (improve the
% updating speed).
interaction_distance = 4;   % [m]

% pedestrians approch the target until target_radius and then they follow
% the next target. If target_radius is chosen to small, the trajectories
% become edgy and the dynamic might be lost. If target_radius is chosen to
% big the update speed decreases pereceptible.
target_radius = 3;          %[m]

% Two color sets are available:
%   1: color depends on generating location: Use this for a clear overall view
%   2: color depends on speed: Use this to determine "blocking-sources"
color_set = 2;              
                       
                            
% COMMON CONTROLL PARAMETERS
%--------------------------------------------------------------------------
% maximum number of pedestrians that are allowed on the grid
p_max = 10000;

% intersection type
%   >> 1: straight path (long and narrow): Use this to adjust the
%   parameters of the forces and to derive expressions for cu-off values
%   >> 2: intersection with 4 braches: Use this for simulation
intersetction_type = 2;
      
% applications:
%   >> with_roundabout: use a roundabout in the middle of the intersection
%   >> with_inner_radius: extend the intersection wih an outer circle
%   ("inverse roundabout")
%   >> with_middle_line: draw lines in the center of each brack to force
%   the pedestrian to walk in predfined sectors
%   >> with_traffic_lights: Use traffic lights. One traffic light of four
%   is green. Use number_of_traffic_lights_red to define how many traffic
%   lights are red at the same time (number between 1 and 3).
with_roundabout     = false;
with_4_roundabouts  = false;
with_inner_radius   = false; 
with_middle_line    = false;
with_traffic_lights = false;
roundabout_radius   = 1.4;            % [m] radius of the single reoundabout
roundabout_4_radius = 0.2;          % [m] radius of the four roundabouts
super_radius        = 2.5;            % [m] used to adjust the position of the 4 roundabouts
intersection_radius = 4;            % [m]
traffic_light_time  = 7;           % [s] change traffic lights after this time
traffic_light__time_closed = 3;     % [s] time in which all brached are closed
number_of_traffic_lights_red = 2;   % [s] number of red traffic light (1,2,3)
% Important notes:
%   (1) all features are only applicable if intersection_type is chosen
%    to be 2
%   (2) Set roundabout_radius and intersection_radius to suitable values
%   even if they are not used! Set traffic_light_time even if
%   with_traffic_lights is set to false.
%   (3) If with_traffic_lights=true make sure that with_middle_line=true is
%   set as well. Ohterwise, the pedestrians will bypass the traffic lights.
%   (4) do not use with_roundabouts and with_4_roundabouts at the same
%   time.



%==========================================================================
% Simulation
%==========================================================================
video_name = 'simulation_video_1.avi';
[measurements, velocity_distribution] = simulate(with_graphic, with_video, video_name);



%==========================================================================
% Output, Results
%==========================================================================
fprintf('\n')
fprintf('\n')
disp('measurements:')
fprintf('\n')

% the following result are obtained:

% (1) avarage velocity
v_avarage = measurements(1)

% (2) avarage of preferred velocity
v0_avarage = measurements(2)

% (3) avaraged number of pedestrians walking slower than 0.2 m/s. This value
% indicates the loss of dynamic.
low_speed_index = measurements(3) 

% (4) avaraged velocity of all pedestrian with preferred velocity >1.5. This
% value compared with v_avarage answers the question whether "stressing"
% does help or not.
high_speed_index = measurements(4)

% (5) avarage time effort to reach the final target.
avarage_time_effort = measurements(5)

% (6) number of pedestrians that have not left the grid after the end of
% the simulation
number_of_pedestrians_left = measurements(6)

% (7) velocity_distribution is a NxM grid that contains in
% velocity_distribution(i,j) the avarage velocity at the indices {i,j}
velocity_distribution;
figure
imagesc(velocity_distribution)



