function [pedestrian] = generate_pedestrian()
%==========================================================================
% Each pedestrian is represented as a circle with radius R. The pedestrian
% tries to approach the final target as follows (in order of imporatance)
%   (1) reach current target point: That is a point on the preferred
%   trajectory that a pedestrian would like to walk on. All target point
%   are set manually. The pedestrian can choice his final point, but
%   the preferred trajectoriy is fixed (minimization of the distance) for 
%   all times after he has chosen (he can not recognize wheter this
%   trajectory lead to a minimized time effort). However, the pedestrian 
%   does not have to walk on this trajectory.
%   (2) evade other pedestrians: Pedestrian can see each ohter. They can
%   communicate with each ohter to find an optimal solution that does not
%   lead to a collision.
%   (3) evade boundary points: pedestrians can receive all obstacle. They
%   do not try to evade them actively (this is the task of how setting the
%   target points on the correct position) but the also can not pass
%   trhough them.  A objecle that is crashed acts like a filter that
%   cancles the velocity component perpendicular to the tangent. Traffic
%   lights are interpreted as boundary points.
%   (4) reach final target: The final target is the beginning of a branch.
%   Each pedestrian can decide where this final target is (randomization)
%--------------------------------------------------------------------------
% Output:   struct that contains
%           > direc: current direction
%           > v: current velocity (vector)
%           > indices: indices i,j  on the grid
%           > position: x,y components of the position
%           > mass
%           > v_boundaries: contains maximum and minimum velocity
%           > cell_dim: contains dimensions of the reaction cell
%           the reaction cell is a square-cell around the pedestrian; its
%           "interaction area" contains the eight neighbour cell around the
%           that cell
%           > target_indices: array that contains the intices (i,j) of the
%           coodinates of target
%           > target_position: array that contains the position (x,y) of
%           the coodinates of target
%           > location: actual target that the pedestrian has focused
%           > gen_location: location where the pedestrian is generated
%           > force: motivation to change the current direction
%           > is_updated: 1 pedestrian is already updated, 0 if not
%           > density: indicatator for density
%           > dist_critical, traffic_lights: see pedestrian_boundary_distance
%==========================================================================


global v_min v_max M N Ncell Mcell Nx Ny interaction_distance

v_min = 0;          % [m/s]
v_max = 4;          % [m/s]

% initialize random behavior (uniform distribution)
uniform=@(a, b) a + (b-a)*rand;

v0 = uniform(1.2,1.6); % Wikipedia: humans tend to walk at about 1.4 m/s
vmax = v0*2.5;
vmin = v0*0.3;

% subdivide N in and M in cells with zize dist*dist m^2
Ncell = ceil(N/(interaction_distance/Ny));
Mcell = ceil(M/(interaction_distance/Nx));


% initialize pedestrian
v               = [0;0];
v_boundaries    = [vmin vmax];
indices         = zeros(2,1);
x               = zeros(2,1);
mass            = uniform(50,90); % max varies between 50kg and 90kg
cell_dim        = [Ncell Mcell];  % assumed to be equal for all pedestrians
target_indices  = 0;              % will be a matrix after initialization
target_position = 0;              % will be a matrix after initialization
location        = 1;
gen_location    = 0;
force           = [0;0];
density         = 0;
dist_critical   = [];
traffic_lights  = [];
generation_time   = 0;

% create pedestrian
pedestrian = struct('velocity', v, 'initial_velocity', v0, 'mass', mass, ...
    'indices', indices, 'position', x, 'v_boundaries', v_boundaries, ...
    'cell_dim', cell_dim, 'target_indices', target_indices, 'targe_position', ...
    target_position , 'location', location, 'gen_location', gen_location, 'force', force,...
    'density', density, 'dist_critical', dist_critical, ...
    'traffic_lights', traffic_lights, 'generation_time', generation_time);

end


