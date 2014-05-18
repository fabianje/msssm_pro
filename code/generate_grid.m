function [Grid] = generate_grid(type)
%==========================================================================
% The grid is represented by a matrix Grid. To acces the matrix use
% Grid(i,j). In this simulation, two kind of position representation are
% used: (1) indices-notation {i,j} with coordinatesystem {i,j} and
% (2) position-notation {j*Nx,i*Ny} with coordinate system {x,y}. The
% two coordinat systems are connetced as follows: x->j and y->-i.
% Notice that some applications requieres a symmetric grid (M=N)!
% If a pedestrian should be able to see an obstacle, this obstacle should
% be implemented in the grid represented as inf-numbers.
%--------------------------------------------------------------------------
% Input:    > type (type of the intersection):
% output:   > Grid: two dimensional NxM grid with intersection boundaries
%==========================================================================

global ax ay bx by N M L_grid W_grid Nx Ny roundabout_radius ...
    intersection_radius with_roundabout with_inner_radius ...
    with_middle_line W_line with_traffic_lights lx ly ...
    number_of_traffic_lights_red traffic_light_time traffic_light__time_closed ...
    with_4_roundabouts super_radius roundabout_4_radius W_street

Grid = zeros(N,M); % all grid points assumed to be empty (0)

% stuct with caracteristic boundary points
%   > profile: contains border points of the profile (counter cklockwise)
%   > streigh_line: contains borer lines of objects
%   > outer_circle: roundabounds
%   > corner: single corner points; often part of the profile
%   > straight_lines: like profile lines but not linked with other lines
%   > traffic_lights: like straight_lines but with additional restrictions
profile = [];
outer_circle = [];
inner_circle = [];
corner = [];
straight_lines = [];
traffic_lights = [];
bounds = struct('profile', profile, 'outer_circle', outer_circle, 'inner_circle', ...
    inner_circle, 'corner', corner, 'straight_lines', straight_lines,...
    'traffic_lights', traffic_lights);

save('variables.mat', 'bounds')

% Check if all parameters are given. The follosing parameters are not
% neccessary for the simulation. However, sometomes an error will arise if
% they are not defined.
if isempty(roundabout_radius) || isempty(intersection_radius)
    warning('roundabout_radius or intersection_radius are not defined yet')
elseif isempty(traffic_light_time) ||isempty(traffic_light__time_closed) || isempty(number_of_traffic_lights_red)
    warning('traffic_light_time, traffic_light__time_closed or number_of_traffic_lights_red is not defined yet')
elseif with_traffic_lights && ~with_middle_line
    warning('with_middle_line is set to false. Pedestrian will be able to walk around the traffic lieghts')
end





switch type
    %----------------------------------------------------------------------
    case 1 % straight path
        % size of the grid
        L_grid = 6;        % [m]
        W_grid = 3;        % [m]
        % grid dimesions
        N = W_grid/Nx;
        M = L_grid/Ny;
        % Width of the streets
        W_street = 2.5; %[m]
        % characteristic points of the intersection
        ay = floor(0.5*N - W_street/(2*Ny));
        by = floor(0.5*N + W_street/(2*Ny));
        
        % allocate grid
        Grid(1:ay,1:M) = inf;
        Grid(by:N,1:M) = inf;
        
        % Save characterisitc points
        profile = [[by; 1],[by; M],[ay; M],[ay; 1]];
    %----------------------------------------------------------------------    
    case 2 % 4 brach intersection
        %-------------------------------
        % GENERAL IMPLEMENTATION
        %-------------------------------
        % size of the grid
        L_grid = 20;        % [m]
        W_grid = 20;        % [m]        
        % grid dimesions
        N = W_grid/Nx;
        M = L_grid/Ny;
        % Width of the streets
        W_street = 4; %[m]
        % characteristic points of the intersection
        ay = floor(0.5*N - W_street/(2*Ny));
        by = floor(0.5*N + W_street/(2*Ny));
        ax = floor(0.5*M - W_street/(2*Nx));
        bx = floor(0.5*M + W_street/(2*Nx));
        
        % allocate grid
        Grid(1:ay,1:ax) = inf;
        Grid(by:N,1:ax) = inf;
        Grid(1:ay,bx:M) = inf;
        Grid(by:N,bx:M) = inf; 
        
        if intersection_radius<=roundabout_radius
            error('roundabout <= intersection_radius')
        end
        
        % Save characterisitc points
        profile = [[by; 1],[by; ax],[N; ax],[N; bx],[by; bx],[by; M],...
            [ay; M],[ay; bx],[1; bx],[1; ax],[ay; ax],[ay; 1] ];
        corner = [[by; ax],[by;bx],[ay;bx],[ay;ax]];

        %-------------------------------
        % INVERSE ROUNDABOUT
        %-------------------------------
        if with_inner_radius
            kx = round(sqrt((intersection_radius/Nx)^2 - ((by-ay)/2)^2));
            ky = round(sqrt((intersection_radius/Ny)^2 - ((bx-ax)/2)^2));
            k1 = [by;M/2-kx];
            k2 = [N/2+ky;ax];
            k3 = [N/2+ky;bx];
            k4 = [by;M/2+kx];
            k5 = [ay;M/2+kx];
            k6 = [N/2-ky;bx];
            k7 = [N/2-ky;ax];
            k8 = [ay;M/2-kx];
            
            profile = [[by; 1],k1,k2,[N; ax],[N; bx],k3,k4,[by; M],[ay; M],k5,...
                k6,[1; bx],[1; ax],k7,k8,[ay; 1] ];
            corner = [k1, k2, k3, k4, k5, k6, k7, k8];
            
            for i = -round(intersection_radius/Ny) : round(intersection_radius/Ny)
                for j = -round(intersection_radius/Nx) : round(intersection_radius/Nx)
                    % inner circle
                    if (i*Ny)^2+(j*Nx)^2 <= intersection_radius^2 && (i*Ny)^2+(j*Nx)^2 >= roundabout_radius^2
                        Grid(N/2+i, M/2+j) = 0;
                    end  
                end
            end 
            inner_circle = [intersection_radius; N/2; M/2];
        end 
        
        
        %-------------------------------
        % ROUNDABOUT
        %-------------------------------
        if with_roundabout            
            for i = -round(roundabout_radius/Ny) : round(roundabout_radius/Ny)
                for j = -round(roundabout_radius/Nx) : round(roundabout_radius/Nx)
                    % outer circle (roundabout)
                    if (i*Ny)^2+(j*Nx)^2 < roundabout_radius^2
                        if Grid(N/2+i, M/2+j) == inf
                            error('roundabout radius is too big')
                        end
                        Grid(N/2+i, M/2+j) = inf;  
                    end
                end
            end           
            outer_circle = [roundabout_radius; N/2; M/2]; 
        end
        
        %-------------------------------
        % 4 ROUNDABOUTS
        %-------------------------------        
        if with_4_roundabouts
            kappa = super_radius/Nx;
            
            for mid_i = N/2-kappa : kappa : N/2+kappa
                for mid_j = M/2-kappa : kappa : M/2+kappa
                    if mid_i==mid_j || (mid_i==N/2-kappa && mid_j==M/2+kappa) || (mid_i==N/2+kappa && mid_j==M/2-kappa)
                        continue
                    end
                    for i = -round(roundabout_4_radius/Ny) : round(roundabout_4_radius/Ny)
                        for j = -round(roundabout_4_radius/Nx) : round(roundabout_4_radius/Nx)
                            % outer circle (roundabout)
                            if (i*Ny)^2+(j*Nx)^2 < roundabout_4_radius^2
                                if Grid(mid_i+i, mid_j+j) == inf
                                    error('roundabout radius is too big')
                                end
                                Grid(mid_i+i, mid_j+j) = inf;  
                            end
                        end
                    end 
                end
            end
            
            circle1 = [roundabout_4_radius; N/2; N/2+kappa];
            circle2 = [roundabout_4_radius; N/2; N/2-kappa];
            circle3 = [roundabout_4_radius; N/2+kappa; N/2];
            circle4 = [roundabout_4_radius; N/2-kappa; N/2];
            outer_circle = [outer_circle, circle1, circle2, circle3, circle4]; 
        end
        
       

        %-------------------------------
        % MIDDLE LINES
        %-------------------------------
        if with_middle_line
            wx = W_line/Nx;
            wy = W_line/Ny;
            if with_inner_radius % length of a line
                lx = round((ax-kx+(bx-ax)/2)*0.9);
                ly = round((ay-ky+(by-ay)/2)*0.9);
            else
                lx = round(ax*0.8);
                ly = round(ay*0.8);
            end

            Grid(round(N/2-wx/2):round(N/2+wx/2),1:lx) = inf; 
            Grid(round(N/2-wx/2):round(N/2+wx/2),M-lx:M) = inf;
            Grid(1:ly,round(M/2-wx/2):round(M/2+wx/2)) = inf; 
            Grid(N-ly:N,round(M/2-wx/2):round(M/2+wx/2)) = inf;
            
            % lines pointing from center of the intersection to the outside
            line_1 = [[N/2;lx], [N/2;1]];
            line_2 = [[N-ly;M/2],[N;M/2]];
            line_3 = [[N/2;M-lx],[N/2;M]];
            line_4 = [[ly;M/2],[1;M/2]];
            
            straight_lines = zeros(2,8);
            straight_lines(:,1:2) = line_1;
            straight_lines(:,3:4) = line_2;
            straight_lines(:,5:6) = line_3;
            straight_lines(:,7:8) = line_4;
            
            % interprete the beginning of each line as a corner boundary
            corner = [corner, [line_1(:,1),line_1(:,1),line_1(:,1),line_1(:,1)]];
        end

        %-------------------------------
        % TRAFFIC LIGHTS
        %-------------------------------
        % A traffic light is in this simulater represented as a line
        % interpreted as both, traffic light and profile. It forces
        % the pedestrian to set v=0 or to prevent the pedestrians to walk trough. 
        % During the simulation traffic_lights will be changed.
        if with_traffic_lights
            traffic_light_1 = [[by;lx],[N/2;lx]];       % brach 1
            traffic_light_4 = [[N-ly;bx],[N-ly; M/2]];  % brach 4
            traffic_light_2 = [[ay;M-lx],[N/2;M-lx]];   % brach 2
            traffic_light_3 = [[ly;ax],[ly; M/2]];      % brach 3
            lights = [traffic_light_1, traffic_light_2, traffic_light_3, traffic_light_4];
            save('variables.mat', 'lights', '-append')           
            
            switch(number_of_traffic_lights_red)
                case 3
                    traffic_lights = [traffic_light_1, traffic_light_2, traffic_light_3];
                    branch_number = [1, 2, 3]; % traffic light appears at branch 1 2 and 3
                case 2
                    traffic_lights = [traffic_light_1, traffic_light_2];
                    branch_number = [1, 2]; % traffic light appears at branch 1 2
                case 1
                    traffic_lights = [traffic_light_1];
                    branch_number = [1]; % traffic light appears at branch 1
                otherwise
                    error('number_of_traffic_lights_red is greater than 3')
            end
            save('variables.mat', 'branch_number', '-append')
    
            time_where_branch_is_closed = 0; % helper
            save('variables.mat', 'time_where_branch_is_closed', '-append')
            
        end
        
        
    %----------------------------------------------------------------------   
end

bounds.profile = profile;
bounds.corner  = corner;
bounds.inner_circle = inner_circle;
bounds.outer_circle = outer_circle;
bounds.straight_lines = straight_lines;
bounds.traffic_lights = traffic_lights;
save('variables.mat', 'bounds', '-append') % save struct for later access

end





% Notice that there a 2 possibilities to detect a boundary point:
%   1) numerically: Iterate over Grid in a small range around the current
%   pedestroan and locate all grid points. This use of this method is not
%   recommanded, since ...
%       - the numercial effort increases due to aditional iterations
%       - the acuracy depend in the parameter Nx and Nx and those values
%       can not be decreased more then 0.05 (numerical effort)
%   2) analytically: Save all characterstic points in a struct and
%   derive for each pedestrian the nearest grid point. This method ...
%       + is very efficient
%       - but requieres a lot of additional implementations.
% We decided for the second method after we failed with the first one
% due to not sufficiently high accuracy.


