function [] = draw_pedestrian(i,j,velocity, gen_location)
%==========================================================================
% This function draws all pedestrian as a circle with radius r at the
% position where the pedestrian is located. Two color schemes are available
%   >> the color depends on the velocity: red if v=0, green if v=vmax
%   >> the color depends on the position where the pedestrians where
%   generated.
%--------------------------------------------------------------------------
% Input:    > coordinate of the pedestrian (i,j)
%           > its velocity vector
%           > and the location where the pedestrian is generated
%==========================================================================

global Nx r color_set v_min v_max
v_min = 0;

hold on         % Hold the graphics

% Define the coordinates for the pedestrian
angles = 0:0.1:(2*pi);
dR = r/Nx;
x = dR*cos(angles);
y = dR*sin(angles);

if color_set == 1 % make the color depending on the generating location
    switch gen_location
        case 1
            color = [1 0 0];
        case 2
            color = [0 1 0];
        case 3
            color = [0 0 1];
        case 4
            color = [0.5 0.5 0.5];
    end

else % make the collor dependend of the velocity
    v = norm(velocity);
    scale = min(1, max(0, (v-v_min)/(v_max-v_min)));
    red     = max(0, 1 - 2*scale);      % slow
    blue    = 2*(0.5 - abs(0.5-scale)); % medium
    green   = max(2*scale - 1, 0);      % fast
    color   = [red green blue];
end

% Draw pedestrian!
patch(j+x, i+y, color, 'EdgeColor','none')
end

