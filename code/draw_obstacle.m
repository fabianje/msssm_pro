function [] = draw_obstacle()
%==========================================================================
% This function draws the whole grid including all obstacles.
%==========================================================================

global with_roundabout Nx Ny with_middle_line W_line ...
    with_inner_radius ay by with_4_roundabouts N M

% draw grid
colormap([0.8 0.8 0.8])
clf                 % clear figure
imagesc(zeros(N,M)) % display grid
axis equal % sets the aspect ratio so that the data units are the same 
% in every direction. The aspect ratio of the x-, y-, and z-axis is 
% adjusted automatically according to the range of data units  
axis([0 M,0,N]) % set axis limits
xlabel('j')
ylabel('i')


load('variables', 'bounds') % load struct profile

% draw profile
profile = bounds.profile;
patch(profile(2,:), profile(1,:), [1 1 1], 'LineWidth', 2);

% draw inner circle
if with_inner_radius
    inner_circle = bounds.inner_circle;
    R = inner_circle(1)/Nx;       % radius
    Mid = inner_circle(2:3);      % center point (indices)

    ky = (by-ay)/2;
    kx = sqrt((R)^2-ky^2);
    for i=0:3
        angles = pi/2*i+atan(ky/kx) : 0.01 : pi/2*i+atan(kx/ky);
        dR = R;
        x = dR*cos(angles);
        y = dR*sin(angles);
        patch(Mid(1)+x, Mid(2)+y, [1 1 1], 'LineWidth', 2);
    end
    angles = 0:0.01:(2*pi);
    dR = 0.99*R;
    x = dR*cos(angles);
    y = dR*sin(angles);
    patch(Mid(1)+x, Mid(2)+y, [1 1 1], 'EdgeColor', 'None');
end


% draw roundabouts
if with_roundabout || with_4_roundabouts
    outer_circle = bounds.outer_circle;
    [ns, ms] = size(outer_circle);
    for k=1:ms
        circle = outer_circle(:,k);
        R = circle(1)/Nx;       % radius
        Mid = circle(2:3);      % center point (indices)   
        angles = 0:0.01:(2*pi);
        dR = R;
        x = dR*cos(angles);
        y = dR*sin(angles);
        patch(Mid(1)+x, Mid(2)+y, [0.8 0.8 0.8], 'LineWidth', 2)
    end
end

% draw lines
if with_middle_line
    straight_lines = bounds.straight_lines;
    [nl, ml] = size(straight_lines);
    for k = 1:2:ml-1
        line = straight_lines(:,k:k+1);
        A = line(:,1);
        B = line(:,2);
        patch([A(1), B(1)],[A(2), B(2)], 'w', 'LineWidth', W_line/Ny)
    end
end







end

