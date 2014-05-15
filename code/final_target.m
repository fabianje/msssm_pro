function [target] = final_target(gen_location)
%==========================================================================
% All pedestrian follow a manual generated (preferred) trajectory.
% This trajectroy consits of several mid_points and exactely one final
% point. The the target points should be chosen such that the resulting
% trajectory minimizes the way the pedestrian have to walk on. Notice that
% is is not necessary to implements several points to obtain a smooth. A
% great number of target points causes predictable peestrian movements and
% lead to a loss of dynamic.
% The definition of the trajectory depends on the application that are
% chosen:
%   >> no applications: the pedestrians approach the center and skip as far
%   as possible to the final target.
%   >> with_roundabout=true: all pedestrians walk on counter clock wise
%   around the roundabout. The try to walk as near as posible to the
%   roundabout.
%   >> with_inner_radius=true: no infuence on the preferred trajectory.
%   >> with_middle_line=true: preffered trajectory is restricted inside the
%   branches.
%   >> with_4_roundabouts=true: If with_roundabout=false, than the
%   the pedestrian walk on trajectories like in the case where no
%   applications are chosen. The only difference is that the roundabouts
%   have a separating impact on the whole dynamik. If with_roundabout=true
%   the trajectory is the same as if only the roundabout is used. The four
%   roundabouts act like a "pre-separater".
%--------------------------------------------------------------------------
%Input:     > gen_location: number of the branch where pedestrian is
%           generated on
%Output:    > indices of the final target
%==========================================================================

global N M Nx Ny intersetction_type ay by ax bx roundabout_radius ...
    with_roundabout with_middle_line with_inner_radius W_street ...
    intersection_radius with_4_roundabouts super_radius with_traffic_lights

% uniform distribution
uniform=@(a, b)(a + (b-a)*rand);

%--------------------------------------------------------------------------
if intersetction_type==1
   final_target = mod(gen_location,2)+1; 

   switch(final_target)
        case 1 % left branch
            final_point = [N/2; -M*0.5]; % overshooting!
        case 2 % right branch
            final_point = [N/2; M*1.5];
   end
   mid_points = [];
   
%--------------------------------------------------------------------------
elseif intersetction_type==2 && ~with_roundabout && ~with_4_roundabouts
    
    % Introduce additional space for mid_points (do not walk into
    % boundary points!!). To make sure that not all pedestrian tries to reach
    % the same mid points, let this factor be varying
    security_x = round(uniform(0.7, W_street*0.6)/Nx);
    security_y = round(uniform(0.7, W_street*0.6)/Ny);

    
    % Randomize the final target
    final_target=gen_location;
    while final_target==gen_location % do not walk back to initial point
        final_target = round(uniform(1,4)); % random number in the range (1,4)
    end
    
    switch(final_target)
        case 1 % left branch
            final_point = [N/2; -M*0.5];
            switch(gen_location)
                case 2 
                    mid_points = [N/2;M/2]; % this point is necessary if the pedestrian is pushed aside
                case 3 
                    mid_points = [ay;ax] + [security_y; security_x];
                case 4
                    mid_points = [by; ax] + [-security_y; security_x];
            end
        case 2 % right branch
            final_point = [N/2; M*1.5];
            switch(gen_location)
                case 1
                    mid_points = [N/2;M/2];
                case 3
                    mid_points = [ay; bx] + [security_y; -security_x];
                case 4
                    mid_points = [by; bx] + [-security_y; -security_x];
            end
        case 3 % upper branch
            final_point = [-N*0.5; M/2];
            switch(gen_location)
                case 1
                    mid_points = [ay; ax] + [security_y; security_x];
                case 2
                    mid_points = [ay; bx] + [security_y; -security_x];
                case 4
                    mid_points = [N/2;M/2];
            end
        case 4 % lower branch
            final_point = [N*1.5; M/2];
            switch(gen_location)
                case 1
                    mid_points = [by; ax] + [-security_y; security_x];
                case 2
                    mid_points = [by; bx] + [-security_y; -security_x];
                case 3
                    mid_points = [N/2;M/2];
            end
    end
    
%--------------------------------------------------------------------------
elseif intersetction_type==2 && with_roundabout
  
    final_target=gen_location;
    while final_target==gen_location
        final_target = round(uniform(1,4));
    end

    % radius the pedestrian try to wolk on
    if with_inner_radius
        R = uniform(roundabout_radius*1.1,intersection_radius*0.9);
    else
        R = uniform(roundabout_radius*1.1, sqrt((Nx*(bx-ax)/2)^2+(Ny*(by-ay)/2)^2)*0.9 );
    end
    
    % two middle points include an angle of pi/8 (-> 16 middle points)
    varphi = 0:pi/8:2*pi;
    % here are the x- and y-components of the middle points
    mid_x = R*cos(varphi);
    mid_y = R*sin(varphi);
    mid_points = [mid_x; mid_y]; % non-indicies notation
    
    if with_inner_radius
        r_max = intersection_radius; % take greates radius that is possible
    else
        r_max = R/0.9;
    end
    
    switch(final_target)
        case 1 % left branch
            final_point = [N/2; -M*0.5];
            switch(gen_location)
                case 2 
                    mid_points = mid_points(:,1:9);
                case 3 
                    mid_points = mid_points(:,5:9)/R*r_max*0.9;
                case 4
                    mid_points = [mid_points(:,13:16), mid_points(:,1:9)];
            end
        case 2 % right branch
            final_point = [N/2; M*1.5];
            switch(gen_location)
                case 1
                    mid_points = [mid_points(:,9:16), mid_points(:,1)];
                case 3
                    mid_points = [mid_points(:,5:16), mid_points(:,1)];
                case 4
                    mid_points = [mid_points(:,13:16), mid_points(:,1)]/R*r_max*0.9;
            end
        case 3 % upper branch
            final_point = [-N*0.5; M/2];
            switch(gen_location)
                case 1
                    mid_points = [mid_points(:,9:16), mid_points(:,1:5)];
                case 2
                    mid_points = mid_points(:,1:5)/R*r_max*0.9;
                case 4
                    mid_points = [mid_points(:,13:16), mid_points(:,1:5)];
            end
        case 4 % lower branch
            final_point = [N*1.5; M/2];
            switch(gen_location)
                case 1
                    mid_points = mid_points(:,9:13)/R*r_max*0.9;
                case 2
                    mid_points = mid_points(:,1:13);
                case 3
                    mid_points = mid_points(:,5:13);
            end
    end 
    % transform mid_points to indices notation
    mid_points = round([-mid_points(2,:)/Ny+N/2; mid_points(1,:)/Nx+M/2]);   
%--------------------------------------------------------------------------
elseif intersetction_type==2 && ~with_roundabout && with_4_roundabouts
    
    final_target=gen_location;
    while final_target==gen_location
        final_target = round(uniform(1,4));
    end
    
    kx = round((bx-ax)/4);
    ky = (round(by-ay)/4);
    px = round((super_radius+roundabout_radius)*uniform(1.15,1.6)/Nx);
    py = round((super_radius+roundabout_radius)*uniform(1.15,1.6)/Ny);
    
    switch(final_target)
        case 1 % left branch
            final_point = [N/2; -M*0.5];
            switch(gen_location)
                case 2 
                    mid_points = [[N/2-ky;M/2+px],[N/2-ky;M/2],[N/2-ky;M/2-px]];
                case 3 
                    mid_points = [[N/2-py;M/2-kx],[ay+ky;ax+kx],[N/2-ky;M/2-px]];
                case 4
                    mid_points = [[N/2+py;M/2-kx],[by-ky;ax+kx],[N/2+ky;M/2-px]];
            end
        case 2 % right branch
            final_point = [N/2; M*1.5];
            switch(gen_location)
                case 1
                    mid_points = [[N/2+ky;M/2-px],[N/2+ky;M/2],[N/2+ky;M/2+px]];
                case 3
                    mid_points = [[N/2-py;M/2+kx],[ay+ky;bx-kx],[N/2-ky;M/2+px]];
                case 4
                    mid_points = [[N/2+py;M/2+kx],[by-ky;bx-kx],[N/2+ky;M/2+px]];
            end
        case 3 % upper branch
            final_point = [-N*0.5; M/2];
            switch(gen_location)
                case 1
                    mid_points = [[N/2-ky;M/2-py],[ay+ky;ay+kx],[N/2-py;M/2-kx]];
                case 2
                    mid_points = [[N/2-ky;M/2+py],[ay+ky;ay-kx],[N/2-py;M/2+kx]];
                case 4
                    mid_points = [[N/2+py;M/2+kx],[N/2;M/2+kx],[N/2-py;M/2+kx]];
            end
        case 4 % lower branch
            final_point = [N*1.5; M/2];
            switch(gen_location)
                case 1
                    mid_points = [[N/2+ky;M/2-px],[by-ky;ax+kx],[N/2+py;M/2-kx]];
                case 2
                    mid_points = [[N/2+ky;M/2+px],[by-ky;bx-kx],[N/2+py;M/2+kx]];
                case 3
                    mid_points = [[N/2-py;M/2+kx],[N/2;M/2+kx],[N/2+py;M/2+kx]];
            end
    end
    
%--------------------------------------------------------------------------
end
%--------------------------------------------------------------------------

% replace last element mid_point and adjust the last target point
if with_middle_line || with_4_roundabouts
    dj = round((bx-ax)/4);
    di = round((by-ay)/4);
    
    if with_inner_radius
       Ay = N/2-intersection_radius/Ny;
       By = N/2+intersection_radius/Ny;
       Ax = M/2-intersection_radius/Nx;
       Bx = M/2+intersection_radius/Nx;
    else
        Ay = ay;
        By = by;
        Ax = ax;
        Bx = bx;
    end
    
    switch(final_target)
        case 3
            last_mid_point = [Ay; M/2];
        	Delta = [0;dj];
        case 4
            last_mid_point = [By; M/2];
            Delta = [0;-dj];
        case 1
            last_mid_point = [N/2; Ax];
            Delta = [-di;0];
        case 2
            last_mid_point = [N/2; Bx];
            Delta = [di;0];
    end
    
    if isempty(mid_points) || length(mid_points)==1
        mid_points = last_mid_point;
    else
        if with_roundabout
            mid_points(:,end) = [];
        else
            last_mid_point = last_mid_point + Delta;
            mid_points(:,end) = last_mid_point;
        end
    end
    final_target = final_target + Delta;

end

% if traffic light are used distribute the pedestrians in front of the
% traffic light over the entire space that is available. THe following code
% will ensure this purpose:
if with_traffic_lights
    dj = round((bx-ax)/4);
    di = round((by-ay)/4);
    
    if with_inner_radius
       Ay = N/2-intersection_radius/Ny;
       By = N/2+intersection_radius/Ny;
       Ax = M/2-intersection_radius/Nx;
       Bx = M/2+intersection_radius/Nx;
    else
        Ay = ay;
        By = by;
        Ax = ax;
        Bx = bx;
    end   
    
    switch(gen_location)
        case 1
            first_mid_point = [N/2; Ax];
        	Delta = [di;0];
        case 2
            first_mid_point = [N/2; Bx];
            Delta = [-di;0];
        case 3
            first_mid_point = [Ay;M/2];
            Delta = [0;-dj];
        case 4
            first_mid_point = [By;M/2];
            Delta = [0;dj];
    end
    first_mid_point = first_mid_point+Delta;
    mid_points = [first_mid_point, mid_points];
end



if isempty(mid_points) % no mid points
    target = final_point;
else
    target = [mid_points, final_point];
end

end

