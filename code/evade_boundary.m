function [x, v] = evade_boundary(dist_critical, distance, x_old, x_new, v)
%==========================================================================
% evade_boundary() checks wheter the updating position is available or if
% there is a boundary point that prevents from update. If the update is not
% possible, new position and velocity vectores will be created based on
% the filter-principle ("set the component to zero that is orthogonal to
% the boundary line").
%--------------------------------------------------------------------------
% > Input:   > dist_critical: cell array with dist_crit (vector pointing
%            from current center pedestrian to nearest boundary point)
%            and boundary type (string with type of the boundary point)
%            > distance: vector pointing from current pedestrian to
%            preferable updating position
%            > x_old: current position
%            > x_new: preferable updating position
%            > v: preferable velocity vector
% > Output:  > corrected position x and corrected velocity v
%==========================================================================

global r dt W_line

% extract infomrations
boundary_type = dist_critical{1};
dist_crit = dist_critical{2};

x=x_new;        % assumption
security = 1.3; % security factor

    
% exclude all cases, where no touching of boundary points expecte
cos_alpha = dist_crit'*distance/(norm(dist_crit)*norm(distance));
if (strcmp(boundary_type,'profile') || strcmp(boundary_type,'outer_circle')) && (cos_alpha*norm(distance) < norm(dist_crit)-security*r || cos_alpha<=0)
    return % leave this function
elseif strcmp(boundary_type,'corner') && norm(dist_crit)>0.9*r
    % do not use any security factor here. This will ensure that
    % pedestrians no not remain on corners when they are blocked.
    return
elseif strcmp(boundary_type,'straight_lines') && (cos_alpha*norm(distance) < norm(dist_crit)-(security*r+W_line/2) || cos_alpha<=0)
    % the lines have a width and have therfore a 3dim charater!
    return
end

% perform a coordinate transformation (dist_critical -> new x-axis):
cos_gamma = [1 0]*dist_crit/(norm(dist_crit)); % angle between dist_crit and x-axis
sin_gamma = sqrt(1-cos_gamma^2);

if dist_crit(2)>0
    M = [cos_gamma sin_gamma; -sin_gamma cos_gamma]; % rotation in positive direction
else
    M = [cos_gamma -sin_gamma; sin_gamma cos_gamma]; % rotation in negative direction
end

% transformation of some date
V = M*v;
DIST_critical = M*dist_crit;

if DIST_critical(2)>10^-3 % check if transformation was successful
    disp('(evade_boundary) Attention: Coordinate transformation faild')
end

if strcmp(boundary_type,'profile') || strcmp(boundary_type,'corner') || strcmp(boundary_type,'outer_circle') || strcmp(boundary_type,'straight_lines')
    V(1) = 0;    % filter: let only pass the y-component of the velocity
    v = M\V;     % inverse transformation
    x=x_old+dt*v;% Calculate new position (Euler)
end

     

end


