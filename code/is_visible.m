function [is_visible] = is_visible(i, j , i_neighbour, j_neighbour, Grid)
%==========================================================================
% This function checks wehter the neighbour (or any other point) is visible
% for the current pedestrian or not.
% ATTENTION: This function slightly deaccelerates the updating speed!
%--------------------------------------------------------------------------
% Output:   > is_visible (true: neighbour is visible, false: neighbour is
%           not visible)
%==========================================================================

global N M
is_visible = true; % assume neighbour is visib


% calculate unit vector pointing from current to neighbour
e = [j_neighbour; i_neighbour] - [j; i];
% iterate from current pedestrian along vec until neighbour is reached and
% check wheter boundary points were crossed or not
if (e(1)>= 0 && e(2) >= 0) || (e(1)>= 0 && e(2) < 0) % x>=0
    cos_alpha = [1; 0]'*e/norm(e); % compute angle between x-axes and vec
    for y = 0 : abs(i_neighbour-(i)) % iterate over y-axis 
        if y==0
            y = sign(i_neighbour-(i)); % skip fist iteration
        end
        % l = y/tan(alpha) = y/sin(alpha)*cos(alpha)
        l = abs(y) / sqrt(1-cos_alpha^2) * cos_alpha; % iterate along x-axis
        x = round(l);
        % Before using the vector y, adjust its sign
        y = sign(i_neighbour-(i))*y;
        if i+y<1 || i+y>N || j+x<1 || j+x>M || isnan(x)
            continue % outside of the grid; no boundary point was found
        elseif i+y==i_neighbour || j+x==j_neighbour 
            return % neighbour is reached; no bloundary point was found
        elseif Grid(i+y, j+x) == inf
            is_visible = false; % boundary point was found
            return           % abort the function
        end
        % make y ready for the next itearation
        y = abs(y);
    end
    
elseif (e(1)< 0 && e(2) >= 0) || (e(1)< 0 && e(2) < 0) % x<0
    cos_alpha = [-1; 0]'*e/norm(e);
    for y = 0 : abs(i_neighbour-(i));
        if y==0
            y = sign(i_neighbour-(i)); % skip fist iteration
        end
        l = abs(y) / sqrt(1-cos_alpha^2) * cos_alpha;
        x = -round(l);
        y = sign(i_neighbour-(i))*y;
        if i+y==i_neighbour || j+x==j_neighbour 
            return
        elseif i+y<1 || i+y>N || j+x<1 || j+x>M || isnan(x)
            continue
        elseif Grid(i+y, j+x) == inf
            is_visible = false;
            return 
        end
        y = abs(y);
    end
            
elseif e(2)==0 % special case
    for x = 0: abs(j_neighbour-(j))
        if x==0
            x = sign(j_neighbour-(j));
        end
        x = sign(j_neighbour-(j))*x;
        if j+x==j_neighbour 
            return
        elseif i<1 || i>N || j+x<1 || j+x>M
            continue
        elseif Grid(i+y, j+iter_j+x) == inf
            is_visible = false;
            return 
        end 
        x = abs(x);
    end
end

end
