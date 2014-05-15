function [x1, v1] = evade_pedestrian(x1,x1_old, x2, v1, v2)
%==========================================================================
% evade_pedestrian() checks wheter the updating position is available or if
% there is an other pedestrian that prevents from update. If the update is
% not possible, new position and velocity vectores will be created based on
% the filter-principle.
%--------------------------------------------------------------------------
% > Input:   > x1 (upodating position),x1_old (current position), x2 , 
%              v1 (velocity), v2
% > Output:  > corrected position x and corrected velocity v
%==========================================================================


global dt lx ly N M Nx Ny with_traffic_lights

[n,m] = size(x2);


for k=1:m
    x = x2(:,k)-x1;
    cos_alpha = x'*v1/(norm(x)*norm(v1));
    if cos_alpha<0
        continue
    end
    
    % special case if neighbour remains on his position. There will be a
    % reson for total deaccelaration and it will be best to do the same.
    if with_traffic_lights % only consider traffic light situations
        x1_indices = round([x1(2)/Ny; x1(1)/Nx]);
        if norm(v2)==0 && (x1_indices(1)<ly || x1_indices(1)>N-ly || x1_indices(2)<lx || x1_indices(2)>M-lx)
            v1 = [0;0]; % set velocity to zero
            x1 = x1_old; % do not update
            return
        end
    end

    % perform a coordinate transformation (x -> new x-axis):
    cos_gamma = [1 0]*x/(norm(x)); % angle between x-axis and v1
    sin_gamma = (1-cos_gamma^2)^(1/2);

    if x(2)>0
        T = [cos_gamma sin_gamma; -sin_gamma cos_gamma]; % rotation in positive direction
    else
    	T = [cos_gamma -sin_gamma; sin_gamma cos_gamma]; % rotation in negative direction
    end

    X = T*x;
    if X(1)<0
        T(1,:) = -T(1,:);
    end

    % transformation of some date
    X = T*x;
    V1 = T*v1;

    if X(2)>10^-5 || X(1)<0 % check if transformation was successful
        warning('Coordinate transformation failed')
    end

    V1(1)=0;
    v1 = T^-1*V1;
    x1 = x1+dt*v1;
end


end

