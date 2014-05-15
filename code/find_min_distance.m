function [distance] = find_min_distance(A,B,C,distance,with_transformation)
%==========================================================================
% This function find the vector with minumum distance pointing from
% a point located on AB to the point C. This function works with indices
% notation only!!
%--------------------------------------------------------------------------
% Input:    > Two points A,B that define the line
%           > point C
%           > with_transfomration: true if transformation is necessary.
%           That is if the line AB should be unpassable for both sides.
%           Notice that the orientation of AB does matter!!
%           > distance: starting vector (for minimization)
% Output:   > distance (vector)
%==========================================================================

global N M

expected_intersection = true;

a = norm(B-C);        
b = norm(A-C);        
c = norm(B-A);
        
cos_beta = (C-B)'*(A-B)/(a*c);        
cos_alpha = (C-A)'*(B-A)/(b*c);        
sin_alpha = sqrt(1-cos_alpha^2);
        
if cos_beta<0 || cos_alpha<0 || ((A(1)==1 && B(1)==1) || (A(1)==N && B(1)==N) || (A(2)==1 && B(2)==1) || (A(2)==M && B(2)==M))           
    expected_intersection = false;       
end

dist = sin_alpha*b;
        
if dist < norm(distance) && expected_intersection   
    e_AB = (B-A)/norm(B-A);     
    distance = dist;

    if with_transformation
 
        % perform a coordinate transformation (AB -> new j-axis):
        cos_gamma = [0 1]*e_AB; % angle between x-axis and v1
        sin_gamma = (1-cos_gamma^2)^(1/2);
         
        if e_AB(1)>0                
            T = [cos_gamma sin_gamma; -sin_gamma cos_gamma]; % rotation in positive direction            
        elseif e_AB(1)<0                
            T = [cos_gamma -sin_gamma; sin_gamma cos_gamma]; % rotation in negative direction            
        else % e_AB(1)=0                
            T = eye(2,2); % no rotation;            
        end 
    
        % Adjust the sign of the axis            
        E_AB = T*e_AB;            
        if E_AB(2)<0                
            T = -T;            
        end
     
        % check if transformation was successful
        E_AB = T*e_AB;            
        if E_AB(1)>10^-5 || E_AB(2)<0  
            warning('Attention: Coordinate transformation faild')            
        end
       
        % Derive an expression for the direction           
        check = T*C-T*A;         
        if check(1)<0            
            e = [-e_AB(2); e_AB(1)]; % rotation with -pi/2          
        else      
            e = [e_AB(2); -e_AB(1)]; % rotation with +pi/2          
        end  
        distance = e*distance; % vector pointing from boundary to pedestrian
            
    else
        e = [-e_AB(2); e_AB(1)]; % rotation with +pi/2
        distance = e*distance; % vector pointing from boundary to pedestrian
    end 
    
end

end
