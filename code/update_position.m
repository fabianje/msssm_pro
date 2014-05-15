function [x, v] = update_position(f, x, v, m)
%==========================================================================
% This function computs the position and velocity using one of two updating
% types
%--------------------------------------------------------------------------
% Input:    > force f
%           > x (position), v (velocity), m (mass)
% Output:   > updated position x and new velocity v
%==========================================================================
global dt update_type

if update_type ==1 % euler forward
    a = f/m;        % compute acceleration
    x = x+dt*v;     % update position
    v = v+dt*a;     % update velocity
        
elseif update_type == 2 % low-storage runga-kutta
    a=f/m;
    c = [0 -17/32 -32/27]';
    b = [1/4 8/9 3/4]';
    q1 = zeros(2,1);
    q2 = zeros(2,1);
    
    for i=1:2
        q1 = c(i)*q1+dt*v;
        q2 = c(i)*q2+dt*a;
        x = x + b(i)*q1;
        v = v + b(i)*q2;
    end   
    
end

end

