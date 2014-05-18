function [f] = force_comfortable_zone(v1, v2, r12, fac)
%==========================================================================
% This force tries to separate pedestrians from each other ("clear" the
% comfortable zone from neighbours). The force acts continuoulsly on all
% pedestrians, even during "overlapping".
% Note that this force should be depending on the pedestrian density (this
% correlation is considered in the compute_force function)
%--------------------------------------------------------------------------
% Input:    > velocities v1, v2
%           > distance between pedestrian 1 and 2
%           > weighting factor fac
%==========================================================================
f = [0;0];
cos_beta = v1'*v2/(norm(v1)*norm(v2)); % angle between velocities

% calculate the angle between v1 and r12
if norm(v1)~=0
    cos_varphi = r12'*v1/(norm(r12)*norm(v1));
else
    cos_varphi=0;
end

cos_alpha = v1'*r12/(norm(v1)*norm(r12));
if cos_alpha <-0.5
    return % sharp cut-off
end

% compute the force: distinguish overtaking and evading situation
lambda = 100;
% pedestrians are not influenced by pedestians walking behind them
f_varphi = (lambda+(1-lambda)*(1-cos_varphi)/2)/lambda;  
f_Deltav = norm(v1-v2);       % velocities have an influence on the force
e = (-r12)/(norm(r12));       % direction: force separets pedestrian
r12 = norm(r12);
if  cos_beta>=0         % pedestrians walk in the same direction
    cut_off = 1;      % no influence at r>cut_off
    B1 = 500;           % amplitude at r=0
    f_r12 = exp(-0.7*r12.^2) .*heaviside(-r12+cut_off); % force with smooth cut_off
else % pedestrians walk against each other
    cut_off = 1;      % no influence at r>cut_off
    B1 = 800;           % amplitude at r=0
    f_r12 = exp(-0.7*r12.^2) .*heaviside(-r12+cut_off); % force with smooth cut_off
end

f = fac*B1*f_varphi*f_r12*f_Deltav.*e;
if isnan(norm(f))
    warning('force is NaN')
    f=[0;0] ;
end

end

