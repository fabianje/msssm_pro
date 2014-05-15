function [f] = force_collision_pedestrian(x1, x2, v1, v2, density)
%==========================================================================
% This force produces an impulse that acts only in some updating
% steps but has a hugh amplitude. It tries to correct the current
% direction and prevent of a total-blocking situation. This force has
% no effact if two pedestirans "overlap".
% such that collisions with other pedestrians can be avoided.
%---------------------------------------------------------------------------
% Input:    > position and velocity of pedestrian 1 and pedestrian 2
%           > density
% Output:   > force f
%==========================================================================

f = [0;0];  % assume no interaction

% do not compute the force if evading does not help. That is if
%   >> density is too great. Evading will produce an unatural float
%   behavior
%   >> the neighbour remains on its current position (v2<0.05). The
%   vanishing neighbour velocity indicates high density. Notice that the
%   parameter "density" can fail becouse we use the efficient cell storage
%   method. An other reason for the failur of "density" might be a small
%   cut off (which depends on the density on the last iterations)
if density>2 || norm(v2)<0.05
    return
end

r12 = x2-x1;

% First of all, Pedestrian 1 has to regognize in which situation he is
% > he walks behind pedestrian 2 (overtaking might be a possibility)
% > he walks in front of pedestrian 2 (evading might be a possiblility)
cos_beta = v1'*v2/(norm(v1)*norm(v2)); % angle between velocities
if  cos_beta>=0 % pedestrians walk in the same direction (-> overtaking?)
    % expected dinstance until collision occurs
    distance0 = norm(r12)*(norm(v1)+norm(v2))/(norm(v1)-norm(v2));
else % pedestrians walk against each other (-> evading?)
    distance0 = norm(r12);
end


% perform a coordinate transformation (v1 -> new x-axis):
cos_gamma = [1 0]*v1/(norm(v1)); % angle between x-axis and v1
sin_gamma = (1-cos_gamma^2)^(1/2);

if v1(2)>0
    M = [cos_gamma sin_gamma; -sin_gamma cos_gamma]; % rotation in positive direction
else
    M = [cos_gamma -sin_gamma; sin_gamma cos_gamma]; % rotation in negative direction
end

% transformation of same date
V1 = M*v1;
V2 = M*v2;
X1 = M*x1;
X2 = M*x2;
R12= M*r12;

if V1(2)>10^-5 % check if transformation was successful
    warning('Coordinate transformation failed')
end

% Decide in which direction the pedestrian want to evade/ overtake
cos_alpha = [1,0]*r12/(norm(r12)); % angle between x-axis and r12 
if abs(cos_alpha) > 0.9 % special case if x-axis is approximalty parallel with r12. 
    % alpha depends on the coordinate system and has therefore no physical
    % meaning. However, this special case solves the problem of
    % "oscilating pedestirans", that is, if y-component of the neighboour's
    % velocity vector changes its sign and the current pedestrian reacts
    % with changing the evading/overtaking direction (sometomes this
    % yields in abstruse situatios)
    e = ([r12(2); -r12(1)])/norm(r12); % evade/ overtake to the righ
elseif R12(2)>=0 % make somthing for this case (random implemented)
    if V2(2)<=0
       e = ([r12(2); -r12(1)])/norm(r12); % evade/ overtake to the right
    else
        e = ([-r12(2); r12(1)])/norm(r12); %evade/ overtake to the left
    end
else % do exactly the oppposite
    if V2(2)<=0
        e = ([r12(2); -r12(1)])/norm(r12); % evade/ overtake to the right
    else
        e = ([-r12(2); r12(1)])/norm(r12); %evade/ overtake to the left
    end
end
% Notice: This algorithm works properly for evading and overtaking
% siuations such that the pedestrians do not make the same decision (
% evade/ overtake in the same direction). A better approach, for instante,
% might be based on reaction-decision-making: One pedestrian decide to
% evade/overtake and the other pedestrians occupies the situation and reacts
% on the neighbour's movement. However, This model is succiciently exact
% for high pedestrian flow systems and provides even in a 2-pedestrian-
% interaction-system good results.

% With the coordinate transformation is is also possible to detect if a
% collision can be excluded:
if X2(1)<X1(1) % neighbour is behind me (I can not see him!!)
    distance0 = -1; % set distance0 to a any invalid value
elseif (X2(2)>0 && V2(2)>0) || (X2(2)<0 && V2(2)<0) % neighbour moves away
    distance0 = -1;
elseif (X2(1)>0 && V2(2)>0) || (X2(1)<0 && V2(2)<0) % neighbour moves away
end

[Theta, Delta_t] = is_collision(x1, x2, v1, v2, distance0);

% Now derive a force that describes the interaction behavior
% > the bigger the expected time until collision uccures is, the bigger the
%   influence becomes
A1 = 6000/(density^2+1);
if Theta == 1
    % maximum Amplitude depends linearly on number of force influences 
    % the force potential is assumed to be a function of Delta_t with a
    % weakly exponentially decreasing potential and without cut-off.
    f = A1*exp(-0.001*Delta_t.^2) .*e;
elseif norm(v1) == 0 % special case if two pedestrians are blocked
    f = A1.*e;
else
    f = [0;0];
end

if isnan(norm(f))
    warning('force is NaN')
    f=[0;0] ;
end


end

