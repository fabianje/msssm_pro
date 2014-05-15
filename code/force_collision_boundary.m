function [f, dist_critical, traffic_lights] = force_collision_boundary(v, x)
%==========================================================================
% Calculation of the force that prevents from collision with boundary
% points.
%--------------------------------------------------------------------------
% Input:    > velocity and position x
% Output:   > force
%           > helper values: dist_critical, traffic_lights
%==========================================================================

if norm(v) <0.1
    % somteomes pedestrian maintain in a "totally-block-situation".
    % Normally, this situation occurs on boundary points. To avoid such
    % situtations use a suitable greate ampluttue.
    C1 = 10000;
else
    C1 = 100; 
end
C2 = 1/2;
cut_off = 0.5;

[D_bp, dist_critical, traffic_lights] = pedestrian_boundary_distance(x);
D_bp_norm = norm(D_bp);
f = C1*exp(-D_bp_norm^3/C2)*heaviside(-D_bp_norm+cut_off) .* D_bp/(D_bp_norm);

if isnan(norm(f))
   warning('Pedestrian-boundary collision -- force is NaN')
   f=[0;0] ;
end


end

