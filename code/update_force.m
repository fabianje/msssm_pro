function [pedestrian_saver, potential_blocking_indices] = update_force(pedestrian_saver, Grid)
%==========================================================================
% This function computes the force for each pedestrian
%--------------------------------------------------------------------------
% Input:    > pedestrian_saver
%           > Grid
% Output:   > updated pedestrian_saver
%           > potential_blocking_indices: indices of pedestrian in a small
%           range around an other pedestrian
%==========================================================================
global N M Ncell Mcell interaction_distance r

fmin=0;
fmax = 500;

potential_blocking_indices = cell(1,length(pedestrian_saver));

% subdivide the grid into cells
Cell_Grid = grid2cell(pedestrian_saver, Ncell, Mcell);

for id=1:length(pedestrian_saver)
    indices_saver = [];
 
    density = 0;% is a kind of density-indicator
    current = pedestrian_saver{id};    
    v = current.velocity;
    x = current.position;
    i = current.indices(1);
    j = current.indices(2);
 
    f = zeros(2,1);
    
    % compute indices of the cell in which the current pedestrian is located
    celli = floor(Ncell*(current.indices(1)-0.9)/N)+1;
    cellj = floor(Mcell*(current.indices(2)-0.9)/M)+1;

    % Calulate indices of "relevant" neighbour cells (note that pedestrians
    % are assumend to be blind for everything that happens behind them)
    accurancy = 1;      % round without decimal accuracy
    if abs(round(v(1)/accurancy)*accurancy) > abs(round(v(2)/accurancy)*accurancy)
        % "main direction": [1; 0];
        i_stop  = 1;
        i_start = -1;
        j_stop  = max(sign(v(1)), 0);
        j_start = j_stop-1;
    elseif abs(round(v(1)/accurancy)*accurancy) < abs(round(v(2)/accurancy)*accurancy)
        % "main direction": [0; 1];
        i_stop  = max(sign(v(2)), 0);
        i_start = i_stop-1;
        j_stop  = 1;
        j_start = -1;
    else % iterate over 3 neighbour cells
        % " main direction" [1; 1]/sqrt(2)
        i_stop  = max(sign(v(2)), 0);
        i_start = i_stop-1;
        j_stop  = max(sign(v(1)), 0);
        j_start = j_stop-1;      
    end
    
    % iterate over 9 neighbour cells (exlude all cells where no cell
    % interaction is expected)
    for di = i_start:i_stop
        for dj = j_start:j_stop
            % compute neighbour indices (no enfocing of periodic boundary
            % condition -> this would not make much sense when trying to model
            % in intersection!)
            if celli+di==0 || celli+di==Ncell+1 || cellj+dj==0 || cellj+dj==Mcell+1
                % the neighbour cell does not exist; assume an empty cell
                % instead (non periodic boundary conditions)
                neighbour_cell = [];
            else
                celli_neighbour = celli+di;
                cellj_neighbour = cellj+dj;
                neighbour_cell = Cell_Grid{celli_neighbour, cellj_neighbour};
            end
       
            % Influence of the density (density measures the number of
            % forces actung on the current pedestrian. It is therefore an
            % indicator for future density)
            fac1 = min(max(1-current.density/5, 0), 1);
            
            for k=1:length(neighbour_cell)
                neighbour_id = neighbour_cell(k);
                neighbour = pedestrian_saver{neighbour_id};
                i_neighbour = neighbour.indices(1);
                j_neighbour = neighbour.indices(2);
                distance = norm(current.position-neighbour.position);
                
                % Influence of the relative differnce velocity
                % ( 1-(v-v0)/v0 is an indicator for the actual density )
                fac2 = min(max(2-norm(current.velocity)/current.initial_velocity, 0), 1);
                
                % Intoduce a sharp cut-off
                %   > to have a better acceleration behavior for high
                %   density crowd flows
                %   > to improve the updating speed
                cut_off = max(interaction_distance*abs(fac1*fac2), r+0.3);
                % avoid interaction  with "myself" and apply cut-off
                if norm([i j])-norm([i_neighbour,j_neighbour])~=0 && distance <cut_off
                    
                    x_neighbour = neighbour.position;
                    v_neighbour = neighbour.velocity;
                    % check if neighbour is visible (notice that the following
                    % computations holds for "boundary-cells" as well
                    
                    % avoid collision with pedestrians (iterate over each
                    % current-neighbour-pair only once)
                    f3 = force_collision_pedestrian(x, x_neighbour, v, v_neighbour, current.density);
                    f=f+f3;
 
                    % ensure comfortable zone (redurce the force if many neighbours are around)
                    f4 = force_comfortable_zone(v, v_neighbour, x_neighbour-x, min(1/(fac1*fac2^2+0.1),10));
                    f = f+f4;
                    
                    % Update density value
                    density = density+1;
                    
                    % Finally, save the indices of the neighbour pedestrian
                    % (also save its velocity)
                    indices_saver = [indices_saver [i_neighbour; j_neighbour; v_neighbour]];
                end       
            end
        end
    end

    % approach target
    [f1, location] = force_reach_target(current, current.location, Grid);
    f = f+f1;

    % avoid collision with boundary
    [f2, dist_critical, traffic_lights] = force_collision_boundary(v, x);
    f = f + f2;
    
    % restrict the force
    if norm(f)>0  
        f = f/norm(f) * max(min(norm(f), fmax), fmin);
    end
    % update force of the current pedestrian
    pedestrian_saver{id}.force=f;
    pedestrian_saver{id}.location = location;
    pedestrian_saver{id}.density = density;
    pedestrian_saver{id}.dist_critical = dist_critical;
    pedestrian_saver{id}.traffic_lights = traffic_lights;
    potential_blocking_indices{id} = indices_saver;
    
end
 
% Notice that we did not implement any following or friction forces. Those
% effects are generated automatically!

end

