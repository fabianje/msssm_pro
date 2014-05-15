function [] = traffic_lights_regulator()
%==========================================================================
% Helper function for ragulating the trafic light dynamic:
% After each traffic_light_time all traffic lieght change to red. Aftre
% additional traffic_light__time_closed some other traffic light are
% changing to green.
%==========================================================================

global  traffic_light_time traffic_light__time_closed number_of_traffic_lights_red

load('variables', 'time', 'time_where_branch_is_closed')

    
% close all branches 
if mod(time,traffic_light_time+traffic_light__time_closed)==0
    time_where_branch_is_closed = time;
    save('variables.mat', 'time_where_branch_is_closed', '-append')
    load('variables', 'lights', 'bounds')
    bounds.traffic_lights = lights;
    save('variables.mat', 'bounds','-append') 
    
     
% change traffic light
elseif time == time_where_branch_is_closed + traffic_light__time_closed   
    load('variables', 'lights', 'branch_number', 'bounds')
    
    % empty the saver 
    bounds.traffic_lights=[];
            
    % update branch numbers
    branch_number = branch_number+(4-number_of_traffic_lights_red);
    for i=1:length(branch_number)
        if branch_number(i)>4
            branch_number(i)= branch_number(i)-4;
        end
        % reallocation
        bounds.traffic_lights = [bounds.traffic_lights, [lights(:,2*branch_number(i)-1:2*branch_number(i)) ]];
    end
     save('variables.mat', 'bounds','-append')
     save('variables.mat', 'branch_number','-append')
end


end

