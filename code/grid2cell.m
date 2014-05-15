function [Cell_Grid] = grid2cell(pedestrian_saver, Ncell, Mcell)
%==========================================================================
% This function uses a cell storage method of Grid to reduce the numerical
% effort when computing interaction between several pedestrians. The Grid
% is subdevided into cells and all pedestrian are assigned to one cell.
%--------------------------------------------------------------------------
% Input:    > pedestrian_saver
%           > dimensions of the cell Ncell and Mcell
% Output:   > Cell_Grid: Matrix that subdivides the grid into cells
%==========================================================================

global N M

Cell_Grid = cell(Ncell, Mcell);
p = length(pedestrian_saver);

for id=1:p
   current =  pedestrian_saver{id};
   % determine indices of the cell
   celli = floor(Ncell*(current.indices(1)-0.9)/N)+1;
   cellj = floor(Mcell*(current.indices(2)-0.9)/M)+1;
   % save current pedestrian into Cell_Grid
   Cell_Grid{celli, cellj} = [Cell_Grid{celli, cellj} id]; % "push back"
end


end

