% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 

% the number of grids for 1 meter.
myResol = param.resol; % cells/meter
% the initial map size in pixels
myMap = zeros(param.size);
% the origin of the map in pixels
myorigin = param.origin; 

% 4. Log-odd parameters 
lo_occ = param.lo_occ;
lo_free = param.lo_free; 
lo_max = param.lo_max;
lo_min = param.lo_min;

N = size(pose,2);
num_rays = length(scanAngles);
for j = 1:N % for each time,
    d = ranges(:,j)';
    
    % Find grids hit by the rays (in the gird map coordinate)
    grid_hits = [d.*cos(pose(3,j)+scanAngles)'; -d.*sin(pose(3,j)+scanAngles)'] + pose(1:2,j);
    
    % Find occupied-measurement cells and free-measurement cells
    occupied_cells_idx = ceil(myResol*grid_hits) + myorigin;
    free_cells = [];
    occupied_cells = [];
    for i=1:num_rays
        orig_x = ceil(myorigin(1)+myResol*pose(1,j));
        orig_y = ceil(myorigin(2)+myResol*pose(2,j));
        [free_x,free_y] = bresenham(orig_x,orig_y,occupied_cells_idx(1,i),occupied_cells_idx(2,i));
        free_cells = [free_cells; sub2ind(param.size,free_y,free_x)];
        occupied_cells = [occupied_cells; sub2ind(param.size,occupied_cells_idx(2,i),occupied_cells_idx(1,i))];
    end
    free_cells = unique(free_cells);
    occupied_cells = unique(occupied_cells);
    
    % Update and saturate the log-odds
    for i=1:length(occupied_cells)
        myMap(occupied_cells(i)) = min(myMap(occupied_cells(i)) + lo_occ, lo_max);
    end
    for i=1:length(free_cells)
        myMap(free_cells(i)) = max(myMap(free_cells(i)) - lo_free, lo_min);
    end

end

end

