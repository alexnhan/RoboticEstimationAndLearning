% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 

% the number of grids for 1 meter.
myResolution = param.resol;
% the origin of the map in pixels
myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.

map_size = size(map);

% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 700;                        % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
%P = repmat(myPose(:,1), [1, M]);
%W = 1/M*ones(1,M); % initial weights uniform

sig_m = diag([0.01 0.01 0.03]);
for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).

    % 1) Propagate the particles 
    P = mvnrnd(myPose(:,j-1), sig_m, M)'; % propagate with sample noise
      
    % 2) Measurement Update 
    d = ranges(:,j)';
    correlation_scores = zeros(1,M);
    for m=1:M
        %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
        pose = P(:,m);
        grid_hits = [d.*cos(pose(3)+scanAngles)'; -d.*sin(pose(3)+scanAngles)'] + pose(1:2);
        occupied_cells_idx = ceil(myResolution*grid_hits) + myOrigin;
        % remove indices out of map boundaries
        rem_idx = occupied_cells_idx(1,:) > map_size(2) | occupied_cells_idx(2,:) > map_size(1) | occupied_cells_idx(1,:) < 1 | occupied_cells_idx(2,:) < 1;
        occupied_cells_idx(:,rem_idx) = [];
        occupied_cells_idx = unique(sub2ind(map_size, occupied_cells_idx(2,:), occupied_cells_idx(1,:)));
        
        %   2-2) For each particle, calculate the correlation scores of the particles
        % count true occupied cell score
        correlation_scores(m) = 10*sum(map(occupied_cells_idx)>0.49);
        % count false occupied cell score
        correlation_scores(m) = correlation_scores(m) - 2*sum(map(occupied_cells_idx)<0);
    end
    
    %   2-3) Update the particle weights
    correlation_scores = correlation_scores/sum(correlation_scores);
 
    %   2-4) Choose the best particle to update the pose
    [~,p] = max(correlation_scores);
    myPose(:,j) = P(:,p);
    
    % 3) Resample if the effective number of particles is smaller than a threshold
%     num_effective_particles = sum(W)^2/sum(W.^2);
%     thresh = 0.1;
%     if num_effective_particles < thresh*M
%         P = P(:,randsample(M,M,true,W));
%     end

    % 4) Visualize the pose on the map as needed
   

end

end

