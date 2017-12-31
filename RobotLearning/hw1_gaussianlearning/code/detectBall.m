% Robotics: Estimation and Learning 
% WEEK 1
%
% Student: Alexander Nhan
%
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
mu = load('mu.mat');
mu = mu.mu';
sig = load('sig.mat');
sig = sig.sig;
thre = mvnpdf(mu-0.5*sqrt(diag(sig)),mu,sig); % 0.5 std from mean


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
[nr,nc,nd] = size(I);
prob_map = zeros(nr,nc);
for i=1:nr
    for j=1:nc
        prob_map(i,j) = mvnpdf(double(reshape(I(i,j,:),nd,1)),mu,sig);
    end
end
bw_map = prob_map > thre;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.

% create new empty binary image
bw_biggest = false(size(bw_map));

CC = bwconncomp(bw_map);
numPixels = cellfun(@numel,CC.PixelIdxList);
[~,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;
segI = bw_biggest;
% 
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
