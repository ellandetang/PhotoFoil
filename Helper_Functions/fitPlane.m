function [plane,points,indices,debug] = fitPlane(pointCloudIn,seedCoordinate,seedRadius,planeDeviation,planeRadius)
% Fits a plane to a point cloud using a 2 iterative process
% pointCloud 3xn matrix
% seedCoordinate 3x1 vector
% seedRadius scalar
% planeDeviation scalar

% Distances from a point to a cloud of points
cDist = @(cloud,point) sqrt(sum((cloud - repmat(point,[1,size(cloud,2)])).^2,1));
% Distances from a plane to a cloud of points
pDist = @(cloud,plane) ((cloud - repmat(plane.origin,[1,size(cloud,2)]))'*plane.normal)';

% Find all points within a certain radius
selIndSeed = cDist(pointCloudIn,seedCoordinate) < seedRadius;
dataSeed = pointCloudIn(:,selIndSeed);

debug.initialPoints = dataSeed;

% Fit plane to initial seed data
[normalSeed,centroidSeed] = planeFit(dataSeed);
planeSeed.normal = normalSeed;
planeSeed.origin = centroidSeed;

% Find points close to initial plane fit
selIndPlane = abs(pDist(pointCloudIn,planeSeed)) < planeDeviation;
if nargin == 5
    selIndRadius = cDist(pointCloudIn,planeSeed.origin) < planeRadius;
    selInd = selIndPlane & selIndRadius;
else
    selInd =  selIndPlane;
end
dataFit_1 = pointCloudIn(:,selInd);

% Use points to perform better fit of plane

[normal,centroid] = planeFit(dataFit_1);

plane.normal = normal;
plane.origin = centroid;

indices = abs(pDist(pointCloudIn,plane)) < planeDeviation;
if nargin == 5
    indices = indices & selIndRadius;
end
points = pointCloudIn(:,indices);


end


function [normal,centroid] = planeFit(in)
%% Fit plane to group of points

centroid = mean(in,2);
points = in - centroid;
A = points;
[U,S,~] = svd(A);
[~,indMin] = min(diag(S));
normal = U(:,indMin);
end



%% Debug plots

% data = pointCloudIn;
% x = data(1,:)';
% y = data(2,:)';
% z = data(3,:)';
% line(x,y,z,'marker','.','linestyle','none','color',[0 0 0]);
% grid on
% axis equal
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
%
% data = [centroid,(normal+centroid)];
% data = [centroidSeed,(normalSeed+centroidSeed)];
% x = data(1,:)';
% y = data(2,:)';
% z = data(3,:)';
% line(x,y,z,'marker','.','linestyle','-','color',[0 0 1]);
%
% data = pointCloudIn(:,indices);
% x = data(1,:)';
% y = data(2,:)';
% z = data(3,:)';
% line(x,y,z,'marker','.','markersize',10,'linestyle','none','color',[0 1 0]);
%
% test = [-.1796 .8519 .7484]';
% pDist =