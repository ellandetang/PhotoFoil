% fits pointcloud with some user input to known prism and provides
% transformation matrix

clear
close all
clc


nfig = 0;

%% Load Point Cloud

loadData

%% Load propeller properties

propProperties

%% Draw initial point cloud
nfig = nfig+1;
figure(nfig)
ptCloud = pointCloud(cloudPositions,'color',cloudColors/255);
pcshow(ptCloud,'MarkerSize',.1)
xlabel('X')
ylabel('Y')
zlabel('Z')


%% Define Known Figure
X = 1.9035; % in
Y = 2.003;
Z = 2.0035;

% X is along the rotation axis of the propeller
% Z is along the blade of choice

FaceOrigin = [X/2 0 0; % +x
    -X/2 0 0; % -x
    0 Y/2 0; % +y
    0 -Y/2 0; % -y
    0 0 Z/2; % +z
    0 0 -Z/2]'; % -z
FaceNormal = [1 0 0; % +x
    -1 0 0; % -x
    0 1 0; % +y
    0 -1 0; % -y
    0 0 1; % +z
    0 0 -1]'; % -z


% define the prism from vertices
verticesX = kron([1,-1],ones(1,4))*X/2;
verticesY = kron([1 1],kron([1,-1],ones(1,2))*Y/2);
verticesZ = kron(ones(1,4),[1,-1])*Z/2;

vertices = [verticesX;verticesY;verticesZ];

x = vertices(1,:)';
y = vertices(2,:)';
z = vertices(3,:)';

% each row is a face, each column is a vertex index
faceMap = [1 2 4 3;
    3 4 8 7;
    7 8 6 5;
    5 6 2 1;
    1 3 7 5;
    2 6 8 4];

%% specify seed points for known faces

% If points are selected from positions, use this to copy them
% seeds = cell2mat({cursor_info.Position}')';
% seeds = flip(seeds,2); % data tooltips lists points from last to first, flip order
%USERINPUT

faceIndices = [1 3 5 2 4]; % Indices of which face each seed point corresponds to
% +x +y +z -x -y

% Plane fitting parameters
seedRadius = .125*ones(size(seeds,2),1);
planeDeviation = .015*ones(size(seeds,2),1);
planeRadius = .125*ones(size(seeds,2),1);

% Modify individual instances as desired
seedRadius(1) = .050;
planeRadius(1) = .050;

% Colors in ROYGBIV order
colors = [1 0 0; % Red
    1 .5 0; % Orange
    1 1 0; % Yellow
    0 1 0; % Green
    0 0 1; % Blue
    1 0 1]; % Purple

seedCloud0 = [];
correspondence_Map = [];

% Fit planes using seed points and draw on point cloud
nfig = nfig+1;
figure(nfig)
ptCloud = pointCloud(cloudPositions,'color',cloudColors/255);
pcshow(ptCloud,'MarkerSize',.1)
xlabel('X')
ylabel('Y')
zlabel('Z')

for ind1 = 1:length(faceIndices)
    [plane,pointsPlaneFit,indices,debug] = fitPlane(cloudPositions',seeds(:,ind1),seedRadius(ind1),planeDeviation(ind1),planeRadius(ind1));
    
    planeList(ind1) = plane;
    
    % Draw points found to be in the fitted plane
    data = pointsPlaneFit;
    x = data(1,:)';
    y = data(2,:)';
    z = data(3,:)';
    line(x,y,z,'marker','.','linestyle','none','color',colors(ind1,:));
    
    % Draw Seed point
    data = seeds(:,ind1);
    x = data(1,:)';
    y = data(2,:)';
    z = data(3,:)';
    line(x,y,z,'marker','o','linestyle','none','color',colors(ind1,:));
    
    % Draw Plane normal
    centroid = plane.origin;
    normal = plane.normal;
    data = [centroid,(normal+centroid)];
    x = data(1,:)';
    y = data(2,:)';
    z = data(3,:)';
    line(x,y,z,'marker','.','linestyle','-','color',[0 0 1]);
     
    % Compile plane points and correspondence map
    seedCloud0 = [seedCloud0,pointsPlaneFit];
    correspondence_Map = [correspondence_Map,faceIndices(ind1)*ones(size(find(indices)))];
    
end

%% Fix plane normals to point outwards

psuedoOrigin = mean(cell2mat({planeList.origin}),2);

for ind2 = 1:length(planeList)
    planeList(ind2).normal = planeList(ind2).normal*-sign((psuedoOrigin - planeList(ind2).origin)'*planeList(ind2).normal);
end

%% Starter Transformation based on Plane fit
homogenize = @(in) [in;ones(1,size(in,2))];

m1 = planeList(1).normal;
m2 = planeList(2).normal;
m3 = planeList(3).normal;

m2_1 = m2 - dot(m1,m2)*m1;
m2_1 = m2_1/norm(m2_1);

m3_12 = m3 - dot(m1,m3)*m1  - dot(m2_1,m3)*m2_1;
m3_12 = m3_12/norm(m3_12);

R0 = blkdiag([m1,m2_1,m3_12]',1);

scale0 = X/-dot((planeList(4).origin - planeList(1).origin),planeList(1).normal);
S0 = diag([scale0, scale0, scale0, 1]);

T0 = eye(4);
T0(1:3,4) = -psuedoOrigin;

M0 = R0*S0*T0;

% sListT0 = l*(M0*(sList0 - repmat(psuedoOrigin,[1,size(sList0,2)])));
seedCloud1 = M0*homogenize(seedCloud0);
seedCloud1 = seedCloud1(1:3,:);

%% Draw transformed sample points with correspondence map colors
 
uniqueFaces = unique(correspondence_Map);

for ind3 = 1:length(uniqueFaces)
    colorTemp = colors(faceIndices == uniqueFaces(ind3),:);    
    indicesTemp = find(correspondence_Map == uniqueFaces(ind3));
    seedColors1(indicesTemp,1) = colorTemp(1);
    seedColors1(indicesTemp,2) = colorTemp(2);
    seedColors1(indicesTemp,3) = colorTemp(3);
end

nfig = nfig+1;
figure(nfig)
ptCloud = pointCloud(seedCloud1','color',seedColors1);
pcshow(ptCloud,'MarkerSize',.1)
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on

%% Assemble new point cloud based on iteratively fitted data

% M0 = blkdiag(M0,1);


sListTemp = seedCloud1;

for ind5 = 1:10
    
    sList = sListTemp;
    
    [sListTemp,M,x_opt(:,ind5)] = scalepoint2plane(sList,FaceOrigin,FaceNormal,correspondence_Map);
    
    M0 = M*M0;
    
end


sTrans = M0*homogenize(cloudPositions');
sTrans = sTrans(1:3,:);

%% Plot result

nfig = nfig+1;
figure(nfig)
ptCloud = pointCloud(sTrans','Color',cloudColors/255);
pcshow(ptCloud,'MarkerSize',.1)
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on


%% Plot True prism

for ind1 = 1:size(faceMap,1)
    patch(vertices(1,faceMap(ind1,:)),vertices(2,faceMap(ind1,:)),vertices(3,faceMap(ind1,:)),[0 0 1],'facealpha',.5)
end

%% Transform coordinates to desired position based off of known geometry

% spacerHeight = .4495; %in
spacerHeight = .530; %in

T = eye(4);
T(1:3,4) = -[X/2+spacerHeight+propHubThickness/2,0,0]';

Raw2Working = T*M0;

pointCloudWorking = Raw2Working*homogenize(cloudPositions');
pointCloudWorking = pointCloudWorking(1:3,:);

save('workingTransformation','Raw2Working','pointCloudWorking')

%% Plot Result

nfig = nfig+1;
figure(nfig)
ptCloud = pointCloud(pointCloudWorking','Color',cloudColors/255);
pcshow(ptCloud,'MarkerSize',.1)
xlabel('X')
ylabel('Y')
zlabel('Z')
hold on
