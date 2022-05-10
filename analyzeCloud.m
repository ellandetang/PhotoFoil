% Truncates data and performs color based filtering to clean point cloud

clear
close all
clc

%% Load propeller properties

propProperties

%% Load and Display initial Data
load workingTransformation
load cloudData

data = pointCloudWorking';
colors = cloudColors;

n = length(data);
data1 = data';
figure(1)

ptCloud = pointCloud(data,'Color',colors/255);
pcshow(ptCloud)
xlabel('X')
ylabel('Y')
zlabel('Z')

%% Manual Rotation and Translation Corrections and Data Truncation and Color Based Data Filtering

rotX = @(th) [1 0 0
    0 cos(th) -sin(th)
    0 sin(th) cos(th)];
rotY = @(th) [cos(th) 0 sin(th)
    0 1 0
    -sin(th) 0 cos(th)];
rotZ = @(th) [cos(th) -sin(th) 0;
    sin(th) cos(th) 0 ;
    0 0 1];

data2 = rotZ(deg2rad(0))*data1;
data2 = rotY(deg2rad(0))*data2;
data2 = rotX(deg2rad(0))*data2;
data2 = data2 - repmat([0 0 0]',[1,size(data2,2)]);


boundsX = trueRadius*[-.1,.1]; %USERINPUT
boundsY = trueRadius*[-0.2,0.2]; %USERINPUT
boundsZ = trueRadius*[-0.05 1.05]; %USERINPUT

selectionIndices = data2(1,:) > boundsX(1) & data2(1,:) < boundsX(2) & ...
    data2(2,:) > boundsY(1) & data2(2,:) < boundsY(2) & ...
    data2(3,:) > boundsZ(1) & data2(3,:) < boundsZ(2);

data3 = data2(:,selectionIndices);
color3 = colors(selectionIndices,:)/255;


% Color Analysis

figure(3) % Color Frequency histogram
clf

colorSample = [97 98 95;
    228 75 61;
    96 146 95]/255; % Select colors corresponding to points to be eliminated
colorThreshold = [0.05 0.1 .05 0]; % Threshold of similarity for elimination
%USERINPUT

colorInd = true(size(color3,1),1);
colorDist = zeros(size(color3,1),size(colorSample,1));
for ind3 = 1:size(colorSample,1)

colorDist(:,ind3) = sum((color3 - repmat(colorSample(ind3,:),[size(color3,1),1])).^2,2);
% colorDist(:,ind3) = sqrt(colorDist); % Optional sqrt of color distance

colorInd = colorInd & (colorDist(:,ind3) > colorThreshold(ind3));

subplot(size(colorSample,1),1,ind3)
histogram(colorDist(:,ind3))
title(sprintf('Color Match Histogram for %i,%i,%i',colorSample(ind3,1)*255,colorSample(ind3,2)*255,colorSample(ind3,3)*255))

end



data4 = data3(:,colorInd);
color4 = color3(colorInd,:);


% Plot Latest transformation/filtering


figure(4)
clf
viewAngles = [90 180 90;
    90 0 0];
titleList = {'Top','Side','Front'};
for ind2 = 1:3
    
     
    subplot(1,3,ind2)
    ptCloud = pointCloud(data4','Color',color4);
    pcshow(ptCloud)
    
    %axis equal
    %grid on
    xlabel('X')
    ylabel('Y')
    zlabel('Z')
    view(viewAngles(1,ind2),viewAngles(2,ind2))
    ax = gca;
    ax.Color = [1 1 1];
    ax.XColor = [.15,.15,.15];
    ax.YColor = [.15,.15,.15];
    ax.ZColor = [.15,.15,.15];
    ax.MinorGridColor = [0.1000 0.1000 0.1000];
    ax.GridColor = [0.1500 0.1500 0.1500];
    ax.Color = [1 1 1];
end

fig = gcf;
fig.Color = [1 1 1];

% Plot samples of sections for quality reference
figure(5)
sampleZ = linspace(boundsZ(1),boundsZ(2),10);
sampleThickness = .05;
for ind4 = 1:8
   subplot(4,2,ind4)
   indSel = data4(3,:) > sampleZ(ind4+1)-sampleThickness/2 & data4(3,:) < sampleZ(ind4+1)+sampleThickness/2;
   plot(data4(1,indSel),data4(2,indSel),'k.')
    grid on
    axis equal
end

%% Final Verification

figure(6)

ptCloud = pointCloud(data4','Color',color4);
pcshow(ptCloud)
xlabel('X')
ylabel('Y')
zlabel('Z')

bladeData = data4;
bladeColor = color4;
save('bladeData','bladeData','bladeColor')
