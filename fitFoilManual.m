% Grab foil twist and chord manually from scaled and aligned data
% Sometimes the fitting algorithm doesn't work well. A manual selection of
% leading and trailing edges at each section might be more appropriate in
% this case.

clear
close all
clc

nFig = 0;

% Load propeller properties
propProperties


% User Input
nSections = 41;% Number of profiles, root and tip inclusive, please make odd integer
sampleThickness = .025; % Total thickness of point cloud sample for each section
truncationBounds = [.05,.95]; % Percent of each airfoil to remove to ignore edge noise

addpath('Helper_Functions')
load('bladeData.mat')

%% Plot Point Cloud for reference

nFig = nFig+1;
figure(nFig)

ptCloud = pointCloud(bladeData','Color',bladeColor);
pcshow(ptCloud);
xlabel('X')
ylabel('Y')
zlabel('Z')


%% Divide blade into sections

nFig = nFig+1;
figure(nFig)

RootSectionZ = trueRadius*.15;
TipSectionZ = trueRadius*.95;

% Set of variable storage for section Properties
sectionsZ = linspace(RootSectionZ,TipSectionZ,nSections); % z coordinate of each section
sectionsX = zeros(7,nSections); % Airfoil fitting parameters

for ind2 = 1:2
    
    if ind2 == 1
        disp('Right Click on Leading Edge')
    elseif ind2 == 2
        disp('Right Click on Trailing Edge')
    end
    
    for ind1 = 1:length(sectionsZ)
        
        clf
        
        indSec = bladeData(3,:) > sectionsZ(ind1) - sampleThickness/2 & bladeData(3,:) < sectionsZ(ind1) + sampleThickness/2;
        scatter(bladeData(1,indSec),bladeData(2,indSec),2,bladeColor(indSec,:))
        grid on
        axis equal
        xlabel('X')
        ylabel('Y')
        title(sprintf('Section Z = %1.2f, (%i%%)',sectionsZ(ind1),round(sectionsZ(ind1)/trueRadius*100)))
        
        [xi,yi] = getpts;
        
        if ind2 == 1
            LE(:,ind1) = [xi(1) yi(1)]';
        elseif ind2 == 2
            TE(:,ind1) = [xi(1) yi(1)]';
        end
    end
    
end


%% Calculate Blade Parameters

radius = sectionsZ;
LECoordinates = LE;

chord = sqrt(sum((LE - TE).^2,1));
twist = rad2deg(atan2(LE(1,:) - TE(1,:),bladeDirection*(LE(2,:) - TE(2,:))));


%% If the manual selection has been done, load that in

if exist('dataOutManual.mat','file')
   load('dataOutManual.mat'); 
end

%% Calculate secondary parameters

rotZ =@(th) [cos(th) -sin(th);
    sin(th) cos(th)];

thickness = zeros(size(radius));
camber = zeros(size(radius));
camberLoc = zeros(size(radius));
cloudRMSE = zeros(size(radius));
nSlicePoints = zeros(size(radius));

for ind3 = 1:length(radius)
    
    indSec = bladeData(3,:) > radius(ind3) - sampleThickness/2 & bladeData(3,:) < radius(ind3) + sampleThickness/2;
    dataIn = [bladeData(1,indSec);bladeData(2,indSec)];
    dataIn = (dataIn - repmat([LECoordinates(1,ind3),LECoordinates(2,ind3)]',[1,size(dataIn,2)]))/chord(ind3);
    dataIn = rotZ(bladeDirection*deg2rad(twist(ind3)))*dataIn;
    dataIn = [-bladeDirection*dataIn(2,:);dataIn(1,:)];
    
    indSel = dataIn(1,:) > truncationBounds(1) & dataIn(1,:) < truncationBounds(2);
    dataIn = dataIn(:,indSel);
    
    [x] = fitAirfoilParams(dataIn);
    thickness(ind3) = x(1);
    camber(ind3) = x(2);
    camberLoc(ind3) = x(3);
    
    
    % calculate RMSE  
    xSample1 = linspace(0,.01,11);
    xSample2 = linspace(.01,1,90);
    xSample = [xSample1(1:end-1),xSample2];
    nacaProfile = NACAProfile(x(1),x(2),x(3),xSample);

    [dataDistance] = contourDistance(nacaProfile,dataIn);

    cloudRMSE(ind3) = sqrt(mean(dataDistance.^2));
    nSlicePoints(ind3) = length(dataIn);
    
end

%%
save('dataOutManual','radius','twist','chord','LECoordinates','thickness','camber','camberLoc','cloudRMSE','nSlicePoints')

%%

nFig = nFig+1;
figure(nFig)

subplot(2,1,1)
line(radius,twist,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('Twist (deg)')

if exist('twist','var')
rSample = linspace(trueRadius*.15,trueRadius);
line(rSample,rad2deg(atan(pitch./(rSample*2*pi))),'color',[0 0 0])
end

subplot(2,1,2)
line(radius,chord,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('Chord (in)')


nFig = nFig+1;
figure(nFig)

subplot(3,1,1)
line(radius,thickness*100,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('Thickness (%)')

subplot(3,1,2)
line(radius,camber*100,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('Camber (%)')

subplot(3,1,3)
line(radius,camberLoc*100,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('Camber Location (%)')

nFig = nFig+1;
figure(nFig)

line(radius,cloudRMSE,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('RMSE error (in)')
