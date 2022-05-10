clear
close all
clc

%% Load propeller properties

propProperties

%% User Input

nSections = 41;% Number of profiles, root and tip inclusive, please make odd integer
sampleThickness = .025; % Total thickness of point cloud sample for each section
truncationBounds = [.10,.90]; % Percent of each airfoil to remove to ignore edge noise

%%
addpath('Helper_Functions')

load bladeData.mat

nFig = 0;

nFig = nFig+1;
figure(nFig)

% Plot Point Cloud for reference

ptCloud = pointCloud(bladeData','Color',bladeColor);
pcshow(ptCloud);
xlabel('X')
ylabel('Y')
zlabel('Z')


%% Divide blade into sections for analysis

% Coordinate system
% Z along blade span, blade of interest pointing upwards
% X Thrust Axis

RootSectionZ = trueRadius*.15;
TipSectionZ = trueRadius*.95;

indMid = (nSections+1)/2; % index of middle section

% Set of variable storage for section Properties
sectionsZ = linspace(RootSectionZ,TipSectionZ,nSections); % z coordinate of each section
sectionsX = zeros(7,nSections); % Airfoil fitting parameters

indSec = bladeData(3,:) > sectionsZ(indMid) - sampleThickness/2 & bladeData(3,:) < sectionsZ(indMid) + sampleThickness/2;


%% Transform airfoil for initial manual fit

secData1 = bladeData(1:2,indSec);

% Draw seed section
nFig = nFig+1;
figure(nFig)
line(secData1(1,:),secData1(2,:),'color',[0 0 0],'linestyle','none','marker','.','markersize',1)
grid on
axis equal


disp('Select Approximate Leading Edge, Trailing Edge, and Upwards Direction')
if ~exist('xi','var')
    [xi,yi] = getpts;
end
LE = [xi(1) yi(1)]';
TE = [xi(2) yi(2)]';
yPos = [xi(3) yi(3)]';

Vx = TE-LE;
VxMag = norm(Vx);
VyTemp = yPos - LE;
Vy = VyTemp - dot(VyTemp,Vx)*Vx/VxMag.^2;
VyMag = norm(Vy);


% Mid Section to global transform in form Ax + b = y;
mid2glob.A = [Vx,(Vy*VxMag/VyMag)];
mid2glob.b = LE;

% Store Transform to Mid Section Coordinate System
sectionTransform = @(dataIn) ([Vx/VxMag,Vy/VyMag]\(dataIn - repmat(LE,[1,size(dataIn,2)])))/VxMag;
% Store Tansform from Mid Section to Global Coordinate System
globalTransform = @(dataIn) [Vx/VxMag,Vy/VyMag]*(dataIn*VxMag) + repmat(LE,[1,size(dataIn,2)]);

secData2 = sectionTransform(secData1);

%% Perform first fit of mid-section

secData3 = secData2(:,secData2(1,:) > truncationBounds(1) & secData2(1,:) < truncationBounds(2));

foilTransform = @(x,data) ([cos(x(3)) -sin(x(3));
    sin(x(3)) cos(x(3))]*data + repmat([x(1),x(2)]',[1,size(data,2)]))*x(4);
dataTransform = @(x,data) ([cos(-x(3)) -sin(-x(3));
    sin(-x(3)) cos(-x(3))]*data./x(4) - repmat([x(1),x(2)]',[1,size(data,2)]));

x = fitAirfoilSection(secData3);
sectionsX(:,indMid) = x;
nacaProfile = NACAProfile(x(1),x(2),x(3),linspace(0,1));
nacaProfile = foilTransform(x(4:7),nacaProfile);

% line(nacaProfile(1,:),nacaProfile(2,:),'color',[0 0 0])

%%
% Draw initial fit
figure(2)
clf
subplot(1,3,1)
%scatterColor = zeros(size(bladeData,2),3);
scatterColor = bladeColor;
scatterColor(indSec,1) = 1;
scatterEntity = scatter3(bladeData(1,:),bladeData(2,:),bladeData(3,:),1,scatterColor,'.');


grid on
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Blade Point Cloud - Section of Interest in Red')
view(44,36)
xl = xlim;
yl = ylim;
zl = zlim;

subplot(1,3,2)

nacaProfile = NACAProfile(x(1),x(2),x(3),linspace(0,1));
nacaProfile = foilTransform(x(4:7),nacaProfile);
nacaProfile = globalTransform(nacaProfile);
line(nacaProfile(1,:),nacaProfile(2,:),repmat(sectionsZ(indMid),[1,size(nacaProfile,2)]),'color',[0 0 0])

xlim(xl)
ylim(yl)
zlim(zl)
grid on
axis equal
xlabel('X')
ylabel('Y')
zlabel('Z')
title('Blade Spacial Reconstruction')
view(44,36)

subplot(1,3,3)
nacaProfile = NACAProfile(x(1),x(2),x(3),linspace(0,1));
nacaProfile = foilTransform(x(4:7),nacaProfile);
line(secData2(1,:),secData2(2,:),'color',[1 0 0],'linestyle','none','marker','.','markersize',1)
line(secData3(1,:),secData3(2,:),'color',[0 0 0],'linestyle','none','marker','.','markersize',1)
line(nacaProfile(1,:),nacaProfile(2,:),'color',[0 0 0])
grid on
axis equal
xlabel('X')
ylabel('Y')
title('Section of Interest Data and Airfoil Fit')

%% Perform fit of next sections

% Assemble order of indices
sectionIndexList = [(indMid+1:nSections),(indMid-1:-1:1)];
% Offset map
prevSectionIndexOffset = [ones(1,(nSections-1)/2),0,-ones(1,(nSections-1)/2)];

for ind1 = sectionIndexList
    
    % Select Sample data
    indSec = bladeData(3,:) > sectionsZ(ind1) - sampleThickness & bladeData(3,:) < sectionsZ(ind1) + sampleThickness;
    
    % Plot point cloud with current section to be analyzed
    subplot(1,3,1)
    scatterEntity.CData = bladeColor;
    scatterEntity.CData(indSec,1) = 1;
    scatterEntity.CData(indSec,2) = 0;
    scatterEntity.CData(indSec,3) = 0;
    drawnow
    
    indOff = prevSectionIndexOffset(ind1); %Offset of adjacent fit
    % Fit airfoil
    secData1 = bladeData(1:2,indSec); % Select blade section data
    secData2 = sectionTransform(secData1); % transform data to blade coordinate system
    secData3 = dataTransform(sectionsX(4:7,ind1 + indOff),secData2); % transform data based off previous fit
    secData4a = secData2(:,secData3(1,:) < truncationBounds(1) | secData3(1,:) > truncationBounds(2)); % show truncated data
    secData4 = secData2(:,secData3(1,:) > truncationBounds(1) & secData3(1,:) < truncationBounds(2)); % truncate edges
    x = fitAirfoilSection(secData4,sectionsX(:,ind1 + indOff));
    sectionsX(:,ind1) = x;
    
    % Plot fitted Airfoil Curve
    subplot(1,3,2)
    nacaProfile = NACAProfile(x(1),x(2),x(3),linspace(0,1));
    nacaProfile = foilTransform(x(4:7),nacaProfile);
    nacaProfile = globalTransform(nacaProfile);
    line(nacaProfile(1,:),nacaProfile(2,:),repmat(sectionsZ(ind1),[1,size(nacaProfile,2)]),'color',[0 0 0])
    drawnow
    
    % Plot Curve alongside data used for fit
    subplot(1,3,3,'replace')
    nacaProfile = NACAProfile(x(1),x(2),x(3),linspace(0,1));
    nacaProfile = foilTransform(x(4:7),nacaProfile);
    line(secData4a(1,:),secData4a(2,:),'color',[1 0 0],'linestyle','none','marker','.','markersize',1)
    line(secData4(1,:),secData4(2,:),'color',[0 0 0],'linestyle','none','marker','.','markersize',1)
    line(nacaProfile(1,:),nacaProfile(2,:),'color',[0 0 0])
    axis equal
    grid on
    xlabel('X')
    ylabel('Y')
    title(sprintf('Data and Airfoil Fit at Z = %d',sectionsZ(ind1)))
    
end


%% Extract usable parameters

figure(3)
clf

thickness = sectionsX(1,:)';
camber = sectionsX(2,:)';
camberLoc = sectionsX(3,:)';

radius = sectionsZ';
twist = -rad2deg(sectionsX(6,:)') + rad2deg(atan2(Vy(2),Vy(1)));
chord = sectionsX(7,:)'*VxMag;
LECoordinates = globalTransform(sectionsX(4:5,:).*sectionsX(7,:));


subplot(3,1,1)
line(radius,twist,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('Twist (deg)')

subplot(3,1,2)
line(radius,chord,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('Chord (in)')

subplot(3,1,3)
line(radius,camber*100,'linestyle','none','marker','.','color',[0 0 0])
grid on
xlabel('Radius (in)')
ylabel('Camber (%)')

save('dataOutAutomatic','radius','twist','chord','LECoordinates','thickness','camber','camberLoc')



