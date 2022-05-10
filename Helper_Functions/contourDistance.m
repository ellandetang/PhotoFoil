function [dataDistance] = contourDistance(profileData,sampleData)
% profile: 2xn datapoints representing the contour
% Data should have the same beginning and end and should for a closed
% contour. Contour should also be from an airfoil profile with the
% leading edge at the origin and the trailing edge at [1,0]' with up in
% the positive y direction.
% data: 2xm pointcloud function will assess its distance from the contour
% distance: distance from each of the datapoints to the contour.
% Points outside the contour criteria will have nan as the value

% Rotation matrix
rot = @(in) [cos(in) -sin(in);
    sin(in) cos(in)];

% Angle of a vector
angle = @(in) atan2(in(2,2) - in(2,1),in(1,2) - in(1,1));

% reset Vector to new origin based on indices
vZero = @(z,in) in - repmat(z,[1,size(in,2)]);


% Run through each segment and compile which points are on which side of
% the bisector
contourCoordinates = [profileData(:,end-1) profileData ];

categoryMap = false(size(profileData,2)-1,size(sampleData,2));
distanceMap = zeros(size(categoryMap));

for ind1 = 1:size(contourCoordinates,2) - 2
    
    dataSample = contourCoordinates(:,(0:2)+ ind1);
    
    th1 = angle(dataSample(:,1:2));
    th2 = angle(dataSample(:,2:3));
    
    % Catch the 180 deg to -180 deg case
    if (sign(th1) ~= sign(th2)) && (abs(th1) > pi/2)
        if th1 < 0
            th1 = th1 + 2*pi;
        elseif th2 < 0
            th2 = th2 + 2*pi;
        end
    end
    
    rotM = rot(-((th2 + th1)/2 - pi/2));
    categoryMap(ind1,:) = (rotM(2,:)*vZero(dataSample(:,2),sampleData)) >= 0;
    
    rotLine = rot(-th2);
    distanceMap(ind1,:) = abs(rotLine(2,:)*vZero(dataSample(:,2),sampleData));
    
end


% Find which points are within the bounds of each panel
panelMap = false(size(categoryMap));

categoryMap = [categoryMap;categoryMap(1,:)];

for ind2 = 1:size(categoryMap,1)-1
    %     panelMap(ind2,:) = categoryMap(ind2,:) & ~categoryMap(ind2+1,:) & (distanceMap(ind2,:) <= .02);
    panelMap(ind2,:) = categoryMap(ind2,:) & ~categoryMap(ind2+1,:);
end

dataDistance = zeros(1,size(sampleData,2));
for ind4 = 1:size(panelMap,2)
    if isempty(find(panelMap(:,ind4), 1))
        dataDistance(ind4) = nan;
    else
        dataDistance(ind4) = min(distanceMap(panelMap(:,ind4),ind4));
    end
end


end

