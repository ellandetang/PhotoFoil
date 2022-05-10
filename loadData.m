% loads appropriate data for subsequent analyses

addpath('Helper_Functions')

%%
if ~exist('cloudData.mat','file')
    [cameras, images, points3D] = read_model('./Dense/');
    
    n = length(points3D);
    
    cloudPositions = zeros(n,3);
    cloudColors = zeros(n,3);
    
    for ind1 = 1:n
        
        cloudPositions(ind1,:) = points3D(ind1).xyz;
        cloudColors(ind1,:) = points3D(ind1).rgb;
        
    end
    
    save('cloudData','cloudPositions','cloudColors')
else
    load('cloudData.mat')
end

