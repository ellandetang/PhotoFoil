function [sTrans,M,x_opt] = scalepoint2plane(sList,dList,nList,sd_map)
% Calculate point-to-plane fit with scaling
% Inputs:
%   sList: 3xN source point cloud
%   dList: 3xM destination point list
%   nList: 3xM destination surface normals
%   sd_map: 1xN correspondence map between source and destination
% Outputs:
%   sTrans: 3xN The fit dataset
%   M: 4x4 augmented transformation matrix
%   x_opt: 7x1 transformation parameters

rotX = @(th) [1 0 0
    0 cos(th) -sin(th)
    0 sin(th) cos(th)];
rotY = @(th) [cos(th) 0 sin(th)
    0 1 0
    -sin(th) 0 cos(th)];
rotZ = @(th) [cos(th) -sin(th) 0;
    sin(th) cos(th) 0 ;
    0 0 1];

R = @(alpha,beta,gamma) blkdiag((rotZ(gamma)*rotY(beta)*rotX(alpha)),1);
T = @(t) [eye(4,3),[t;1]];
S = @(l) diag([l l l 1]);

d = dList(:,sd_map);
n = nList(:,sd_map);
s = sList;

b = dot(n,d,1)';
a = cross(s,n,1);
A = [a' n' dot(n,s,1)'];

x_opt = A\b;

x_opt(1:3) = x_opt(1:3)./x_opt(7);

M = T(x_opt(4:6))*R(x_opt(1),x_opt(2),x_opt(3))*S(x_opt(7));

sTrans = M*[sList;ones(1,size(sList,2))];
sTrans = sTrans(1:3,:);

end

