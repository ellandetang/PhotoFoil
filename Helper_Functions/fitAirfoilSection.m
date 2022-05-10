function [x] = fitAirfoilSection(dataIn,x0)
% x0 Are initial parameter guesses based on priors
% dataIn is a 2xn matrix of raw coordinate data points roughly set up with
% leading edge near the origin, trailing edge near [1,0] and up in the
% positive y direction
% x are the parameters out

% Parameters are [thickness,camber, camber location, x displacement, y displacement, rotation, scale]

xSample1 = linspace(0,.01,11);
xSample2 = linspace(.01,1,90);
xSample = [xSample1(1:end-1),xSample2];

constants.xSample = xSample;
constants.data = dataIn;

A = [];
b = [];
Aeq = [];
beq = [];

if nargin == 2
    
    lb = [.01,0,.3, x0(4)-.05, x0(5)-.05, x0(6) - deg2rad(3), x0(7)*.8]';
    ub = [.2,.1,.7, x0(4)+.05, x0(5)+.05, x0(6) + deg2rad(3), x0(7)*1.2]';
    
else
    
    % x0 = [.12,0,0,0,0,0,1]'; % NACA 0012
    x0 = [.04,.055,.5,0,0,0,1]'; % E63 approximate
    lb = [.01,0,.3,-.1,-.1,deg2rad(-3),.8]';
    ub = [.2,.1,.7,.1,.1,deg2rad(3),1.2]';
    
end

options = optimoptions('fmincon','Display','off');

[x,~,exitflag] = fmincon(@(x) fun(x,constants),x0,A,b,Aeq,beq,lb,ub,[],options);
if (exitflag ~= 1) && (exitflag ~= 2)
    keyboard
end
end


function [cost] = fun(x,constants)
t = x(1);
m = x(2);
p = x(3);
dx = x(4);
dy = x(5);
theta = x(6);
scale = x(7);

nacaProfile = NACAProfile(t,m,p,constants.xSample);

nacaProfile = ([cos(theta) -sin(theta);
    sin(theta) cos(theta)]*nacaProfile + repmat([dx,dy]',[1,size(nacaProfile,2)]))*scale;

[dataDistance] = contourDistance(nacaProfile,constants.data);

cost = sum(dataDistance(~isnan(dataDistance)).^2);

end
