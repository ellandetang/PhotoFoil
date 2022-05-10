function [x] = fitAirfoilParams(dataIn,x0)
% Fit thickness, camber, and camber location from pre-aligned data
% x0 Are initial parameter guesses based on priors
% dataIn is a 2xn matrix of raw coordinate data points roughly set up with
% leading edge near the origin, trailing edge near [1,0] and up in the
% positive y direction
% x are the parameters out

% Parameters are [thickness, camber, camber location]

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
    
    lb = [.01,0,.3]';
    ub = [.2,.1,.7]';
    
else
    
    % x0 = [.12,0,0]'; % NACA 0012
    x0 = [.04,.055,.5]'; % E63 approximate
    lb = [.01,0,.3]';
    ub = [.2,.1,.7]';
    
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

nacaProfile = NACAProfile(t,m,p,constants.xSample);

[dataDistance] = contourDistance(nacaProfile,constants.data);

cost = sum(dataDistance(~isnan(dataDistance)).^2);

end
