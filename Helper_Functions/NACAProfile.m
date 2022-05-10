function [profilePoints] = NACAProfile(t,m,p,xSample)
% Produces profile points of a 4 digit naca foil
% mptt
% m Maximum camber in, (m/100)% of chord 
% p Maximum Camber location, (p/10)% of chord

if nargin == 4
    x = xSample;
elseif nargin == 3
    x = linspace(0,1);
end

yt = 5*t.*(.2969.*sqrt(x) - 0.1260.*x - 0.3516.*x.^2 + 0.2843*x.^3 -.1015.*x.^4);

yc(x < p) = m./p.^2.*(2.*p.*x(x < p) - x(x < p).^2);
yc(x >= p) = m./(1-p).^2.*((1 - 2*p) + 2.*p.*x(x >= p) - x(x >= p).^2);

theta(x<p) = atan(2*m./p.^2.*(p - x(x<p)));
theta(x>=p) = atan(2*m./(1-p).^2.*(p-x(x>=p)));

xu = x - yt.*sin(theta);
xl = x + yt.*sin(theta);
yu = yc + yt.*cos(theta);
yl = yc - yt.*cos(theta);

profilePoints = [xu flip(xl);yu flip(yl)];

end

