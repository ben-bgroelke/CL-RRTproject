function [sx, sy] = randomSample(x0, y0, theta_ref, str, sig_r, sig_theta, r_0)

% sig_r = 5;
% sig_theta = 0.2*pi;
% r_0 = 3;
theta_0 = theta_ref;
park_zone = [50 pi 3 0.44*pi];

if str == 'park' | str == 'Park'
    sig_r = park_zone(1);
    sig_theta = park_zone(2);
    r_0 = park_zone(3);
    theta_0 = parke_zone(4);
    
    nr = normrnd(0,1);
    nt = normrnd(0,1);
    r = sig_r*abs(nr)+r_0;
    theta = sig_theta*nt + theta_0;
    sx = x0 + r*cos(theta);
    sy = y0 + r*sin(theta);
else
    nr = normrnd(0,1);
    nt = normrnd(0,1);
    r = sig_r*abs(nr)+r_0;
    theta = sig_theta*nt + theta_0;
    sx = x0 + r*cos(theta);
    sy = y0 + r*sin(theta);
end

end