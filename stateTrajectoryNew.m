function [x_traj, d2g, vcmd, xL, yL] = stateTrajectoryNew(Vert, r_traj, dt, steps)
x0 = Vert.x;
y0 = Vert.y;
theta0 = Vert.theta;
del0 = Vert.del;
v0 = Vert.v;
a0 = Vert.a;

rx0 = Vert.rx;
ry0 = Vert.ry;
rv0 = Vert.rv;
rxf = r_traj(1) + Vert.rx;
ryf = r_traj(2) + Vert.ry;
accel = 1.0;
decel = 2.5;
alpha0 = -0.5347;
alpha1 = 1.2344;
alpha2 = -0.0252;
% alpha0 = 0;
% alpha1 = 0;
% alpha2 = 0;
vmax = 11.176;
tmin = 1;
Td = 0.3;
Ta = 0.3;
vCH = 20;

% dt = 0.01;
L = 2.885;
lfan = 0.1;
tauv = 12;

Kp = 1.25;
Ki = 0.04;
% steps = 500000;
x = zeros(1, steps); y = zeros(1, steps); theta = zeros(1, steps);
v = zeros(1, steps); a = zeros(1, steps); u = zeros(1, steps);
eta = zeros(1, steps); del = zeros(1, steps); Lfw = zeros(1, steps); 
xL = zeros(1, steps); yL = zeros(1, steps);d2g = zeros(1, steps);
vcmd = zeros(1, steps); delcmd = zeros(1, steps); vcoast = zeros(1, steps);
error = zeros(1, steps); 
%initialize states
x(1) = x0; y(1) = y0; theta(1) = theta0; del(1) = del0; v(1) = v0; a(1) = a0;
vcmd(1) = rv0;

ROUTE_END = 0;
i = 0;
integral = 0;
coastin = 0;
tstat = 0;
tflat = 0;
r_f = [rxf ryf];
while ROUTE_END == 0
    i = i+1;
    [eta(i), Lfw(i), xL(i), yL(i)] = findEta(x(i), y(i), lfan, rxf, ryf, rx0, ry0, vcmd(i), theta(i));
%     [eta(i), Lfw(i), xL(i), yL(i)] = findEta(x(i), y(i), lfan, rxf, ryf, rx0, ry0, v(i), theta(i));
    delcmd(i) = -atan((L*sin(eta(i)))/((Lfw(i)/2) + lfan*cos(eta(i))));
    del(i+1) = del(i) + (dt/Td)*(delcmd(i) - del(i));
    if del(i+1) > 0.5435
        del(i+1) = 0.5435;
    elseif del(i+1) < -0.5435
        del(i+1) = -0.5435;
    end
    Gss = 1/(1 + (v(i)/vCH)^2);
    theta(i+1) = theta(i) + (dt*v(i)/L)*tan((del(i)+del(i+1))/2)*Gss;
    
    [vcmd(i+1), d2g(i), vcoast(i), tstat1, tflat1] = findVcmd(vcmd(i), r_f, accel, decel, vmax, alpha0, alpha1, alpha2, tmin, Lfw(i), xL(i), yL(i), dt, v(1), coastin, tstat, tflat, x(i));
%     [vcmd(i+1), d2g(i), vcoast(i), tstat1, tflat1] = findVcmd(v(i), r_traj, accel, decel, vmax, alpha0, alpha1, alpha2, tmin, Lfw(i), xL(i), yL(i), dt, v(1), coastin, tstat, tflat, x(i));
    coastin = vcoast(1);
    tstat = tstat1;
    tflat = tflat1;
    
    error(i) = vcmd(i+1) - v(i);
    if i < 100
        integral = integral + error(i)*dt;
    else
        integral = sum(error(i-99:i).*dt);
    end
    u(i) = Kp*error(i) + Ki*integral;
%     
%     syms s t
%     UtoV = ilaplace((0.1013*v(i)^2+0.5788*v(i)+49.1208)/(tauv*s + 1));
%     UtoA = diff(UtoV,t);
%     t = 0;
%     ac = subs(-UtoA);
%     
%     ac = (1/12)*(u(i)*(0.1013*v(i)^2+0.5788*v(i)+49.1208) - v(i));
    
    a(i+1) = a(i) + (dt/Ta)*(u(i) - a(i));
    if a(i+1) > 1.8
        a(i+1) = 1.8;
    elseif a(i+1) < -6
        a(i+1) = 6;
    end
    v(i+1) = v(i) + dt*((a(i+1) + a(i))/2);
%     v(i+1) = v(i) + dt*((vcmd(i+1) + vcmd(i))/2);
    x(i+1) = x(i) + dt*(((v(i) + v(i+1))/2)*cos((theta(i+1) + theta(i))/2));
    y(i+1) = y(i) + dt*(((v(i) + v(i+1))/2)*sin((theta(i+1) + theta(i))/2));
    if d2g(i) < 0.1 || i >= steps - 1
        ROUTE_END = 1;
        break;
    end
    if abs(d2g(1) - d2g(i))/d2g(1) > 0.01 && abs(d2g(i) - d2g(i-1))/d2g(i) <= 10E-7
        buttr = 0;
        break
    end
    
end
x = x(1:i); y = y(1:i); theta = theta(1:i); del = del(1:i); 
v = v(1:i); a = a(1:i); d2g = d2g(1:i); vcmd = vcmd(1:i); Lfw = Lfw(1:i);
xL = xL(1:i); yL = yL(1:i); vcoast = vcoast(1:i);

x_traj.x = x;
x_traj.y = y;
x_traj.theta = theta;
x_traj.del = del;
x_traj.v = v;
x_traj.a = a;


% x_traj = [x; y; theta; del; v; a; d2g; vcmd; Lfw; xL; yL; vcoast];
end

function [eta, Lfw, xL, yL]= findEta(x, y, lfan, rxf, ryf, rx0, ry0, vcmd, theta)

% Lfw as function of v
if vcmd < 1.34
    Lfw = 3;
elseif vcmd >= 1.34 && vcmd < 5.36
    Lfw = 2.24*vcmd;
else
    Lfw = 12;
end

m = (ryf - ry0)/(rxf - rx0);
if m ~= inf
    xq = x + lfan*cos(theta);
    yq = y + lfan*sin(theta);
    c = -m*rx0 + ry0 - yq;
    D = xq^2 + c^2 - Lfw^2;
    xdp = (-(2*m*c - 2*xq) + sqrt((2*m*c - 2*xq)^2 - 4*(1+m^2)*D))/(2*(1+m^2));
    xdm = (-(2*m*c - 2*xq) - sqrt((2*m*c - 2*xq)^2 - 4*(1+m^2)*D))/(2*(1+m^2));
    ydp = m*xdp - m*rx0 + ry0;
    ydm = m*xdm - m*rx0 + ry0;
    distp = sqrt((5*rxf - xdp)^2 + (5*ryf - ydp)^2);
    distm = sqrt((5*rxf - xdm)^2 + (5*ryf - ydm)^2);
    if distp > distm
        xL = xdm;
        yL = ydm;
    else
        xL = xdp;
        yL = ydp;
    end
    
    if xL ~= xq && yL ~=yq
        if xq < xL
            eta = theta - atan((yL - yq)/(xL - xq));
        else
            eta = -1*(90*pi/180  + (90*pi/180 - atan((yL - yq)/(xq - xL))) - theta);
        end
    elseif xL == xq
        eta = theta - pi/2;
%     else       
    end
    
else
    xq = x + lfan*cos(theta);
    yq = y + lfan*sin(theta);
    if ry0 == ryf
        xL = xq + Lfw;
        yL = 0;
    else
        xL = 0;
        yL = sqrt(Lfw^2 - (xq^2))+yq;
    end
    if xL ~= xq && yL ~=yq
        if xq < xL
            eta = theta - atan((yL - yq)/(xL - xq));
        else
            eta = -1*(90*pi/180  + (90*pi/180 - atan((yL - yq)/(xq - xL))) - theta);
        end
    elseif xL == xq
        eta = theta - pi/2;
%     else       
    end
end

end

function [vcmd, D, vcoast, tstat, tflat]= findVcmd(v0, r_traj, accel, decel, vmax, alpha0, alpha1, alpha2, tmin, Lfw, xL, yL, dt, vstart, coastin, tstat, tflat,xv)
%find distance D
Lfwmin = 3;
rxf = r_traj(1);
ryf = r_traj(2);
D = Lfw + sqrt((xL - rxf)^2 + (yL - ryf)^2) - Lfwmin;
% find vcoast

Dtest = ((vmax^2 - v0^2)/(2*accel)) + vmax*tmin ...
    + ((vmax^2)/(2*decel)) + alpha2*vmax^2 + alpha1*vmax + alpha0;

if Dtest < D && Dtest > 0
    vcoast = vmax;
    ts = (D - (((vmax^2 - v0^2)/(2*accel)) ...
        + ((vmax^2)/(2*decel)) + alpha2*vmax^2 + alpha1*vmax + alpha0))/vmax;
else
    A = (1/(2*accel) + alpha2 + 1/(2*decel));
    B = (tmin + alpha1);
    C = (alpha0 - ((v0^2)/(2*accel)) - D);
    vcoast = (-B + sqrt(B^2 - 4*A*C))/(2*A);
    ts = tmin;
end
if v0 == vstart
    coastin = vcoast;
    tflat = ts;
end
%
% if tflat > tmin
%     x1end = ((vmax^2 - vstart^2)/(2*accel));
%     x2end = ((vmax^2 - vstart^2)/(2*accel)) + vmax*tflat;
%     x3end = ((vmax^2 - vstart^2)/(2*accel)) + vmax*tflat ... 
%     + ((vmax^2)/(2*decel)) + alpha2*vmax^2 + alpha1*vmax + alpha0;
% else
%     x1end = ((coastin^2 - vstart^2)/(2*accel));
%     x2end = ((coastin^2 - vstart^2)/(2*accel)) + coastin*tflat;
%     x3end = ((coastin^2 - vstart^2)/(2*accel)) + coastin*tflat ... 
%     + ((coastin^2)/(2*decel)) + alpha2*coastin^2 + alpha1*coastin + alpha0;
% end

if v0 < vcoast && tstat == 0
    vcmd = v0 + accel*dt;
elseif tstat <= tflat
    tstat = tstat + dt;
    vcmd = v0;
else
    vcmd = v0 - decel*dt;
end
% if v0 < vcoast
%     vcmd = v0 + accel*dt;
% else
%     vcmd = v0 - decel*dt;
% end

if vcmd < 0
    vcmd = 0;
end


% if xv < x1end
%     vcmd = accel;
% elseif xv >= x1end && xv < x2end
%     vcmd = 0;
%     tstat = tstat + dt;
% elseif xv >= x2end && xv < x3end
%     vcmd = -decel;
% else
%     vcmd = 0;
% end
% if v0 + vcmd*dt < 0
%     vcmd = 0;
% end

end









