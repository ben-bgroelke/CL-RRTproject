function [x_traj, d2g, vcmd, xL, yL, d_trav] = stateTrajectoryNewTest(Vert, r_traj, dt, steps, vel_safe)
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
Td = 0.1;
Ta = 0.3;
vCH = 20;

% dt = 0.01;
L = 2.885;
lfan = 0.2;
tauv = 12;

Kp = 1.0;
Ki = 0.02;
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
Lfwmin = 3;
CLOSE_END = 0;
while ROUTE_END == 0
    i = i+1;
    [eta(i), Lfw(i), xL(i), yL(i), xq, yq] = findEta(x(i), y(i), lfan, rxf, ryf, x(i), y(i), vcmd(i), theta(i));
    if i == 1 && CLOSE_END == 0
        [D_vec, V_vec, CE_FLAG] = findVcmd(v(i), r_f, accel, decel, vmax, alpha0, alpha1, alpha2, tmin, Lfw(i), xL(i), yL(i), dt, v(1), coastin, x(i), y(i), vel_safe, CLOSE_END);
        D_orig = D_vec(1);
    elseif CLOSE_END == 0
        [D_vec, V_vec, CE_FLAG] = findVcmd(v(i), r_f, accel, decel, vmax, alpha0, alpha1, alpha2, tmin, Lfw(i), xL(i), yL(i), dt, v(1), coastin, x(i), y(i), vel_safe, CLOSE_END);
    end
%     [eta(i), Lfw(i), xL(i), yL(i)] = findEta(x(i), y(i), lfan, rxf, ryf, rx0, ry0, v(i), theta(i));
    CLOSE_END = CE_FLAG;
    delcmd(i) = -atan((L*sin(eta(i)))/((Lfw(i)/2) + lfan*cos(eta(i))));
    del(i+1) = del(i) + (dt/Td)*(delcmd(i) - del(i));
    if del(i+1) > 0.7435
        del(i+1) = 0.7435;
    elseif del(i+1) < -0.7435
        del(i+1) = -0.7435;
    end
    Gss = 1/(1 + (v(i)/vCH)^2);
    theta(i+1) = theta(i) + (dt*v(i)/L)*tan((del(i)+del(i+1))/2)*Gss;
    dist2go = Lfw(i) + sqrt((xL(i) - rxf)^2 + (yL(i) - ryf)^2) - Lfwmin;
%     d2g(i) = Lfw(i) + sqrt((xL(i) - rxf)^2 + (yL(i) - ryf)^2) - Lfwmin;
 
 d2g(i) = sqrt((x(i) - rxf)^2 + (y(i) - ryf)^2);

    %     disp(i)
%     if v(i) < 1 && D_orig - D_vec(1) < D_orig*0.1
%         vcmd(i+1) = v(i) + accel*dt;
%     else
    if i < 300
        vcmd(i+1) = interpone(D_vec, V_vec, d2g(i) - 1*(50/(i+25)), 'linear', 0);
    else
        vcmd(i+1) = interpone(D_vec, V_vec, d2g(i), 'linear', 0);
    end
%          vcmd(i+1) = interp1(D_vec, V_vec, dist2go, 'linear', 0);
%     end
%     [vcmd(i+1), d2g(i), vcoast(i), tstat1, tflat1] = findVcmd(v(i), r_traj, accel, decel, vmax, alpha0, alpha1, alpha2, tmin, Lfw(i), xL(i), yL(i), dt, v(1), coastin, tstat, tflat, x(i));

    
    error(i) = vcmd(i+1) - v(i);
    if i < 50
        integral = integral + error(i)*dt;
    else
        integral = sum(error(i-49:i).*dt);
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
%     if a(i+1) > 1.8
%         a(i+1) = 1.8;
%     elseif a(i+1) < -6
%         a(i+1) = 6;
%     end
    v(i+1) = v(i) + dt*((a(i+1) + a(i))/2);
%     v(i+1) = v(i) + dt*((vcmd(i+1) + vcmd(i))/2);
    x(i+1) = x(i) + dt*(((v(i) + v(i+1))/2)*cos((theta(i+1) + theta(i))/2));
    y(i+1) = y(i) + dt*(((v(i) + v(i+1))/2)*sin((theta(i+1) + theta(i))/2));
    if d2g(i) < 0.1 || i >= steps - 1
        ROUTE_END = 1;
        break;
    end
    if abs(d2g(1) - d2g(i))/d2g(1) > 0.5 && abs(d2g(i) - d2g(i-1))/d2g(i) <= 10E-7 && vcmd(i) < 0.5
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
x_traj.vcmd = vcmd;

d = hypot(diff(x_traj.x), diff(x_traj.y));                            % Distance Of Each Segment
d_trav = sum(d); 

% x_traj = [x; y; theta; del; v; a; d2g; vcmd; Lfw; xL; yL; vcoast];
end

function [eta, Lfw, xL, yL, xq, yq]= findEta(x, y, lfan, rxf, ryf, rx0, ry0, vcmd, theta)

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
    distp = sqrt((rxf - xdp)^2 + (ryf - ydp)^2);
    distm = sqrt((rxf - xdm)^2 + (ryf - ydm)^2);
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
    elseif yL == yq && xL > xq
        eta = 0;
    elseif yL == yq && xq > xL
        eta = -pi/2;
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

function [D_vec, Vcmd_vec, CE_FLAG]= findVcmd(v0, r_traj, accel, decel, vmax, alpha0, alpha1, alpha2, tmin, Lfw, xL, yL, dt, vstart, coastin, xx, yy, vel_safe, CLOSE_END)
%find distance D
Lfwmin = 3;
rxf = r_traj(1);
ryf = r_traj(2);
% D = Lfw + sqrt((xL - rxf)^2 + (yL - ryf)^2) - Lfwmin;
% find vcoast
  D = sqrt((xx - rxf)^2 + (yy - ryf)^2);
Dtest = ((vmax^2 - v0^2)/(2*accel)) + vmax*tmin ...
    + ((vmax^2)/(2*decel)) + alpha2*vmax^2 + alpha1*vmax + alpha0;

if Dtest < D && Dtest > 0
    vcoast = vmax;
    ts = (D - (((vmax^2 - v0^2)/(2*accel)) ...
        + ((vmax^2)/(2*decel)) + alpha2*vmax^2 + alpha1*vmax + alpha0))/vmax;
    
    d1 = D - (vcoast^2 - v0^2)/(2*accel);
    d2 = D - (vcoast^2 - v0^2)/(2*accel) - vcoast*ts;
    d3 = 0;
else
    A = (1/(2*accel) + alpha2 + 1/(2*decel));
    B = (tmin + alpha1);
    C = (alpha0 - ((v0^2)/(2*accel)) - D);
    vcoast = (-B + sqrt(B^2 - 4*A*C))/(2*A);
    ts = tmin;
    if v0 < vcoast
        d1 = D - (vcoast^2 - v0^2)/(2*accel);
        d2 = D - (vcoast^2 - v0^2)/(2*accel) - vcoast*ts;
        d3 = 0;
    else
        d2 = D - vcoast*ts;
        d3 = 0;
        d1 = (D + d2)/2;
    end
        
%         d1 = [];
%         d2 = D - vcoast*ts;
%         d3 = 0;
%     end
end
% if ~isempty(d1)
%     D_vec1 = linspace(D, d1, 100);
%     Vcmd_vec1 = linspace(v0, vcoast, 100);
%     D_vec2 = linspace(d1, d2, 100);
%     Vcmd_vec2 = vcoast*ones(1, 99);
%     D_vec3 = linspace(d2, d3, 100);
%     Vcmd_vec3 = linspace(vcoast,0,99);
%     
%     D_vec = fliplr(unique([D_vec1 D_vec2 D_vec3]));
%     Vcmd_vec = [Vcmd_vec1 Vcmd_vec2 Vcmd_vec3];
%     
%     D_vec1 = linspace(D, d1, 50);
%     Vcmd_vec1 = linspace(v0, vcoast, 50);
%     D_vec2 = linspace(d1 + 0.0001, d2, 49);
%     Vcmd_vec2 = vcoast*ones(1, 49);
%     D_vec3 = linspace(d2 + 0.0001, d3, 49);
%     Vcmd_vec3 = linspace(vcoast,0,49);
%     
%     D_vec = [D_vec1 D_vec2 D_vec3];
%     Vcmd_vec = [Vcmd_vec1 Vcmd_vec2 Vcmd_vec3];
    if D >= 3
        if v0 < vcoast
            D_vec1 = D:((d1 - D)/100):d1;
            Vcmd_vec1 = v0:((vcoast-v0)/(length(D_vec1) - 1)):vcoast;
        else
            D_vec1 = D:((d1 - D)/100):d1;
            Vcmd_vec1 = vcoast*ones(1, length(D_vec1));
        end
        D_vec2 = d1+0.001:((d2 - d1)/100):d2;
        Vcmd_vec2 = vcoast*ones(1, length(D_vec2));
        D_vec3 = d2+0.001:((d3-d2)/100):d3;
        Vcmd_vec3 = vcoast:((-vcoast)/(length(D_vec3) - 1)):vel_safe;
        
        D_vec = [D_vec1 D_vec2 D_vec3];
        Vcmd_vec = [Vcmd_vec1 Vcmd_vec2 Vcmd_vec3];
        CE_FLAG = 0;
    else
        D_vec = D:-D/50:0;
        Vcmd_vec = v0:(-v0/(length(D_vec) - 1)):vel_safe;
        Vcmd_vec(end-round(length(D_vec)*0.33):end) = vel_safe;
        CE_FLAG = 1;
    end
    
% else
%     test = 0;
%     D_vec1 = linspace(D, d2, 100);
%     Vcmd_vec1 = linspace(v0, vcoast, 100);
%     D_vec2 = linspace(d2, d3, 100);
%     Vcmd_vec2 = linspace(vcoast,0,99);
%     
%     D_vec = fliplr(unique([D_vec1 D_vec2]));
%     Vcmd_vec = [Vcmd_vec1 Vcmd_vec2];
% end
end



function return_val = interpone(X_data, Y_data, X_q, str, extrapval)
    if X_q >= X_data(end) && X_q <= X_data(1)
        [~, I] = min((X_q - X_data).^2);
        if X_data(I) < X_q
            x_ub = X_data(I);
            x_lb = X_data(I-1);
            ind_xub = I;
            ind_xlb = I-1;
        else
            x_lb = X_data(I);
            ind_xlb = I;
            if I+1 >= length(X_data)    % M is probably equal to X_q and also the max value 
                x_lb = X_data(I-1);
                x_ub = X_data(I);
                ind_xub = I;
                ind_xlb = I-1;
            else
                x_ub = X_data(I+1);
                ind_xub = I+1;
            end
        end 
        if length(Y_data) ~= length(X_data)
            see = 1;
        end
        
        y_0 = Y_data(ind_xlb);
        y_1 = Y_data(ind_xub);
        if isnan(y_0 + (X_q - x_lb)*((y_1 - y_0)/(x_ub - x_lb)))
            [C, IA, ~] = unique(X_data);
            return_val = interpone(C, Y_data(IA), X_q, str, extrapval);
        else
            return_val = y_0 + (X_q - x_lb)*((y_1 - y_0)/(x_ub - x_lb));
        end
    else
        return_val = extrapval;
    end
end

