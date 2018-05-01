function [Tfin] = buildRRT(q_init, K, veh_param, driveMap, goalx, goaly, x_ref, y_ref, theta_ref, sig_r, sig_theta, r_0)
%Each node in the CL-RRT stores the states of the vehicle and the controler
%input

T.Vert = [];
T.Edges = [];     %edges are represented as two row incdices of T.Vert
% x_ref = q_init(1);
% y_ref = q_init(2);
% theta_ref = q_init(3);
% theta_ref = q_init(3);
% sig_r = 20.*ones(1,K);
% sig_theta = pi/10.*ones(1,K);
% r_0 = 20.*ones(1,K);
% r_0((K/2):end) = 60;
vel_safe = 0;

T = initialize(q_init, goalx, goaly);

figure(1); hold on;
% rectangle('Position', [40 30 5 15], 'FaceColor', [0 0 0], 'EdgeColor', 'k');
% axis([-10 100 -10 100]);
% hold on
% mycolormap = jet(K); hold on;
contourf(driveMap.x_vec, driveMap.y_vec, driveMap.map', [0 1]);
hold on
colormap(gray);
mycolormap = jet(K); hold on;
for i = 1:K
    [sx, sy] = randomSample(x_ref(i), y_ref(i), theta_ref(i), 'move', sig_r(i), sig_theta(i), r_0(i));
    Tnew = extendRRT(T, sx, sy, driveMap, goalx, goaly, veh_param, i, mycolormap, vel_safe);
    T = Tnew;
    disp(i)
end
Tfin = T;
end

function T = initialize(q_init, goalx, goaly)
    T.Vert(1).x = q_init(1);
    T.Vert(1).y = q_init(2);
    T.Vert(1).theta = q_init(3);
    T.Vert(1).del = 0;
    T.Vert(1).v = 0;
    T.Vert(1).a = 0;
    T.Vert(1).safe = 0;
    T.Vert(1).rx = 0;
    T.Vert(1).ry = 0;
    T.Vert(1).rv = 0;
    T.Edge(1).points = [];
    T.Edge(1).cost_lb = sqrt((q_init(1) - goalx)^2 + (q_init(2) - goaly)^2);
    T.Edge(1).cost_ub = inf;
end