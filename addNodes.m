function [Tadd, num] = addNodes(Root, r_traj, x_traj, d2g, vcmd, n_b, xL, yL, goalx, goaly)                     
%add edges to originating node with index of destination node;
%T.Vert(asc(i) already exists, but its edg to sx and sy does not
num = n_b;
inter = (d2g(1) - d2g(end))/n_b;     %   add nodes at intervals along x_traj
ind = zeros(1, n_b);
for i = 1:length(ind)
    [~, ind(i)] = min((d2g - inter*(i-1) - d2g(end)).^2);
end
%add Vertices
 i = n_b;
for count = 1:n_b
    Tadd.Vert(i).x = x_traj.x(ind(count));
    Tadd.Vert(i).y = x_traj.y(ind(count));
    Tadd.Vert(i).theta = x_traj.theta(ind(count));
    Tadd.Vert(i).del = x_traj.del(ind(count));
    Tadd.Vert(i).v = x_traj.v(ind(count));
    Tadd.Vert(i).a = x_traj.a(ind(count));   
    Tadd.Vert(i).rx = xL(ind(count));
    Tadd.Vert(i).ry = yL(ind(count));
%     Tadd.Vert(i).rx = x_traj.x(ind(count));
%     Tadd.Vert(i).ry = x_traj.y(ind(count));
    Tadd.Vert(i).rv = vcmd(ind(count));
    if i ~= n_b
        Tadd.Vert(i).safe = 1;
    else
        Tadd.Vert(i).safe = 1;
    end
     i = i - 1;
end
%add Edges
for i = 1:n_b
    Tadd.Edge(i).points = [];
    if i == 1
        Tadd.Edge(i).cost_lb = sqrt((Root.x - goalx)^2 + (Root.y - goaly)^2);
        Tadd.Edge(i).cost_ub = inf;
    else
        Tadd.Edge(i).cost_lb = sqrt((Tadd.Vert(i-1).x - goalx)^2 + (Tadd.Vert(i-1).y - goaly)^2);
        Tadd.Edge(i).cost_ub = inf;
    end
end






end