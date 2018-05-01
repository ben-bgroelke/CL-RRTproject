clear
load('BigTreeMaze2_500.mat')
goalx = 107.5;
goaly = 140;
STOP_FLAG = 0;
max_steps = 100000;
idx = 1;
count = 0;
while STOP_FLAG == 0 && count < max_steps
    count = count + 1;
    if T.Edge(idx(count)).cost_ub == 0
        STOP_FLAG = 1;
        break;
    end
    smallest = inf;
    for i = 1:length(T.Edge(idx(count)).cost_ub)
        if T.Edge(idx(count)).cost_ub(i) < smallest
            smallest = T.Edge(idx(count)).cost_ub(i);
            ndex = i;
        end
    end
%     [~, I] = min(T.Edge(idx).cost_ub);
     idx(count+1) = T.Edge(idx(count)).points(ndex);
end

sx_vec = zeros(1, length(idx));
sy_vec = zeros(1, length(idx));
sv_vec = zeros(1, length(idx));

for i = 1:length(idx) + 1
    if i < length(idx) + 1
        sx_vec(i) = T.Vert(idx(i)).x;
        sy_vec(i) = T.Vert(idx(i)).y;
        sv_vec(i) = T.Vert(idx(i)).v;
    else
        sx_vec(i) = goalx;
        sy_vec(i) = goaly;
        sv_vec(i) = 0;
    end
end
openfig('fig_Maze2_500'); hold on;
for i = 2:length(sx_vec)
    line([sx_vec(i-1) sx_vec(i)], [sy_vec(i-1) sy_vec(i)], 'Color', 'k', 'LineWidth', 3);
    hold on;
end
