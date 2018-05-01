function T_new = extendRRT(T, sx, sy, driveMap, goalx, goaly, veh_param, colind, colormap, vel_safe)
n_b = 3;
asc = sortClosest(T,sx,sy, veh_param(9));
T_new = T;
T_n0 = T;
dt = 0.01;
steps = 10000;
% figure(1)
% contourf(driveMap.x_vec, driveMap.y_vec, driveMap.map');
% hold on
index_final = min(length(asc), 25);
for i = 1:index_final %length(asc)
    r_traj = formRefNew(T.Vert(asc(i)), sx, sy);
    [x_traj, d2g, vcmd, xL, yL, dist_trav] = stateTrajectoryNewTest(T.Vert(asc(i)), r_traj, dt, steps, vel_safe);
    [Tadd, num] = addNodes(T.Vert(asc(i)), r_traj, x_traj, d2g, vcmd, n_b, xL, yL, goalx, goaly);
    if asc(i) == 1
        rrv = 1;
    else
        rrv = T.Vert(asc(i)).rv;
    end
    if noCollision(x_traj, driveMap) && sqrt((x_traj.x(end) - sx)^2 + (x_traj.y(end) - sy)^2) < 1 && dist_trav < 2*sqrt((x_traj.x(1) - sx)^2 + (x_traj.y(1) - sy)^2) && rrv > 0
        T_new.Vert(asc(i)).safe = 1;        %make root safe
        if isempty(T_new.Edge(asc(i)).points)     %add edges to root
            T_new.Edge(asc(i)).points = length(T_n0.Vert) + 1;
            T_new.Edge(asc(i)).cost_ub = inf;
        else
            T_new.Edge(asc(i)).points = [T_new.Edge(asc(i)).points length(T_new.Vert)+1];
            T_new.Edge(asc(i)).cost_ub = [T_new.Edge(asc(i)).cost_ub Tadd.Edge(1).cost_ub];
        end
        T_new.Edge(asc(i)).cost_lb = Tadd.Edge(1).cost_lb;        %already has an edge
%         T_new.Edge(asc(i)).cost_ub = Tadd.Edge(1).cost_ub;
        for j = 1:n_b       %add rest of edges
            T_new.Vert(length(T_n0.Vert) + j) = Tadd.Vert(j);
            if j > 1
                T_new.Edge(length(T_n0.Vert) + j - 1).cost_lb = Tadd.Edge(j).cost_lb;
                T_new.Edge(length(T_n0.Vert) + j - 1).cost_ub = Tadd.Edge(j).cost_ub;
                T_new.Edge(length(T_n0.Vert) + j - 1).points = length(T_n0.Vert) + j;
            end
            if j == n_b
                T_new.Edge(length(T_n0.Vert) + j).points = [];
                T_new.Edge(length(T_n0.Vert) + j).cost_lb = Tadd.Edge(j).cost_lb;
            end
        end
        break
        %     else
        %         r_traj = formRefNew(T.Vert(asc(i)), Tadd.Vert(end-1).x, Tadd.Vert(end-1).y);
        %         x_traj = stateTrajectoryNewTest(T.Vert(asc(i)), r_traj, dt, steps);
        %         [Tadd2, num] = addNodes(T.Vert(asc(i)), r_traj, x_traj, Tadd.Vert(end-1).x, Tadd.Vert(end-1).x, n_b - 1);
        %         if noColliion(x_traj, driveMap)
        %             [Tadd2, num] = markUnsafe(Tadd2);
        %             for j = 1:n_b
        %                 %make root not safe
        %                 T_new.Vert(length(T_new.Vert) + j) = Tadd2.Vert(j);
        %                 T_new.Edge(length(T_new.Vert) + j) = Tadd2.Edge(j);
        %                 Tadd = Tadd2;
        %             end
        %             break
        %         end
        %     end
    else
        Tadd = [];
    end
end

    
if ~isempty(Tadd)
    plot(x_traj.x, x_traj.y, 'Color', colormap(colind,:));
    hold on
    for j = 1:num
        plot(Tadd.Vert(j).x, Tadd.Vert(j).y, 'Color', colormap(colind,:), 'Marker', 'o', 'MarkerSize', 6);
        hold on
        plot(sx, sy, 'Color', colormap(colind,:), 'Marker', '*', 'MarkerSize', 8);
        hold on
    end
    for i = 1:num
        r_traj_goal = formRefNew(Tadd.Vert(i), goalx, goaly);
        [x_traj_goal, d2g, vcmd, xL, yL, D_orig] = stateTrajectoryNewTest(Tadd.Vert(i), r_traj_goal, dt, steps, vel_safe);
        [Tgoal, ~] = addNodes(Tadd.Vert(i), r_traj_goal, x_traj_goal, d2g, vcmd, 1, xL, yL, goalx, goaly);
        if noCollision(x_traj_goal, driveMap)  && sqrt((x_traj_goal.x(end) - goalx)^2 + (x_traj_goal.y(end) - goaly)^2) < 1 && dist_trav < 2*sqrt((x_traj_goal.x(1) - goalx)^2 + (x_traj_goal.y(1) - goaly)^2)
            indd = [];
            for j = 1:length(T_new.Vert)
                if min(T_new.Edge(j).cost_ub) == 0
                    indd = j;
                end
            end
            
            if isempty(indd)
                index = length(T_new.Vert) + 1;
                T_new.Vert(index) = Tgoal.Vert(1);
                T_new.Edge(index).points = [];
                T_new.Edge(index).cost_lb = 0;
                T_new.Edge(index).cost_ub = 0;
                goal_ind = index;
            else
                goal_ind = indd;
            end
            
            T_new.Edge(length(T_n0.Vert) + i).points = [T_new.Edge(length(T_n0.Vert) + i).points goal_ind];
            T_new.Edge(length(T_n0.Vert) + i).cost_ub = [T_new.Edge(length(T_n0.Vert) + i).cost_ub D_orig];
            index2 = length(T_n0.Vert) + i;
            BCK_ROOT = 0;
            
            while BCK_ROOT == 0
                count = 0 ;
                for k = 1:length(T_new.Edge)
                    if isempty(find(T_new.Edge(k).points == index2,1))
                        r=1;
                    else
                        count = count + 1;
                        index2next(count) = k;
                        xpar = T_new.Vert(k).x;
                        ypar = T_new.Vert(k).y;
                        xn = T_new.Vert(index2).x;
                        yn = T_new.Vert(index2).y;
                        T_new.Edge(k).cost_ub(find(T_new.Edge(k).points == index2,1)) = min(T_new.Edge(index2).cost_ub) + sqrt((xn - xpar)^2+(yn - ypar)^2);
                    end
                end
                if index2next == 1
                    BCK_ROOT = 1;
                else
                    
                    if length(index2next) > 1
                        see = 1;
                    else
                        index2 = index2next;
                    end
                end
            end
        end
    end
end

end




