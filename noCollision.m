function [bool] = noCollision(x_traj, driveMap)
diffx = driveMap.x_vec(1) - driveMap.x_vec(2);
diffy = driveMap.y_vec(1) - driveMap.y_vec(2);
upper_limitx = driveMap.x_vec(end);
upper_limity = driveMap.y_vec(end);
lower_limitx = driveMap.x_vec(1);
lower_limity = driveMap.y_vec(1);

theta = x_traj.theta;
w = 1.5;
l = 2.885;

booll = true;
for i = 1:length(x_traj.x)
    if x_traj.x(i) >= lower_limitx && x_traj.x(i) <= upper_limitx && x_traj.y(i) >= lower_limity && x_traj.y(i) <= upper_limity
        [~,Ix1] = min((driveMap.x_vec - (x_traj.x(i) - w*sin(theta(i)))).^2);
        [~,Iy1] = min((driveMap.y_vec - (x_traj.y(i) + w*cos(theta(i)))).^2);
        
        [~,Ix2] = min((driveMap.x_vec - (x_traj.x(i) + w*sin(theta(i)))).^2);
        [~,Iy2] = min((driveMap.y_vec - (x_traj.y(i) - w*cos(theta(i)))).^2);
        
        [~,Ix3] = min((driveMap.x_vec - (x_traj.x(i) + l*cos(theta(i)) - w*sin(theta(i)))).^2);
        [~,Iy3] = min((driveMap.y_vec - (x_traj.y(i) + l*sin(theta(i)) + w*cos(theta(i)))).^2);
        
        [~,Ix4] = min((driveMap.x_vec - (x_traj.x(i) + l*cos(theta(i)) + w*sin(theta(i)))).^2);
        [~,Iy4] = min((driveMap.y_vec - (x_traj.y(i) + l*sin(theta(i)) - w*cos(theta(i)))).^2);
%         disp(i)
        if driveMap.map(Ix1, Iy1) == 0 || driveMap.map(Ix2, Iy2) == 0 || driveMap.map(Ix3, Iy3) == 0 || driveMap.map(Ix4, Iy4) == 0 %not safe
            booll = false;
        end
    else
        booll = false;
    end
end
bool = booll;
end