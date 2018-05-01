function driveMap = createDriveMapMaze()

x0 = -20;
y0 = -20;

xf = 150;
yf = 150;

x_vec = x0:0.2:xf;
y_vec = y0:0.2:yf;
map = zeros(length(x_vec), length(y_vec));

for i = 1:length(x_vec)
    for j = 1:length(y_vec)
        if x_vec(i) >= 20 && x_vec(i) <= 30 && y_vec(j) >= 20 && y_vec(j) <= 100
            map(i, j) = 0; %not safe
        elseif x_vec(i) >= 30 && x_vec(i) <= 90 && y_vec(j) >= 20 && y_vec(j) <= 30
            map(i, j) = 0; %not safe
        elseif x_vec(i) >= 45 && x_vec(i) <= xf && y_vec(j) >= 55 && y_vec(j) <= 65
            map(i, j) = 0; %not safe
        elseif x_vec(i) >= 30 && x_vec(i) <= 100 && y_vec(j) >= 90 && y_vec(j) <= 100
            map(i, j) = 0; %not safe
        elseif x_vec(i) >= x0 && x_vec(i) <=100  && y_vec(j) >= 120 && y_vec(j) <= yf
            map(i, j) = 0; %not safe
        elseif x_vec(i) >= 115 && x_vec(i) <=xf  && y_vec(j) >= 120 && y_vec(j) <= yf
            map(i, j) = 0; %not safe
        else
            map(i, j) = 1; %safe
        end
    end
end

driveMap.map = map;
driveMap.x_vec = x_vec;
driveMap.y_vec = y_vec;

end