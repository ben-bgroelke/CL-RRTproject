function driveMap = createDriveMapU()

x0 = -10;
y0 = -10;

xf = 150;
yf = 150;

x_vec = x0:0.2:xf;
y_vec = y0:0.2:yf;
map = zeros(length(x_vec), length(y_vec));

for i = 1:length(x_vec)
    for j = 1:length(y_vec)
        if x_vec(i) >= 40 && x_vec(i) <= 60 && y_vec(j) >= 30 && y_vec(j) <= 35
            map(i, j) = 0; %not safe
        elseif x_vec(i) >= 40 && x_vec(i) <= 45 && y_vec(j) >= 30 && y_vec(j) <= 50
            map(i, j) = 0; %not safe
        elseif x_vec(i) >= 40 && x_vec(i) <= 60 && y_vec(j) >= 45 && y_vec(j) <= 50
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