function driveMap = createDriveMap(x0, y0, xf, yf, xcl, xcu, ycl, ycu)

x_vec = x0:0.2:xf;
y_vec = y0:0.2:yf;
map = zeros(length(x_vec), length(y_vec));
for i = 1:length(x_vec)
    for j = 1:length(y_vec)
        if x_vec(i) >= xcl && x_vec(i) <= xcu && y_vec(j) >= ycl && y_vec(j) <= ycu
            map(i, j) = 0; %not safe
        else
            map(i, j) = 1; %safe
        end
    end
end

driveMap.map = map;
driveMap.x_vec = x_vec;
driveMap.y_vec = y_vec;
% contourf(x_vec, y_vec, driveMap')

end