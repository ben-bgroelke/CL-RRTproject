clear;
num_samples = 10;
numparams = 9; 
veh_param = zeros(1, numparams);
rho = 4.77;
veh_param(9) = rho;        %m - minimum turning radius RTMP
q_init = [0 0 45*pi/180];
goalx = 107.5;
goaly = 140;

xf = 100;
yf = 100;
xx0 = -10;
yy0 = -10;
xl = [40 45];
yl = [30 45];
% driveMap = createDriveMap(xx0, yy0, xf, yf, xl(1), xl(2), yl(1), yl(2));
driveMap = createDriveMapMaze();
K = 500;
change = 5;
dk = K/change;
x_ref = [0 0 0 0 107.5];
y_ref = [0 0 0 110 100]; 
theta_ref = [pi/2 pi/6 pi/4 0  pi/2];
sig_r = [70 80 50 100 20];
sig_theta = [pi/20 pi/12 pi/8 pi/24 pi/4];
r_0 = [10 5 40 5 2];
count = 1;
for i = 1:K
   if mod(i - 1, dk) == 0 && i>2
       count = count+1;
   end
   x_ref_arr(i) = x_ref(count);
   y_ref_arr(i) = y_ref(count);
   theta_ref_arr(i) = theta_ref(count);
   sig_r_arr(i) = sig_r(count);
   sig_theta_arr(i) = sig_theta(count);
   r_0_arr(i) = r_0(count);
end



tic
T =  buildRRT(q_init, K, veh_param, driveMap, goalx, goaly, x_ref_arr, y_ref_arr, theta_ref_arr, sig_r_arr, sig_theta_arr, r_0_arr);
toc
save('BigTreeMaze2_500');
savefig(gcf, 'fig_Maze2_500');
