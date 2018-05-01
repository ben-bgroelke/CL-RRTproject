function I = sortClosest(T,sx,sy, rho)
% findSafe = 
Vrt = T.Vert;
Lp = zeros(1, length(Vrt));
count = 0;
for i = 1:length(Vrt)
    if Vrt(i).safe == 1
        
    else
        
    end
    
end
for i = 1:length(Vrt)
    if Vrt(i).safe == 1
        x = Vrt(i).x;
        y = Vrt(i).y;
        theta = Vrt(i).theta;
        R = [cos(theta) -sin(-theta); sin(-theta) cos(theta)];
        %now transform s.t. node is at (0,0,0) - RTMP 1109
        sxx = sx - x;
        syy = sy - y;
        s = R*[sxx; syy];     %theta = 0
        sx_rel = s(1);
        sy_rel = abs(s(2));
        if  sqrt((sx_rel - 0)^2 + (sy_rel - rho)^2) < rho %relative point is in Dp  - RTMP 1109
            df = sqrt((sx_rel^2) + (sy_rel + rho)^2);
            dc = sqrt((sx_rel^2) + (sy_rel - rho)^2);
            theta_i = atan2(sx_rel, rho - sy_rel);
            theta_c = mod((theta_i + 2*pi), 2*pi);
            phi = acos((5*rho^2 - df^2)/(4*rho^2));
            alpha = 2*pi - phi;
            Lp(i) = rho*(alpha + asin(dc*sin(theta_c)/df) + asin((rho*sin(phi))/df));
        else
            dc = sqrt((sx_rel^2) + (sy_rel - rho)^2);
            theta_i = atan2(sx_rel, rho - sy_rel);
            theta_c = mod((theta_i + 2*pi), 2*pi);
            Lp(i) = sqrt(dc^2 - rho^2) + rho*(theta_c - acos(rho/dc));
        end
    end
end

[~, I] = sort(Lp);

end
