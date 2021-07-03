function [xd,b1d] = trajectory2(i)

segment2 = 1/200;

if i<=100
    x = 0;
    y = 0;
    z = -0.015;
    b1d = [1;0;0];
end
if i>100 && i<=300
    z = -0.015 - (i-100)*segment2;
    x = 0;
    y = 0;
    b1d = [1;0;0];
end
if i>300 && i<=500
    x = 0;
    y = 0;
    z = -1.015;
    b1d = [1;0;0];
end
if i>500 && i<=700
     z = -1.015;
     x = (i-500)*segment2;
     y = 0;
     b1d = [1;0;0];
end
if i>700 && i<=800
    z = -1.015;
    x = 1;
    y = 0;
    b1d = [1;0;0];
end
if i>800 && i<=1200
    z = -1.015;
    x = cos((i-800)*pi/400);
    y = sin((i-800)*pi/400);
    b1d = [1;0;0];
end
if i>1200
    z = -1.015;
    x = -1;
    y = 0;
    b1d = [1;0;0];
end
xd = [x;y;z];
end

