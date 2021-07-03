function [xd,b1d] = trajectory(i)

segment = 1/200;
segment2 = 1/300;

if i<=100
    x = 0;
    y = 0;
    z = -0.015;
    b1d = [1;0;0];
end
if i>100 && i<=300
    z = -0.015 - (i-100)*segment;
    x = 0;
    y = 0;
    b1d = [1;0;0];
end
if i>300 && i<=400
    x = 0;
    y = 0;
    z = -1.015;
    b1d = [1;0;0];
end
if i>400 && i<=700
     z = -1.015;
     x = (i-400)*segment2;
     y = -x;
     b1d = [1;0;0];
end
 
if i>700
    z=-1.015;
    x=1;
    y=-1;
    b1d = [1;0;0];
end

xd = [x;y;z];
end

