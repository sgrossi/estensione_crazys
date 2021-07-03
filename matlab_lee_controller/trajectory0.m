function [xd,b1d] = trajectory0(i)

segment = 1/200;

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
if i>300
    z=-1;
    x=0;
    y=0;
    b1d = [1;0;0];
end

xd = [x;y;z];
end

