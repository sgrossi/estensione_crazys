function [xd,b1d] = trajectory3(i)

segment = 0.01;


if i<=100
    x = 0;
    y = 0;
    z = -0.015;
    b1d = [1;0;0];
end
if i>100 && i<=200
    z = -0.015 - (i-100)*segment;
    x = 0;
    y = 0;
    b1d = [1;0;0];
end
if i>200 && i<=400
    x = 0;
    y = 0;
    z = -1.015;
    b1d = [1;0;0];
end
if i>400 && i<=500
     z = -1.015;
     x = 0;
     y = 0;
     b1d = [cos((i-400)*pi/200);sin((i-400)*pi/200);0];
end

if i>500
    z = -1.015;
    x = 0;
    y = 0;
    b1d = [0;1;0];
end
xd = [x;y;z];
end

