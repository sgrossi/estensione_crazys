a = zeros(2000,3);
b = zeros(2000,3);
for i=1:2000
    
    [xd,b1d] = trajectory(i);
    a(i,1) = xd(1);
    a(i,2) = xd(2);
    a(i,3) = xd(3);
end