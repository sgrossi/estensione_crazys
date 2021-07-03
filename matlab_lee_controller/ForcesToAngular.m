function ang = ForcesToAngular(input, MixMatr)

ang_2 = MixMatr * input;
ang = zeros(4,1);

if ang_2(1) < 0
    ang(1) = 0;
else
    ang(1) = sqrt(ang_2(1));
end
if ang(1) > 3052
    ang(1) = 3052;
end

if ang_2(2) < 0
    ang(2) = 0;
else
    ang(2) = sqrt(ang_2(2));
end
if ang(2) > 3052
    ang(2) = 3052;
end

if ang_2(3) < 0
    ang(3) = 0;
else
    ang(3) = sqrt(ang_2(3));
end
if ang(3) > 3052
    ang(3) = 3052;
end

if ang_2(4) < 0
    ang(4) = 0;
else
    ang(4) = sqrt(ang_2(4));
end
if ang(4) > 3052
    ang(4) = 3052;
end

end

