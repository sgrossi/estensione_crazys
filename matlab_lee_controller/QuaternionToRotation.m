function out = QuaternionToRotation (qr, qi, qj, qk)

s = norm([qr;qi;qj;qk])^(-2);

R = [1-2*s*(qj^2+qk^2), 2*s*(qi*qj-qk*qr), 2*s*(qi*qk+qj*qr); ...
    2*s*(qi*qj+qk*qr), 1-2*s*(qi^2+qk^2), 2*s*(qj*qk-qi*qr); ...
    2*s*(qi*qk-qj*qr), 2*s*(qj*qk+qi*qr), 1-2*s*(qi^2+qj^2)];

out = R;

end

