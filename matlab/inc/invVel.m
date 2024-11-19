function q_d = invVel(J,Xi)
%INVVEL Summary of this function goes here
%   Detailed explanation goes here
J_inv = pinv(J);
q_d = J_inv*Xi;
end

