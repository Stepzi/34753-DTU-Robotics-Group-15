function J = jac(on,T)
%JACOBIAN Summary of this function goes here
%   Detailed explanation goes here
o0 = [0 0 0]';
o1 = T.T10(1:3,4);
o2 = T.T20(1:3,4);
o3 = T.T30(1:3,4);
z0 = [0 0 1]';
z1 = T.T10(1:3,3);
z2 = T.T20(1:3,3);
z3 = T.T30(1:3,3);

J = [J_i(z0,on,o0,"revolute"),J_i(z1,on,o1,"revolute"),J_i(z2,on,o2,"revolute"),J_i(z3,on,o3,"revolute")];

end

