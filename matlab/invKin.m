function q = invKin(x4,o4,elbow,p)
%invKin: Computes joint configuration from desired tip position and
%orientation
%   x4: x-component of tip orientation
%   o4: tip position
%   elbow: "ElbowUp" or "ElbowDown" solution
%   p: paramteres d1-4,a1-4


ox = o4(1);
oy = o4(2);
oz = o4(3);

x_4z = x4(3);

q1 = atan2(oy,ox);

a = atan2(x_4z,sqrt(1-x_4z^2));

w_x = ox-p.a4*cos(a)*cos(q1);
w_y = oy-p.a4*sin(q1);
w_z = oz-p.a4*sin(a);
w = [w_x;w_y;w_z];


r = sqrt(w_x^2 +w_y^2);

s = w_z-p.d1;
cos3 = (r^2+s^2-p.a2^2-p.a3^2)/(2*p.a2*p.a3);
if(elbow == "ElbowUp")
    q3 = atan2(-sqrt(1-cos3^2),cos3);
    q2 = atan2(-r,s)+atan2(sqrt(1-cos3^2)*p.a3,p.a2+p.a3*cos3);
else
    q3 = atan2(+sqrt(1-cos3^2),cos3);
    q2 = atan2(-r,s)-atan2(sqrt(1-cos3^2)*p.a3,p.a2+p.a3*cos3);
end

% q2 = atan2(-r,s)+atan2(sqrt(1-cos3^2)*p.a3,p.a2+p.a3*cos3);
% q2 = atan2(s,r)-atan2(sqrt(1-cos3^2)*p.a3,p.a2+p.a3*cos3);


q4 = a-pi/2-q2-q3;

q = [q1;q2;q3;q4];
end

