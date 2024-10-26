function Ji = J_i(zi1,on,on1,type)
%J_I Summary of this function goes here
%   Detailed explanation goes here

if(type == "revolute")
    Ji(1:3,1) = cross(zi1,on-on1);
    Ji(4:6,1) = zi1;
else
    Ji(1:3,1) = zi1;
    Ji(4:6,1) = [0 0 0]';
end
end

