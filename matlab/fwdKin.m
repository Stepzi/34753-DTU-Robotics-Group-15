function T = fwdKin(q,p)
%fwdKin Computes forward kinematics and return local and global
%transformation matrices
%   q: joint vector
%   p: system DH-parameters
T.T10 = DH(q(1),p.d1,0,pi/2);
T.T21 = DH((q(2)+pi/2),0,p.a2,0);
T.T32 = DH(q(3),0,p.a3,0);
T.T43 = DH(q(4),0,p.a4,0);
T.T54 = [eye(3),[-0.015;0.045;0];
        0,0,0,1];

T.T20 = T.T10*T.T21;
T.T30 = T.T20*T.T32;
T.T40 = T.T30*T.T43;
T.T50 = T.T40*T.T54;

end

