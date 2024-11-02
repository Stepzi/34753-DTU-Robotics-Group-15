function A = trajPol(condA,condB,tA,tB)
%TRAJPOL Computes a polynomial of 1,3,5th order that satisfies the boundary
%conditions condA and condB at time tA and tB
%   the size of condA = condB must match the desired polynomial order: 1 -> 1st order,
%   2 -> 3rd order, 3 -> 5th order
%   returns A, a vector with coefficients in increasing order

cond(1:3,1) = condA;
cond(4:6,1) = condB;
switch length(cond)
    case 2
        T = [1 tA           
            1 tB];
    case 4
        T = [1 tA tA^2 tA^3;
            0 1 2*tA 3*tA^2;
            1 tB tB^2 tB^3;
            0 1 2*tB 3*tB^2];
    case 6
        T = [1 tA tA^2 tA^3 tA^4 tA^5;
            0 1 2*tA 3*tA^2 4*tA^3 5*tA^4;
            0 0 2 6*tA 12*tA^2 20*tA^3;
            1 tB tB^2 tB^3 tB^4 tB^5;
            0 1 2*tB 3*tB^2 4*tB^3 5*tB^4;
            0 0 2 6*tB 12*tB^2 20*tB^3];
end
A = T\cond;

end

