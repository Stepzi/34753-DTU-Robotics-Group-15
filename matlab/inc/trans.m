function T = trans(axis, l)
    % TRANS Creates a 4x4 translation matrix along a specified axis
    %
    % Usage:
    %   T = trans(axis, l)
    %
    % Inputs:
    %   axis - a character specifying the axis ('x', 'y', or 'z')
    %   l    - the translation length along the specified axis
    %
    % Output:
    %   T - the resulting 4x4 translation matrix

    % Initialize the translation matrix as an identity matrix
    T = eye(4);

    % Set the translation value along the specified axis
    switch axis
        case 'x'
            T(1, 4) = l;
        case 'y'
            T(2, 4) = l;
        case 'z'
            T(3, 4) = l;
        otherwise
            error('Invalid axis. Choose ''x'', ''y'', or ''z''.');
    end
end
