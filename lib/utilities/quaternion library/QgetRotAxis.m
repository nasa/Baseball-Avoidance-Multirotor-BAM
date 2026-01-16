function r = QgetRotAxis(q)

%  Return rotation vector corresponding to the given quaternion.
%
%  function r = QgetRotAxis(q)
%
%  Usage: r = QgetRotAxis(q);
%
%  Description:
%
%    Return rotation vector corresponding to the given quaternion.  The
%    rotation vector is a simultaneous rotation of rx (r(1)) about the X
%    axis ry (r(2)) about Y, and rz (r(3)) about Z.
%
%  Input:     q = quaternion array, either (Nx4) or (4xN).  If N = 4, 
%                   the array will be considered as 4 quaternion rows.
%
%  Outputs:   r = the corresponding vector of rotation angles (rads),
%                   either (Nx3) or (3xN).

%
%    Calls:
%      none.
%
%   2010-02-06  S.Derry     support arrays of quaternions
%    Author:  Stephen D. Derry      2007-03-06
%

[qa,n,t] = qcheck4(q);  % check dimensionality, convert to rows
r = zeros(n,3);

theta = 2 * acos(qa(:,1));     % rotation angles (magnitude)
s = sin(theta/2);      	% sines of half rotation angles
for i = 1:n
    if abs(s(i)) > 1e-14
        r(i,:) = q(i,2:4) * theta / s(i);     % rotation vector
    else                    % (avoid indeterminate division)
        r(i,:) = [0 0 0];	% rotation nearly zero (preserve dimensionality)
    end
end

if t
    r = r';
end

