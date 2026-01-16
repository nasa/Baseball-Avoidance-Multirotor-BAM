function q = QsetRotAxis(r)

%  Return quaternion(s) corresponding to the given rotation vector(s).
%
%  Usage: q = QsetRotAxis(r);
%
%  Description:
%
%    Return quaternion(s) corresponding to the given rotation vector(s).  The
%    rotation vector(s) are a simultaneous rotation of rx (r(1)) about the X
%    axis ry (r(2)) about Y, and rz (r(3)) about Z.
%
%  Input:     r = vector(s) of rotation angles (rads), either (Nx3) or (3xN).
%
%  Outputs:   q = the corresponding quaternion, either (Nx4) or (4xN).

%
%    Calls:
%      none.
%
%   2010-05-26  S.Derry     bug fix to support arrays of quaternions
%   2010-02-06  S.Derry     support arrays of quaternions
%    Author:  Stephen D. Derry      2007-03-06
%

[ra,n,t] = qcheck4(r,3);
axis = zeros(n,3);
theta = zeros(n,1);

for i = 1:n
    theta(i) = norm(ra(i,:));    % total rotation angle (magnitude)
    if abs(theta(i)) > 1e-14
        axis(i,:) = ra(i,:)/theta(i);     % axis of rotation (unit vector)
    else
        axis(i,:) = [1 0 0];
    end
end

c = cos(theta/2);
s = sin(theta/2);

q =  [ c, ...
       s .* axis(:,1), ...
       s .* axis(:,2), ...
       s .* axis(:,3) ];

if t
    q = q';
end
