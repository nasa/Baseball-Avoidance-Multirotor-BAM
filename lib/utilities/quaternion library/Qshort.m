function q = Qshort(q1)

%  Return the given quaternion(s), insuring that the scalar is positive
%  (i.e. that the quaternion represents a "short" rotation (<180 deg).
%
%  function q = Qshort(q1)
%
%  Usage: q = Qshort(q1);
%
%  Description:
%
%    Return the given quaternion(s) q1, insuring positive scalar.  
%
%  Input:     q1 = quaternion array, either (Nx4) or (4xN).  If N = 4, 
%                   the array will be considered as 4 quaternion rows.
%
%  Outputs:   q = the inverse of q1, same dimensionality as q1.

%
%    Calls:
%      none.
%
%   2010-03-01  S.Derry

[q,n,t] = qcheck4(q1);  % check dimensionality, convert to rows
ix = find(q(:,1) < 0);
q(ix,:) = -q(ix,:);
if t
    q = q';
end
