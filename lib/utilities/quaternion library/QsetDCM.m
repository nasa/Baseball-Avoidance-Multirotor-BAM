function q = QsetDCM(m)

%  Return quaternion corresponding to the given direction cosine matrix.
%
%  function q = QsetDCM(m)
%
%  Usage: q = QsetDCM(m);
%
%  Description:
%
%    Return quaternion corresponding to the given direction cosine matrix.  The
%    quaternion is a positive-sense eigenaxis quaternion.  The method is
%    from Stevens and Lewis _Aircraft Control and Simulation_, First
%    Edition page 42.
%
%  Input:     m = direction cosine matrix.
%
%  Outputs:   q = the corresponding quaternion.

%
%    Calls:
%      none.
%
%    Author:  Stephen D. Derry      2008-06-19
%

fourq0sqd = 1 + m(1,1) + m(2,2) + m(3,3);   % 4 q0 ^ 2
fourq1sqd = 1 + m(1,1) - m(2,2) - m(3,3);   % 4 q1 ^ 2
fourq2sqd = 1 - m(1,1) + m(2,2) - m(3,3);   % 4 q2 ^ 2
fourq3sqd = 1 - m(1,1) - m(2,2) + m(3,3);   % 4 q3 ^ 2

maxqsqd = max([fourq0sqd,fourq1sqd,fourq2sqd,fourq3sqd]);

if fourq0sqd >= maxqsqd
    q0 = sqrt(fourq0sqd/4);
    q1 = (m(2,3) - m(3,2)) / (4*q0);
    q2 = (m(3,1) - m(1,3)) / (4*q0);
    q3 = (m(1,2) - m(2,1)) / (4*q0);
elseif fourq1sqd >= maxqsqd
    q1 = sqrt(fourq1sqd/4);
    q0 = (m(2,3) - m(3,2)) / (4*q1);
    q2 = (m(1,2) + m(2,1)) / (4*q1);
    q3 = (m(1,3) + m(3,1)) / (4*q1);
elseif fourq2sqd >= maxqsqd
    q2 = sqrt(fourq2sqd/4);
    q0 = (m(3,1) - m(1,3)) / (4*q2);
    q1 = (m(1,2) + m(2,1)) / (4*q2);
    q3 = (m(2,3) + m(3,2)) / (4*q2);
else
    q3 = sqrt(fourq3sqd/4);
    q0 = (m(1,2) - m(2,1)) / (4*q3);
    q1 = (m(1,3) + m(3,1)) / (4*q3);
    q2 = (m(2,3) + m(3,2)) / (4*q3);
end

q = [q0 q1 q2 q3];      % form final quaternion
if q0 < 0               % insure positive scalar (short rotation)
    q = -q;
end
