function e = QgetEulers(q,seq)

%  Compute the Euler angles from a quaternion using a selected Euler angle sequence.
%
%  Usage: e = QgetEulers(q,seq);
%
%  Description:
%
%    Return the Euler angles from a quaternion using a selected Euler angle
%    sequence.  The sequence is specified as a 3-digit integer for the
%    three axes of rotation (X-axis = 1, Y-axis = 2, Z-axis = 3).  The
%    standard aircraft sequence (psi, theta, phi for heading, pitch, and
%    bank) is 321 (ZYX), which is the default.  There are 12 possible
%    sequences consisting of the three axes in any permutation; duplication
%    of axes is permitted as long as the same axis is not given twice
%    consecutively.  If an invalid sequence is given, a warning message is
%    displayed and zeros are returned.
%
%    The first and third angles range from -180 to +180 degrees, while the
%    second angle will range from -90 to +90 degrees.  Singularities occur
%    whenever the second angle is +90 or -90 degrees.
%
%  Input:     q = quaternion array, either (Nx4) or (4xN).  If N = 4, 
%                   the array will be considered as 4 quaternion rows.
%             s = a 3-digit integer specifying the desired sequence.
%
%  Outputs:   e = sequence of Euler rotation angles (using the specified
%                   sequence).  Result is either (Nx3) or (3xN), with
%                   angles in rad.

%
%    Calls:
%      none.
%
%   2010-02-06  S.Derry     support arrays of quaternions
%    Author:  Stephen D. Derry      2007-03-15
%

%
%	Compute body rotation matrices and Euler angles from quaternions.
%

[qa,n,t] = qcheck4(q);  % check dimensionality, convert to rows
e = zeros(n,3);

for i = 1:n
    q0 = qa(i,1);				% extract quaternion elements
    q1 = qa(i,2);
    q2 = qa(i,3);
    q3 = qa(i,4);

    b11 = q0^2 + q1^2 - q2^2 - q3^2;	% compute cosine matrix elements
    b12 = 2 * (q1*q2 + q0*q3);
    b13 = 2 * (q1*q3 - q0*q2);
    b21 = 2 * (q1*q2 - q0*q3);
    b22 = q0^2 - q1^2 + q2^2 - q3^2;
    b23 = 2 * (q2*q3 + q0*q1);
    b31 = 2 * (q1*q3 + q0*q2);
    b32 = 2 * (q2*q3 - q0*q1);
    b33 = q0^2 - q1^2 - q2^2 + q3^2;

%	Euler angle calcs need to be protected against angle2 = +/- 90 deg

    if seq == 123
        a1 = atan2 (-b32, b33);
        a2 = asin (max(-1,min(1,b31)));
        a3 = atan2 (-b21, b11);
    elseif seq == 231
        a1 = atan2 (-b13, b11);
        a2 = asin (max(-1,min(1,b12)));
        a3 = atan2 (-b32, b22);
    elseif seq == 312
        a1 = atan2 (-b21, b22);
        a2 = asin (max(-1,min(1,b23)));
        a3 = atan2 (-b13, b33);

    elseif seq == 132
        a1 = atan2 (b23, b22);
        a2 = asin (max(-1,min(1,-b21)));
        a3 = atan2 (b31, b11);
    elseif seq == 213
        a1 = atan2 (b31, b33);      % mistake in report?
        a2 = asin (max(-1,min(1,-b32)));
        a3 = atan2 (b12, b22);
    elseif seq == 321
        a1 = atan2 (b12, b11);
        a2 = asin (max(-1,min(1,-b13)));
        a3 = atan2 (b23, b33);

    elseif seq == 121
        a1 = atan2 (b12, -b13);
        a2 = acos (max(-1,min(1,b11)));
        a3 = atan2 (b21, b31);
    elseif seq == 232
        a1 = atan2 (b23, -b21);
        a2 = acos (max(-1,min(1,b22)));
        a3 = atan2 (b32, b12);
    elseif seq == 313
        a1 = atan2 (b31, -b32);
        a2 = acos (max(-1,min(1,b33)));
        a3 = atan2 (b13, b23);

    elseif seq == 131
        a1 = atan2 (b13, b12);
        a2 = acos (max(-1,min(1,b11)));
        a3 = atan2 (b31, -b21);
    elseif seq == 212
        a1 = atan2 (b21, b23);
        a2 = acos (max(-1,min(1,b22)));
        a3 = atan2 (b12, -b32);
    elseif seq == 323
        a1 = atan2 (b32, b31);
        a2 = acos (max(-1,min(1,b33)));
        a3 = atan2 (b23, -b13);

    else
        disp (['Invalid Euler angle sequence ' num2str(seq)])
        a1 = 0;
        a2 = 0;
        a3 = 0;
    end

    e(i,:) = [a1, a2, a3];
end

if t
    e = e';
end
