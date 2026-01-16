function [e,m] = Qconv(q)

%  Compute the Euler angles and cosine matrix for a given quaternion.
%
%  function [e,m] = Qconv(q)
%
%  Usage: [e,m] = Qconv(q);
%
%  Description:
%
%    Return the Euler angles and cosine matrix for a given quaternion.
%
%  Input:     q = quaternion.  Must be a single quaternion,
%                   either 1x4 or 4x1.
%
%  Outputs:   e = sequence of Euler rotation angles (psi, theta, phi),
%               a 3x1 vector with angles given in rad.
%             m = cosine matrix.

%
%    Calls:
%      none.
%
%       2010-02-06  S.Derry     clarify introductory comments
%    Author:  Stephen D. Derry      2006-12-15
%

%
%	Compute body rotation matrices and Euler angles from quaternions.
%

q0 = q(1);				% extract quaternion elements
q1 = q(2);
q2 = q(3);
q3 = q(4);

b11 = q0^2 + q1^2 - q2^2 - q3^2;	% compute rotation matrix elements
b12 = 2 * (q1*q2 + q0*q3);
b13 = 2 * (q1*q3 - q0*q2);
b21 = 2 * (q1*q2 - q0*q3);
b22 = q0^2 - q1^2 + q2^2 - q3^2;
b23 = 2 * (q2*q3 + q0*q1);
b31 = 2 * (q1*q3 + q0*q2);
b32 = 2 * (q2*q3 - q0*q1);
b33 = q0^2 - q1^2 - q2^2 + q3^2;

m = [b11 b12 b13 ; b21 b22 b23 ; b31 b32 b33];	% assemble matrix

%	Euler angle calcs need to be protected against theta = +/- 90 deg

psi = atan2 (b12, b11);		% compute Euler angles
theta = -asin (max(-1,min(1,b13)));	% limit to keep asin real
phi = atan2 (b23, b33);

e = [psi; theta; phi];
