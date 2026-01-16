function q = QsetE(psi,theta,phi)

%  Return quaternion(s) corresponding to the given set(s) of Euler angles.
%  The Euler angles are expected to be in the 3-2-1 sequence.
%
%  Usage: q = QsetE(psi,theta,phi);
%
%  Input:     psi   = heading angle(s) (rads).   (Nx1) or (1xN)
%             theta = elevation angle(s) (rads). (Nx1) or (1xN)
%             phi   = bank angle(s) (rads).      (Nx1) or (1xN)
%
%  Outputs:   q = the corresponding quaternion(s).  (Nx4) or (4xN)

%
%    Calls:
%      none.
%
%   2010-02-06  S.Derry     support arrays of quaternions
%    Author:  Stephen D. Derry      2006-12-15
%

[psi,  n1,t1] = qcheck4(psi,1);
[theta,n2,t2] = qcheck4(theta,1);
[phi,  n3,t3] = qcheck4(phi,1);

if n1~=n2 || n1~=n3
    disp('Input dimensions must be consistent.');
    return
end

cpsi = cos(psi/2);
spsi = sin(psi/2);
cthe = cos(theta/2);
sthe = sin(theta/2);
cphi = cos(phi/2);
sphi = sin(phi/2);

q =  [ cpsi.*cthe.*cphi + spsi.*sthe.*sphi, ...
       cpsi.*cthe.*sphi - spsi.*sthe.*cphi, ...
       cpsi.*sthe.*cphi + spsi.*cthe.*sphi, ...
      -cpsi.*sthe.*sphi + spsi.*cthe.*cphi ];
  
if t1 && t2 && t3
	q = q';
end
