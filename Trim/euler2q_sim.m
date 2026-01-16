function q = euler2q_sim(phi,the,psi)
%
%  q = euler2q_sim(phi,the,psi); Converts euler angles to quaternion
%
%  Description:
%
%    Computes the quaternion for the input Euler angles using 
%    the standard (body fixed) aerospace rotation sequence (yaw-pitch-roll).   
%
%  Input:
%    
%    phi = Euler roll angle, rad.
%    the = Euler pitch angle, rad.
%    psi = Euler yaw angle, rad.
%
%  Output:
%
%    q = quaternion = [q0, qx, qy, qz].
%

% Written by:
% Michael J. Acheson and Gene Morelli
% NASA LaRC Research Center
%
% History:
% 9/10/2025 - created MJA

cos_phi2=cos(phi/2);
sin_phi2=sin(phi/2);
cos_the2=cos(the/2);
sin_the2=sin(the/2);
cos_psi2=cos(psi/2);
sin_psi2=sin(psi/2);

%  Express Euler angles as quaternion elements
q0=cos_phi2.*cos_the2.*cos_psi2 + sin_phi2.*sin_the2.*sin_psi2;
qx=sin_phi2.*cos_the2.*cos_psi2 - cos_phi2.*sin_the2.*sin_psi2;
qy=cos_phi2.*sin_the2.*cos_psi2 + sin_phi2.*cos_the2.*sin_psi2;
qz=cos_phi2.*cos_the2.*sin_psi2 - sin_phi2.*sin_the2.*cos_psi2;

%  Define the quaternion.
q = [q0; qx; qy; qz];

%  Ensure rotation scalar is positive
if (q0 < 0)
  q = -q;
end

return