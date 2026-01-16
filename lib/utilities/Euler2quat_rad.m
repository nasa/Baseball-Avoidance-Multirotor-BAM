function quat=Euler2quat(PHI2,THE2,PSI2,Euler2)
% Euler2quat Creates a quaternion from given Euler angle sequence.
% quat = Euler2quat(PHI2,THE2,PSI2,Euler2) Creates a quaternion from given
% Euler angle sequence.
% Input
%     PHI2   rotation in rad about x axis in Euler2 rotation sequence
%     THE2   rotation in rad about y axis in Euler2 rotation sequence
%     PSI2   rotation in rad about z axis in Euler2 rotation sequence
%     Euler2 1x3 string defining desired rotation order, e.g. '312'
% 
% Output
%     quat   quaterion

% bbacon circa late 2000's
% Modified MJA for radian version 1/22/2025
% $Id$

AngLab={'PHI2','THE2','PSI2'};
norder=[eval(Euler2(1)) eval(Euler2(2)) eval(Euler2(3))];

phi2=PHI2; %*pi/180;
theta2=THE2; %*pi/180;
psi2=PSI2; %*pi/180;

cphi=cos(phi2);
sphi=sin(phi2);
cthe=cos(theta2);
sthe=sin(theta2);
cpsi=cos(psi2);
spsi=sin(psi2);

rot123={'PHI','THE','PSI'};
PHI=[1 0 0;0 cphi sphi;0 -sphi cphi];
THE=[cthe 0 -sthe;0 1 0; sthe 0 cthe];
PSI=[cpsi spsi 0;-spsi cpsi 0;0 0 1];
T1=eye(3);
for k=1:3,
    id=eval(Euler2(k));
    eval(['T1=' rot123{id} '*T1;']);
end;


dum=eval(AngLab{norder(2)});
if abs(dum>90),
    disp(['Second rotation(' AngLab{norder(2)} ') must be within [-90, 90] degrees';]);
    quat=[NaN NaN NaN NaN]';
    return
end;
%check for colinear rotational sense when second rotation is +-90
if abs(abs(dum)-90)<1e-06,
    disp(['Warning: indeterminancy for ' AngLab{norder(1)} ' and ' AngLab{norder(3)}]);
    v1=zeros(3,1);v2=zeros(3,1);
    v1(norder(1),1)=1;
    v2(norder(3),1)=1;
    v3m=v1-T1'*v2;
    v3p=v1+T1'*v2;
    if max(abs(v3m))<1e-06,
        disp([AngLab{norder(1)} '+' AngLab{norder(3)} '=constant']);
    elseif max(abs(v3p))<1e-06
        disp([AngLab{norder(1)} '-' AngLab{norder(3)} '=constant']);
    end
end
    

d2r=pi/180;
qphi=[cos(PHI2*d2r/2) sin(PHI2*d2r/2) 0 0]';
qthe=[cos(THE2*d2r/2) 0 sin(THE2*d2r/2) 0]';
qpsi=[cos(PSI2*d2r/2) 0 0 sin(PSI2*d2r/2)]';

Q=[qphi qthe qpsi];

q2 = Qmult(Q(:,norder(1)),Q(:,norder(2)));
quat = Qmult(q2,Q(:,norder(3)));
quat=sign(quat(1))*quat(:);
return





  
