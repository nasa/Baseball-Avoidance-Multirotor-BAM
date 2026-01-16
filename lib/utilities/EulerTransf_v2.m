function [PHI2,THE2,PSI2] = EulerTransf_v2(PHI1,THE1,PSI1,Euler1,Euler2)
% EULERTRANSF_V2 Transfer one Euler sequence to another
% 
%Input
%     PHI1   rotation in degs about x axis in Euler1 rotation sequence
%     THE1   rotation in degs about y axis in Euler1 rotation sequence
%     PSI1   rotation in degs about z axis in Euler1 rotation sequence
%     Euler1 1x3 string defining first rotation order, e.g. '321'
%     Euler2 1x3 string defining desired rotation order, e.g. '312'
% 
%Output
%     PHI2   rotation in degs about x axis in Euler2 rotation sequence
%     THE2   rotation in degs about y axis in Euler2 rotation sequence
%     PSI2   rotation in degs about z axis in Euler2 rotation sequence
% 

% bbacon(?) circa late 2000's(?)
% $Id$

phi1=PHI1*pi/180;
theta1=THE1*pi/180;
psi1=PSI1*pi/180;

cphi=cos(phi1);
sphi=sin(phi1);
cthe=cos(theta1);
sthe=sin(theta1);
cpsi=cos(psi1);
spsi=sin(psi1);

rot123={'PHI','THE','PSI'};
PHI=[1 0 0;0 cphi sphi;0 -sphi cphi];
THE=[cthe 0 -sthe;0 1 0; sthe 0 cthe];
PSI=[cpsi spsi 0;-spsi cpsi 0;0 0 1];
T1=eye(3);
for k=1:3,
    id=eval(Euler1(k));
    eval(['T1=' rot123{id} '*T1;']);
end;
DirCrx=[2 3;3 1; 1 2];
%Obtain rotated frame definition in 0 Frame
F1=T1';
irt1=eval(Euler2(1));
irt3=eval(Euler2(3));
%Preserve right-hand rotation: if irt3 is equal to first vector in cross
%product DirCrx table, atan2(vec1,vec2), else atan2(-vec2,vec1)
if irt3==DirCrx(irt1,1)
  ROT1=atan2( F1(DirCrx(irt1,2),irt3),F1(DirCrx(irt1,1),irt3));
else
  ROT1=atan2( -F1(DirCrx(irt1,1),irt3),F1(DirCrx(irt1,2),irt3));
end
ROT1_d=ROT1*180/pi;
ROT_ax1=zeros(3,1);
ROT_ax1(irt1)=1;
ROT_ax1=sin(ROT1/2)*ROT_ax1;
quat021=[cos(ROT1/2);ROT_ax1];q0=quat021(1);;q1=quat021(2);q2=quat021(3);q3=quat021(4);
qcx=[0 -q3 q2;q3 0 -q1;-q2 q1 0];
%Express col(F1) in 1st rotated frame using quaternion transformation
F1_1=[2*ROT_ax1*ROT_ax1'+(q0*q0-ROT_ax1'*ROT_ax1)*eye(3)-2*q0*qcx]*F1;
irt2=eval(Euler2(2));

if irt3==DirCrx(irt2,1)
  ROT2=atan2( F1_1(DirCrx(irt2,2),irt3),F1_1(DirCrx(irt2,1),irt3));
else
  ROT2=atan2( -F1_1(DirCrx(irt2,1),irt3),F1_1(DirCrx(irt2,2),irt3));
end
ROT2_d=ROT2*180/pi;

ROT_ax2=zeros(3,1);
ROT_ax2(irt2)=1;
ROT_ax2=sin(ROT2/2)*ROT_ax2;
quat122=[cos(ROT2/2);ROT_ax2];q0=quat122(1);q1=quat122(2);q2=quat122(3);q3=quat122(4);
qcx=[0 -q3 q2;q3 0 -q1;-q2 q1 0];
%Express col(F1_1) in 2nd rotated frame using quaternion transformation
F1_2=[2*ROT_ax2*ROT_ax2'+(q0*q0-ROT_ax2'*ROT_ax2)*eye(3)-2*q0*qcx]*F1_1;

if irt2==DirCrx(irt3,1)
  ROT3=atan2( F1_2(DirCrx(irt3,2),irt2),F1_2(DirCrx(irt3,1),irt2));
else
  ROT3=atan2( -F1_2(DirCrx(irt3,1),irt2),F1_2(DirCrx(irt3,2),irt2));
end
ROT3_d=ROT3*180/pi;

for k=1:3,
    id2=eval(Euler2(k));
    eval([rot123{id2} '2=ROT' int2str(k) '_d;'])
end;



  
