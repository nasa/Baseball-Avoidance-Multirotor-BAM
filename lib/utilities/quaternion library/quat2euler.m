function [PHI2,THE2,PSI2]=quat2euler(quat,Euler2)
% QUAT2EULER Convert quaternion to Euler angle sequence.
%   [PHI2,THE2,PSI2] = QUAT2EULER(quat,Euler2) Convertw quaternion, quat to
%   Euler angles PHI2, THETA2, and PSI2 in desired Euler anlge sequence
%   Euler2.
% 
% Input
%     quat   quaterion
%     Euler2 1x3 string defining desired rotation order, e.g. '312'
% 
% Output
%     PHI2   rotation in degs about x axis in Euler2 rotation sequence
%     THE2   rotation in degs about y axis in Euler2 rotation sequence
%     PSI2   rotation in degs about z axis in Euler2 rotation sequence

% bbacon circa late 2000's
% $Id$

quat=quat(:);
q0=quat(1);
qa=quat(2:4,1);
dcm=2*qa*qa'+(q0*q0-qa'*qa)*eye(3)-2*q0*[0 -qa(3) qa(2);qa(3) 0 -qa(1);-qa(2) qa(1) 0];  
T1=dcm;
DirCrx=[2 3;3 1; 1 2];
rot123={'PHI','THE','PSI'};
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
%Determinacy check
norder=[eval(Euler2(1)) eval(Euler2(2)) eval(Euler2(3))];
if abs(abs(ROT2_d)-90)<1e-06,
    disp(['Warning: indeterminancy for ' rot123{norder(1)} ' and ' rot123{norder(3)}]);
    v1=zeros(3,1);v2=zeros(3,1);
    v1(norder(1),1)=1;
    v2(norder(3),1)=1;
    v3m=v1-T1'*v2;
    v3p=v1+T1'*v2;
    if max(abs(v3m))<1e-06,
        c=ROT1_d+ROT3_d;
        disp([rot123{norder(1)} '+' rot123{norder(3)} '= ' num2str(c)]);
    elseif max(abs(v3p))<1e-06
        c=ROT1_d-ROT3_d;
        disp([rot123{norder(1)} '-' rot123{norder(3)} '= ' num2str(c)]);
    end
end


  
