function [eangle,eaxis] = Qcomp(qm1,qm2);
% q1 and q2 are Nx4 matrices

n = size(qm1,1);     % number of quaternions to compare
eangle = zeros(n,1);    % dummy eigen-angle vector
eaxis = zeros(n,3);     % dummy eigen-axis matrix

for i = 1:n
    q1 = (qm1(i,:))';   % q1 and q2 are column vectors
    q2 = (qm2(i,:))';
    cq = Qmult(Qinvert(q1),q2)';	% comparator (row vector)
    if cq(1) < 0          % insure positive scalar (short quaternion)
        cq = -cq;
    end
    vm = norm(cq(2:4));
    eaxis(i,:) = cq(2:4) / vm;
    eangle(i) = 2 * atan2(vm,cq(1));
end
return
