function [eangle,eaxis] = QgetAngleAxis(q)
%  Extracts eigen-angle(s) and axes  from quaternion(s).
%
%  Usage:  [eangle,eaxis] = QgetAngleAxis(q); 
%
%       q = array of quaternions (Nx4) or (4xN).
%       eangle = returned eigen-angles, rad (Nx1) or (1xN).
%       eaxis = returned eigen-axes (Nx3) or (3xN).

%   2010-02-06  S.Derry     original version

[q,n,t] = qcheck4(q);

eangle = zeros(n,1);    % dummy eigen-angle vector
eaxis = zeros(n,3);     % dummy eigen-axis matrix

for i = 1:n
    if q(i,1) >= 0          % insure positive scalar (short quaternion)
        cq = q(i,:);
    else
        cq = -q(i,:);
    end
    vm = norm(cq(2:4));
    eaxis(i,:) = cq(2:4) / max(vm,1e-14);
    eangle(i) = 2 * atan2(vm,cq(1));
end

if t
    eaxis = eaxis';
    eangle = eangle';
end
