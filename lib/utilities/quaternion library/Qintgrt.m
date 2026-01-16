function q = Qintgrt(q1,omega,omegaDot,dt)

%  Quaternion integration.
%
%  function q = Qintgrt(q1,omega,omegaDot,dt)
%
%  Usage: q = Qintgrt(q1,omega,omegaDot,dt);
%
%  Description:
%
%    Integrate a given quaternion one time step with the given angular rate and accel.
%    The local linearization method is used.
%
%  References:
%       NASA TN D-7347; "Development and Application of a Local Linearization Algorithm for the 
%           Integration of Quaternion Rate Equations in Real-Time Flight Simulation Problems";
%           Barker, Bowles, and Williams;  December 1973.
%       "Review of Attitude Representations Used for Aircraft Kinematics";  Phillips and Hailey;
%           AIAA Journal of Aircraft, Vol.38 No.4, July - August 2001.
%
%  Input:     q1 = original quaternion.
%             omega = vector (3x1) of angular rates.
%             omegaDot = vector (3x1) of angular accelerations.
%             dt = time step.
%
%  Outputs:   q = the new quaternion propagated one step forward in time.

%
%    Calls:
%      none.
%
%    Author:  Stephen D. Derry      2006-12-18
%

omegak = norm(omega);
lomegak = max (0.001, omegak);	% limited to preclude division by zero
lomegak2 = lomegak^2;
rhok = lomegak * dt / 2;

C1  = cos(rhok);
C2p = sin(rhok) / lomegak;
C3p = 2 * (1 - C1) / lomegak2;
C4  = (2*C2p - dt) / lomegak2;

H =  C1 + C4 * dot(omega,omegaDot);
V = C4 * cross(omega,omegaDot) + C2p * omega + C3p * omegaDot;
K = V(1);
J = V(2);
G = V(3);

Mk = [ H -K -J -G ; K H G -J ; J -G H K ; G J -K H ];
Xk1 = Mk * q1;
q = Xk1 ./ norm(Xk1);	% normalize
