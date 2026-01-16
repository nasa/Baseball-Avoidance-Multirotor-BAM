function q = Qunity

%  Return a unity quaternion.
%
%  Usage: q = Qunity;
%
%  Description:
%
%    Returns a unity quaternion (i.e. all rotation angles are zero).  
%
%  Outputs:   q = quaternion (1,0,0,0), as a (1x4) row vector.

%
%    Calls:
%      none.
%
%   2010-02-06  S.Derry     return row vector
%    Author:  Stephen D. Derry      2006-12-15
%

q = [ 1.0, 0.0, 0.0, 0.0 ];
