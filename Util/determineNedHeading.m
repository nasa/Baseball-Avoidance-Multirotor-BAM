function y = determineNedHeading(u)

s = cos(u/2);
v = sin(u/2);
y = [s, 0*v, 0*v, v]';

end