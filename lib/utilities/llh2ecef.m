function ecef = llh2ecef(lat,lon,h,earth)
% LLH2ECEF Lat-Long-Alt to ECEF coordinates
% ecef = LLH2ECEF (lat,lon,h,earth) Converts geodetic latitude, longitude, and altitude to Cartesian ECEF coordinates.
%
%   WGS-84 is the reference ellipsoid, and the methods used are taken from
%   "Explanatory Supplement to the Astronomical Almanac by P. Kenneth
%   Seidelmann (U.S. Naval Observatory), University Science Books, 1992.
%   Script uses publicly releasable source material
%   Earth related quantities are provided by unclassified, publicly
%   releaseable from the National Geospace Intelligence Agency standard:
%   NGA.STND.0036_1.0.0_WGS84 which can be found at:
%   https://nsgreg.nga.mil/doc/view?i=4085&month=3&day=21&year=2023
% *************************************************************************
%
%   The ECEF coordinate system is a right-handed Cartesian system, centered at the
%   earth's mass center, with the +Z axis through the north pole and the +X
%   axis through the intersection of the equator and the prime meridian.
%
%       lat = geodetic latitude (rad)
%       lon = longitude (rad)
%       h = geodetic altitude (i.e. wrt ellipsoid)
%       earth = data structure containing ellipsoid parameters 
%
%       ecef = resulting ECEF [X,Y,Z] coordinates in 3x1 column vector
%
%   h and ecef are in the same units as the ellipsoid semi-major axis.

%    Author:  Stephen D. Derry      2007-03-12
% $Id: llh2ecef.m 411 2016-02-28 02:44:02Z eheim $
% Modified 1/29/2025, MJA

a = earth.RadiusEquator;
% e = earth.Eccentricity;
f = earth.Flattening;

sinphi = sin(lat);      % Seidelmann method
% N0 = a ./ sqrt(1 - e*e * sinphi.*sinphi);
N0 = a ./ sqrt(1 - (2*f - f*f) * sinphi.*sinphi);
xy = (N0 + h) .* cos(lat);
X = xy .* cos(lon);
Y = xy .* sin(lon);
% Z = ((1 - e*e) * N0 + h) .* sinphi;
Z = ((1 -(2*f - f*f)) * N0 + h) .* sinphi;
ecef = [X; Y; Z];
