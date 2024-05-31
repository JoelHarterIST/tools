% This file stores constants that can be accessed by other scripts/functions.
% The values can be stored here in one official place rather than being written in each file or passed as arguments to functions.
% This makes things easy and keeps things organized
%
% Joel T Harter
% 2024 May 14

classdef Constants
    properties(Constant=true)
        
        % Fundamental Physical Constants
        G = 6.67430e-11;% [N·m^2·kg^-2] universal gravitational constant
        c = 299792458;% [m·s^-1] (exact) speed of light in vacuum
        h = 6.62607015e-34;% [J·Hz^-1] Planck's constant

        % Earth Properties
        a = 6378137.0;% [m] semi-major axis <WGS-84>
        b = 6.356752314245179e6;% [m] semi-minor axis :=(1-f)*a | f:=1/298.257223563 <WGS-84>
        R1 = 6.371008771415059e6;% [m] arithmetic mean mean radius :=(2*a_e+b_e)/3
        R2 = 6.371007180918474e6;% [m] authalic (sphere with equal area) radius :=sqrt(a_e^2/2+b_e^2/2*atanh(e)/e) | e:=sqrt((a_e^2-b_e^2)/a_e^2)
        R3 = 6.371000790009154e6;% [m] volumetric (sphere with equal volume) radius :=(a_e^2*b_e)^(1/3)
        Omega = 7.2921151467e-5;% [rad·s^-1] earth's angular speed. This is the value used by Interstellar Technologies; WGS-84 specifies this value should be 72.92115e-6 rad/s (less precise);
        tilt=23.439281;% [deg] earth's axial tilt (obliquity of ecliptic)
        mu = 3.986004418e14;% [m^3·s^-2] earth's gravitational constant, including mass of atmosphere :=G*M_e; This value was defined by <WGS-84> independently of G or M_e since it can be measured more directly
        g_0 = 9.80665;% [m·s^-2] standard acceleration of gravity set by ISO 80000 (international standard)
        M = 5.972168494074286e24;% [kg] earth mass; NOTE: current best estimate of earth's mass is (5.9722±0.0006)*10^24 kg. The more precise value assigned to M_e is obtained from mu_e/G to keep things consistent; the significant digits do not imply higher accuracy.
        tropicalYear = 3.1556925216e7;% [s] tropical year, the length of time it takes the earth to go from one vernal equinox to the next
        siderialYear = 3.15581497635456e7;% [s] siderial year, the length of time it takes the sun to return to the same relative position with respect to the fixed stars, taking the earth's axial precession into account
        
        % Colors
        hex2frac_color=@(hex)sum(reshape(mod(lower(char(hex))-48,39),2,3).*[16;1],1)/255;% convert hex color triplets like 'ff0000' to 1×3 fractional color vectors
        orange=[1, 0.458823529411765, 0];% #ff7500 official Interstellar orange
    end
end