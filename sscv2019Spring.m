function [Ul, Ur] = sscv2019Spring(speed, steering,AxelLen, MaxSpeed)
%SSCV2019SPRING Summary of this function goes here
%   speed (speed in m/s)
%   steering (turning in radians/meter)
%   AxelLen (distance between wheels in meters)
%   MaxSpeed (top abs speed of any wheel)
% If a wheel runs into the MaxSpeed constraint, the speed will be less.

  omega = min(max(-MaxSpeed * (2.0/AxelLen), steering*speed), MaxSpeed * (2.0/AxelLen));
  speed_lim = max(min(speed, MaxSpeed - AxelLen/2.0 * abs(omega)), .01);
  Ul = speed_lim - .5*(omega)*AxelLen;
  Ur= speed_lim + .5*(omega)*AxelLen;
end

