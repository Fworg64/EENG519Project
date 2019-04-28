function [alpha_seg] = learnSlip(Uls,Urs, vals, type)
%LEARNSLIP Solves for the alpha vector that maps:
% min ||vals - [
% Solves for alpha vector of general kinematic slip model. 
input_speed = .5*(Uls + Urs);
axel_len = .62; %axel len shows up as omega_alpha_omega
input_omega = (Urs - Uls)/axel_len;
if (type == 'dx')
   input_omega = abs(input_omega);
   b = input_speed;
elseif (type == 'dy')
   b = 0;
else % type == omega
   b = input_omega;
end

bigA = [input_speed, input_omega, input_speed.*input_omega];
alpha_seg = (bigA'*bigA)^-1 * bigA' *(vals-b);

if (type ~= 'dx')
  otherBigA = [input_speed, -input_omega, -input_speed.*input_omega];
  other_alpha_seg = (otherBigA'*otherBigA)^-1 * otherBigA' *(-(vals-b));
  alpha_seg = (alpha_seg + other_alpha_seg)/2.0;
end
end

