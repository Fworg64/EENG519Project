function [alpha_seg] = learnSlip(Uls,Urs, vals, type)
%LEARNSLIP Solves for the alpha vector that maps:
% min ||vals - [
% Solves for alpha vector of general kinematic slip model. 
input_speed = .5*(Uls + Urs);
input_omega = Urs - Uls; %forget about axel len?
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
end

