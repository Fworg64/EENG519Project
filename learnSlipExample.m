begining_time = 430

local_speeds = (cos(new_est_rec(3,begining_time:end)).*new_est_rec(4,begining_time:end)...
              + sin(new_est_rec(3,begining_time:end)).*new_est_rec(5,begining_time:end))';
local_dist = (-sin(new_est_rec(3,begining_time:end)).*new_est_rec(4,begining_time:end)...
              + cos(new_est_rec(3,begining_time:end)).*new_est_rec(5,begining_time:end))';
local_omega = new_est_rec(6,begining_time:end)';
Uls = left_cmd(begining_time:end)';%new_est_rec(7,begining_time:end)';
Urs = right_cmd(begining_time:end)'; %new_est_rec(8,begining_time:end)';
%Uls = left_cmd(begining_time:end)';%new_est_rec(7,begining_time:end)';
%Urs = right_cmd(begining_time:end)'; %new_est_rec(8,begining_time:end)';
alpha_x = learnSlip(Uls, Urs,...
                    local_speeds, 'dx')
alpha_d = learnSlip(Uls, Urs,...
                    local_dist, 'dy')
alpha_om = learnSlip(Uls, Urs,...
                    local_omega, 'om')