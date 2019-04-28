begining_time = 1
%project dx,dy onto forward vector for speed
%     f_vec = [cos(theta);sin(theta)]
% orthogonal vector for disturbance
%robot_theta = new_est_rec(3,begining_time:end);
%run after run_model.m, learns the slip model of the system from sensor and
%command history data
local_speeds = 0;
local_dist = 0;
for index = begining_time:length(state_rec)
    robot_theta = new_est_rec(3,index);
    local_speeds(index - begining_time + 1,1) = [cos(robot_theta)', sin(robot_theta)'] * [state_rec(4,index);...
                                                                    state_rec(5,index)];
    local_dist(index - begining_time +1,1) = [-sin(robot_theta)',  cos(robot_theta)'] * [state_rec(4,index);...
                                                                     state_rec(5,index)];    
end
local_omega = (1/axel_len)*(state_rec(9,begining_time:end) - state_rec(8,begining_time:end))';%new_est_rec(6,begining_time:end)';
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
           
if (true) %(re)init record vars
    btimes = [];
    alpha_d_rec = [];
    alpha_om_rec = [];
    alpha_x_rec = [];
end
                
%lazy record
btimes = [btimes, begining_time];
alpha_x_rec = [alpha_x_rec, alpha_x];
alpha_d_rec = [alpha_d_rec, alpha_d];
alpha_om_rec = [alpha_om_rec, alpha_om];