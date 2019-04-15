begining_time = 100
%project dx,dy onto forward vector for speed
%     f_vec = [cos(theta);sin(theta)]
% orthogonal vector for disturbance
%robot_theta = new_est_rec(3,begining_time:end);
local_speeds = 0;
local_dist = 0;
for index = begining_time:length(x_rec)
    robot_theta = new_est_rec(3,index);
    local_speeds(index - begining_time + 1,1) = [cos(robot_theta)', sin(robot_theta)'] * [new_est_rec(4,index);...
                                                                    new_est_rec(5,index)];
    local_dist(index - begining_time +1,1) = [-sin(robot_theta)',  cos(robot_theta)'] * [new_est_rec(4,index);...
                                                                     new_est_rec(5,index)];    
end
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
                
                
%lazy record
btimes = [btimes, begining_time];
alpha_x_rec = [alpha_x_rec, alpha_x];
alpha_d_rec = [alpha_d_rec, alpha_d];
alpha_om_rec = [alpha_om_rec, alpha_om];