%%generate curve, omega_dx, path_fwd (workspace variables for simulation)
dt = .01;
t = 0:dt:1;

axel_len = .62;
learning_rate = .3;
%initial parameter guess:
if (false)
disp("alpha reset to 0") %this below is alpha^T (each col of alpha is vec)
alpha = [0, 0, 0; %a_x [speed, |omega|, speed*|omega|] + speed = speed_gks
         0, 0, 0; %a_y [speed, omega, speed*omega] + speed     = side_gks
         0, 0, 0];%a_om[""]                        + omega     = omega_gks
else
%alpha(1:3) = alpha_x;
%alpha(7:9) = alpha_om;
disp("alpha trained to:")
%alpha(1:3) = (1-learning_rate)*alpha(1:3) + (learning_rate)*alpha_x';
%alpha(7:9) = (1-learning_rate)*alpha(7:9) + (learning_rate)*alpha_om';
alpha = (1-learning_rate)*alpha + (learning_rate)*alpha_episode;
disp(alpha);
end
%define start and end
P0 = [0;0];
angle1 = 0;
dist1 = 2.5;
P3 = [6; 2];
dist2 = 1.8;
angle2 = -.5;

P1 = P0 + dist1*[cos(angle1);sin(angle1)];
P2 = P3 - dist2*[cos(angle2);sin(angle2)];

curve = (1 - t).^3 .* P0 ...
        + 3*(1-t).^2 .*t .* P1 ...
        + 3*(1-t).*t.^2 .* P2 ...
        + t.^3 .* P3;
    
first_der = 3*(1 - t).^2 .*(P1 - P0) ...
          + 6*(1-t).*t.*(P2 - P1) ...
          + 3*t.^2.*(P3 - P2);
theta = atan2(first_der(2,:), first_der(1,:));
delta_theta_delta_t = angleDiff(theta(2:end), theta(1:(length(theta)-1)));
t_off = t(1:length(t)-1) + dt/2;
%distance via nasty piecewise integration
curve_length = 0;
delta_x_delta_t = zeros(1,length(t)-1);
for index = 2:length(t)
  delta_x_delta_t(1,index-1) = norm(curve(:,index) - curve(:,index-1),2);
end
curve_length = sum(delta_x_delta_t);
%this is rate of change of theta at each t_off
omega_dx = delta_theta_delta_t ./ delta_x_delta_t;
path_fwd = 1;
%%run simulink sys
delta_time = .02;
time_to_solve = 0:delta_time:15;
%coder.extrinsic('PathControl');
%coder.extrinsic('findCPP2019Spring');
%coder.extrinsic('sscv2019Spring');

GGG = simset('Solver', 'ode4', 'FixedStep', delta_time);
sim('robotdynamic_path_simulink.slx', [0,time_to_solve(end)], GGG)
%learn slip model from measurements/results
local_speeds = 0;
local_dist = 0;
for index = 1:length(VelOut.Data(:,1))
    robot_theta = delta_accum.Data(index, 3);
    local_speeds(index,1) = ...
        [cos(robot_theta)', sin(robot_theta)'] * [VelOut.Data(index,1);...
                                                 VelOut.Data(index, 2)];
    local_dist(index,1) = ...
        [-sin(robot_theta)',  cos(robot_theta)'] * [VelOut.Data(index,1);...
                                                    VelOut.Data(index, 2)];   
end
Uls = ControlOut.Data(:, 1);%new_est_rec(7,begining_time:end)';
Urs = ControlOut.Data(:,2); %TODO investigate lead compensator
local_omega = VelOut.Data(:,3);%(1/axel_len)*(Urs - Uls);%new_est_rec(6,begining_time:end)';
alpha_x = learnSlip(Uls, Urs,...
                    local_speeds, 'dx');
alpha_d = learnSlip(Uls, Urs,...
                    local_dist, 'dy');
alpha_om = learnSlip(Uls, Urs,...
                    local_omega, 'om');
disp("Alpha from episode:")
alpha_episode = [alpha_x, alpha_d, alpha_om];
disp(alpha_episode);
disp("RMS Path Err:");
disp(sqrt(sum(ControlOut.Data(:,4).^2)));
%adjust control parameters
%happens at begining of next iteration
%print
figure();
subplot(2, 1, 1)
hold on;
plot(curve(1,:), curve(2,:), 'b--');
plot(delta_accum.Data(:,1), delta_accum.Data(:,2), 'gd');
xlim([0, 8])
ylim([-3, 3])
subplot(2,1,2)
hold on;
plot(Uls);
plot(Urs);
ylim([-1.3, 1.3]);
legend('Uls', 'Urs');
hold on;
%repeat.