%%generate curve, omega_dx, path_fwd (workspace variables for simulation)
dt = .01;
t = 0:dt:1;

axel_len = .62;
learning_rate = .2;
%initial parameter guess:
if (true)
disp("alpha reset to 0") %this below is alpha^T (each col of alpha is vec)
alpha = [0, 0, 0; %a_x [speed, |omega|, speed*|omega|] + speed = speed_gks
         0, 0, 0; %a_y [speed, omega, speed*omega] + speed     = side_gks
         0, 0, 0];%a_om[""]                        + omega     = omega_gks
else
disp("alpha trained to:")
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
time_to_solve = 0:delta_time:8;
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
plot_time = ControlOut.Time;
figure()
ax = gca;
ax.FontSize = 24; 
OtherFsize = 18;
%plot robot and path
subplot(3, 2, 1:2:3)
hold on;
title('Ideal Path and Robot Trajectory', 'FontSize',OtherFsize);
plot(curve(1,:), curve(2,:), 'b--');
mymap = colormap(lines(length(delta_accum.Data(:,1))));
scatter(delta_accum.Data(:,1), delta_accum.Data(:,2), 40, mymap);
%plot(delta_accum.Data(:,1), delta_accum.Data(:,2), 'gd');
legend('Ideal Path', 'Robot Path', 'FontSize',OtherFsize);
xlabel('X (m)', 'FontSize',OtherFsize)
ylabel('Y (m)', 'FontSize',OtherFsize)

xlim([0, 7])
ylim([-3, 3])
%plot wheels
subplot(3,2,2)
hold on;
title('Wheel Velocities', 'FontSize',OtherFsize);
plot(plot_time, Uls);
plot(plot_time, Urs);
ylabel('Wheel Vel (m/s)', 'FontSize',OtherFsize);
xlabel('Time (s)', 'FontSize',OtherFsize);
ylim([-1.3, 1.3]);
legend('Uls', 'Urs', 'FontSize',OtherFsize);
subplot(3,2,4);
hold on;
title('Control States', 'FontSize',OtherFsize);
plot(plot_time,ControlOut.Data(:,3)/100.0);
plot(plot_time,ControlOut.Data(:,4));
plot(plot_time,ControlOut.Data(:,5));
xlabel('Time (s)', 'FontSize',OtherFsize);
ylim([-1, 1]);
legend('path param [0,1]', 'path err (m)', 'angle err (rad)', 'FontSize',OtherFsize);
subplot(3,2,5:6)
hold on;
title('System Parameter Estimation', 'FontSize',OtherFsize);
ylabel('Coefficient value', 'FontSize',OtherFsize)
xlabel('Index', 'FontSize',OtherFsize);
stem(alpha(:), 'go');
stem(alpha_episode(:), 'r*');
legend('current alpha', 'calc alpha', 'FontSize',OtherFsize);
%repeat.