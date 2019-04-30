%%generate curve, omega_dx, path_fwd (workspace variables for simulation)
dt = .01;
t = 0:dt:1;

axel_len = .62;
learning_rate = .08;
%initial parameter guess:
if (reset_learning) %define this var outside
iterations = 0;
disp("alpha reset to 0") %this below is alpha^T (each col of alpha is vec)
alpha = [0, 0, 0; %a_x [speed, |omega|, speed*|omega|] + speed = speed_gks
         0, 0, 0; %a_y [speed, omega, speed*omega] + speed     = side_gks
         0, 0, 0];%a_om[""]                        + omega     = omega_gks
else
iterations = iterations + 1;
disp("alpha trained to:")
alpha = (1-learning_rate)*alpha + (learning_rate)*alpha_episode;
disp(alpha);
end
%define start and end
P0 = [0;0];
angle1 = 0;
dist1 = 2.5;
P3 = [6; 2];%*cos(iterations)];
dist2 = 1.8;
angle2 = -sin(iterations);

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
rmsperr = sqrt(sum(ControlOut.Data(:,4).^2));
disp(rmsperr);
%adjust control parameters
%happens at begining of next iteration
reset_learning = false;
%print
plot_time = ControlOut.Time;
figure()
OtherFsize = 16;
ticksize = 18;
%plot robot and path
subplot(3, 2, 1:2:3)
ax = gca;
ax.FontSize = ticksize; 
hold on;
title(sprintf('Ideal Path and Robot Trajectory, RMS path err: %.4f',rmsperr), 'FontSize',OtherFsize);
plot(curve(1,:), curve(2,:), 'b--', 'LineWidth',3);
mymap = colormap(lines(length(delta_accum.Data(:,1))));
scatter(delta_accum.Data(:,1), delta_accum.Data(:,2), 40, mymap);
%plot(delta_accum.Data(:,1), delta_accum.Data(:,2), 'gd');
legend('Ideal Path', 'Robot Path', 'FontSize',OtherFsize, 'Location', 'southeast');
xlabel('X (m)', 'FontSize',OtherFsize)
ylabel('Y (m)', 'FontSize',OtherFsize)

xlim([0, 7])
ylim([-3, 3])
%plot wheels
subplot(3,2,2)
ax = gca;
ax.FontSize = ticksize; 
hold on;
title('Wheel Velocities', 'FontSize',OtherFsize);
plot(plot_time, Uls, 'LineWidth',3);
plot(plot_time, Urs, 'LineWidth',3);
ylabel('Wheel Vel (m/s)', 'FontSize',OtherFsize);
xlabel('Time (s)', 'FontSize',OtherFsize);
ylim([-1.3, 1.3]);
legend('Uls', 'Urs', 'FontSize',OtherFsize, 'Location', 'southeast');
subplot(3,2,4);
ax = gca;
ax.FontSize = ticksize; 
hold on;
title('Control States', 'FontSize',OtherFsize);
plot(plot_time,ControlOut.Data(:,3)/100.0, 'LineWidth',3);
plot(plot_time,ControlOut.Data(:,4), 'LineWidth',3);
plot(plot_time,ControlOut.Data(:,5), 'LineWidth',3);
xlabel('Time (s)', 'FontSize',OtherFsize);
ylim([-1, 1]);
legend('path param [0,1]', 'path err (m)', 'angle err (rad)', 'FontSize',OtherFsize, ...
    'Location', 'southeast');
subplot(3,2,5:6)
ax = gca;
ax.FontSize = ticksize; 
hold on;
titstr = sprintf('System Parameter Estimation, k=%d, learning rate=%.2f', iterations, learning_rate);
title(titstr, 'FontSize',OtherFsize);
ylabel('Coefficient value', 'FontSize',OtherFsize)
xlabel('Index', 'FontSize',OtherFsize);
stem(alpha(:), 'go', 'LineWidth',3);
stem(alpha_episode(:), 'r*', 'LineWidth',3);
ylim([-1.4, 1.4]);
legend('current alpha', 'calc alpha', 'FontSize',OtherFsize, 'Location', 'southeast');
%repeat.
%set(gcf, 'Position', get(0, 'Screensize'));
set(gcf,'WindowState','maximized')
filestr = sprintf('printout/bettergainsnolearndiffangleiter%d', iterations);
print(filestr, '-dpng');