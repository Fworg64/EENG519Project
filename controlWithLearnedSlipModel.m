%%generate curve, omega_dx, path_fwd (workspace variables for simulation)
dt = .01;
t = 0:dt:1;

axel_len = .62;

%define start and end
P0 = [0;0];
angle1 = 0;
dist1 = .5;
P3 = [6; 2];
dist2 = .4;
angle2 = 0;

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

%adjust control parameters

%print
figure();
hold on;
plot(curve(1,:), curve(2,:), 'b--');
plot(delta_accum.Data(:,1), delta_accum.Data(:,2), 'gd');
%repeat.