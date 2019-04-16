rng(1776)
axel_len = .62;
%dt = .02;
%time_to_solve = 0:dt:10;
delta_time = .02;
total_time = 20;%480;
time_to_solve = [0:delta_time:total_time];
left_cmd = zeros(1,length(time_to_solve));
right_cmd = zeros(1,length(time_to_solve));
period_1 = 10;%total_time / 2;
period_2 = 5;%period_1 / 12;

for index = 1:length(time_to_solve)
    if (time_to_solve(index) < period_1) 
      left_cmd(index) = .5*sin((2*pi / period_2)*time_to_solve(index));%.5 - .5*exp(-2*time_to_solve);
      right_cmd(index) = .5*sin((2*pi / period_1)*time_to_solve(index));%.2 - .2*exp(-2*time_to_solve);%sin(time_to_solve);
    else
      left_cmd(index) = .5*sin((2*pi / period_1)*time_to_solve(index));%.5 - .5*exp(-2*time_to_solve);
      right_cmd(index) = .5*sin((2*pi / period_2)*time_to_solve(index));%.2 - .2*exp(-2*time_to_solve);%sin(time_to_solve);
    end
end

%send commands through lead compensator
[A, B, C, D] = tf2ss([1,4],[1,20]); %move pole at -4 to -20
c2d(ss(A,B,C,D), .02)
state = [0;0];
output_record = zeros(2,length(time_to_solve));
for index = 1:length(time_to_solve)
    state = .6703 * state + .01648 * [left_cmd(index);right_cmd(index)];
    output_record(:,index) = 5*(-16*state + 1*[left_cmd(index);right_cmd(index)]);
end
%use lead compensated commands on system
left_wheel_cmd_vels  = [time_to_solve' , output_record(1,:)'];%left_cmd']; %packaged for simulink
right_wheel_cmd_vels = [time_to_solve' , output_record(2,:)'];%right_cmd'];


GGG = simset('Solver', 'ode4', 'FixedStep', delta_time)
sim('robotdynamic_simulink.slx', [0,time_to_solve(end)], GGG)

%figure();
%subplot(2,1,1);
%plot(delta_accum)
%legend('x', 'y', 'theta', 'Ul', 'Ur')
%subplot(2,1,2);
%plot(measurements)
%legend('Ul_{cmd}', 'Ur_{cmd}', 'd2x', 'd2y', 'omega', 'x', 'y', 'theta', 'Ul_{read}', 'Ur_{read}')

%TODO add IMU offset and transform from local to global coord
%and add wandering IMU bias
%add noise to measurements
%maybe this is std. dev instead
IMUx_variance = .01;
IMUy_variance = .01;
IMUomega_variance = .01;

POSx_variance = .05;
POSy_variance = .05;
POStheta_variance = .05;

Ul_read_variance = .002;
Ur_read_variance = .002;


time_len = size(measurements.Data);
%give estimator input commands as planned
Ul_cmd_rec = left_cmd';%measurements.Data(:,1); %know input exactly
Ur_cmd_rec = right_cmd';%measurements.Data(:,2);

d2x_rec = measurements.Data(:,3) + IMUx_variance * randn(time_len(1), 1);
d2y_rec = measurements.Data(:,4) + IMUy_variance * randn(time_len(1), 1);
omega_rec = measurements.Data(:,5) + IMUomega_variance*randn(time_len(1), 1);

x_rec = measurements.Data(:,6) + POSx_variance *  randn(time_len(1), 1);
y_rec = measurements.Data(:,7) + POSy_variance *  randn(time_len(1), 1);
theta_rec = measurements.Data(:,8) + POStheta_variance *  randn(time_len(1), 1);
Ul_read_rec = measurements.Data(:,9) + Ul_read_variance * randn(time_len(1), 1);
Ur_read_rec = measurements.Data(:,10) + Ur_read_variance * randn(time_len(1),1);


%plot(delta_accum.Time, delta_accum.Data(:,3))
%plot(delta_accum.Time, delta_accum.Data(:,4))

%recursivly build estimate of true states x, y, theta, dx, dy, omega,
%and effective Ul, Ur (wheel velocity - slip)

 new_est_rec = zeros(16, time_len(1));
x0  = 0;  y0 = 0; th0 = 0;
dx0 = 0; dy0 = 0; om0 = 0;
Ul0 = 0; Ur0 = 0;
Sl0 = 0; Sr0 = 0;
d2x = 0; d2y = 0;
bx  = 0;  by = 0;
Ul_acc = 0; Ur_acc = 0;
 new_est_prev = [x0, y0, th0, dx0, dy0, om0, Ul0, Ur0, Sl0, Sr0, d2x, d2y, bx, by,Ul_acc, Ur_acc ];

vel_window_size = 3;
prev_mea_pos_imu = zeros(4, vel_window_size); %populate x0 here
p_v_and_b_buff = zeros(6,vel_window_size); %[pos; velocity for begining of fit
                                         %and imu offset
for index = 1:time_len
  meas = [d2x_rec(index), d2y_rec(index), omega_rec(index),...
          x_rec(index),     y_rec(index), theta_rec(index),...
          Ul_read_rec(index), Ur_read_rec(index)];
  %need to pop p0 and pass in to propagate
  prev_mea_pos_imu(:,1:end-1) = prev_mea_pos_imu(:,2:end); %scoot for new data
  prev_mea_pos_imu(:,end) = [new_est_prev(1);new_est_prev(2);
                             new_est_prev(11); new_est_prev(12)];
  new_est_rec(:,index) = propagateEstimate([Ul_cmd_rec(index), Ur_cmd_rec(index)],...
                                       new_est_prev, meas, delta_time,...
                                       prev_mea_pos_imu, p_v_and_b_buff(:,1));
  new_est_prev = new_est_rec(:,index);
  p_v_and_b_buff(:,1:end-1) = p_v_and_b_buff(:,2:end);
  p_v_and_b_buff(:,end) = [new_est_prev(1);new_est_prev(2); %position
                           new_est_prev(4);new_est_prev(5); %velocity
                           new_est_prev(13);new_est_prev(14)]; %imu offset
end

%subplot(2,1,2)
figure();
hold on;
plot(measurements.Time, x_rec, 'b*', 'MarkerSize', 1)
plot(measurements.Time, y_rec, 'r*', 'MarkerSize', 1)
plot(measurements.Time, theta_rec, 'c*', 'MarkerSize', 1)
plot(measurements.Time, omega_rec, 'g*', 'MarkerSize', 1)
plot(measurements.Time, d2x_rec, '*', 'MarkerSize', 1,'MarkerFaceColor',[.2 .4 .6])
plot(measurements.Time, d2y_rec, '*', 'MarkerSize', 1,'MarkerFaceColor',[.6 .2 .4])
plot(measurements.Time, new_est_rec(1,:), 'b--')
plot(measurements.Time, new_est_rec(2,:), 'r--')
plot(measurements.Time, new_est_rec(3,:), 'c--')
plot(measurements.Time, new_est_rec(6,:), 'g--')
plot(measurements.Time, new_est_rec(4,:), '--', 'MarkerFaceColor',[.2 .4 .6])
plot(measurements.Time, new_est_rec(5,:), '--', 'MarkerFaceColor',[.6 .2 .4])
plot(measurements.Time, new_est_rec(11,:), '--', 'MarkerFaceColor', [.4, .6, .6]);
plot(measurements.Time, new_est_rec(12,:), '--', 'MarkerFaceColor', [.6, .6, .4]);
plot(measurements.Time, delta_accum.Data(:, 1), 'b-')
plot(measurements.Time, delta_accum.Data(:, 2), 'r-')
plot(measurements.Time, delta_accum.Data(:, 3), 'c-')
legend('x_{mea}', 'y_{mea}', 'th_{mea}', 'om_{mea}',...
       'd2x_{mea}', 'd2y_{mea}',...
       'x_{est}', 'y_{est}', 'th_{est}', 'om_{est}',...
       'dx_{est}', 'dy_{est}', 'd2x_{est}', 'd2y_{est}',...
       'x_{tru}', 'y_{tru}', 'th_{tru}');

figure();
hold on;
plot(measurements.Time, Ul_read_rec, '*', 'MarkerSize', 1,'MarkerFaceColor',[.2 .4 .6])
plot(measurements.Time, Ur_read_rec, '*', 'MarkerSize', 1,'MarkerFaceColor',[.6 .2 .4])
plot(measurements.Time, new_est_rec(7,:), '--', 'MarkerFaceColor',[.2 .4 .6])
plot(measurements.Time, new_est_rec(8,:), '--', 'MarkerFaceColor',[.6 .2 .4])
plot(measurements.Time, delta_accum.Data(:, 4), '-', 'MarkerFaceColor',[.2 .4 .6])
plot(measurements.Time, delta_accum.Data(:, 5), '-', 'MarkerFaceColor',[.6 .2 .4])
plot(measurements.Time, left_cmd, 'd', 'MarkerFaceColor',[.9 .4 .6])
plot(measurements.Time, right_cmd, 'd', 'MarkerFaceColor',[.9 .6 .4])

%plot new_est vs time.




