axel_len = .62;
%dt = .02;
%time_to_solve = 0:dt:10;
delta_time = .02;
time_to_solve = [0, 10];
GGG = simset('Solver', 'ode4', 'FixedStep', delta_time)
sim('robotdynamic_simulink.slx', time_to_solve, GGG)

figure();
subplot(2,1,1);
plot(delta_accum)
legend('x', 'y', 'theta', 'Ul', 'Ur')
%subplot(2,1,2);
%plot(measurements)
legend('Ul_{cmd}', 'Ur_{cmd}', 'd2x', 'd2y', 'omega', 'x', 'y', 'theta', 'Ul_{read}', 'Ur_{read}')

%TODO add IMU offset and transform from local to global coord
%add noise to measurements
IMUx_variance = .1;
IMUy_variance = .1;
IMUomega_variance = .1;

POSx_variance = .1;
POSy_variance = .1;
POStheta_variance = .1;

Ul_read_variance = .1;
Ur_read_variance = .1;


time_len = size(measurements.Data)
Ul_cmd_rec = measurements.Data(:,1); %know input exactly
Ur_cmd_rec = measurements.Data(:,2);

d2x_rec = measurements.Data(:,3) + IMUx_variance * randn(time_len(1), 1);
d2y_rec = measurements.Data(:,4) + IMUx_variance * randn(time_len(1), 1);
omega_rec = measurements.Data(:,5) + IMUomega_variance*randn(time_len(1), 1);

x_rec = measurements.Data(:,6) + POSx_variance *  randn(time_len(1), 1);
y_rec = measurements.Data(:,7) + POSy_variance *  randn(time_len(1), 1);
theta_rec = measurements.Data(:,8) + POStheta_variance *  randn(time_len(1), 1);
Ul_read_rec = measurements.Data(:,9) + Ul_read_variance * randn(time_len(1), 1);
Ur_read_rec = measurements.Data(:,10) + Ur_read_variance * randn(time_len(1),1);

subplot(2,1,2)
hold on;
plot(measurements.Time, x_rec)
plot(measurements.Time, y_rec)
plot(delta_accum.Time, delta_accum.Data(:,3))
plot(delta_accum.Time, delta_accum.Data(:,4))



%recursivly build estimate of true states x, y, theta, dx, dy, omega,
%and effective Ul, Ur (wheel velocity - slip)

% new_est_rec = zeros(6(or8?), time_len)
%for index = 1:time_len
%  new_est(:,index) = propagateEstimate([Ul, Ur], new_est, meas, delta_time)
%end

%plot new_est vs time.




