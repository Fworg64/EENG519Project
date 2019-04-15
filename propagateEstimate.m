function state_estimate = propagateEstimate(inputs, prev_est, meas, dt,...
                                            prev_mea_pos_imu, p_v_and_b)
%propagateEstimate
% given the current inputs (wheel velocitiy commands): [Ul; Ur],
% the previous estimate of the state:
%  [x;  y;  theta;
%   dx; dy; omega;
%   Ul_act; Ur_act;
%   Slip_l; Slip_r;
%   d2x;    d2y;
%    Bx;     By;];
% and the measurement vector: 
%  [d2x; d2y; omega;
%   x;     y; theta;
%   Ul_mea;  Ur_mea]
% and the delta_time: dt
% and a record of the previous position and imu measurements:
% [x_n-5, x_n-4, ..., x_n;
%  y ...; 
%  d2x ...;
%  d2y ...];
% and the estimated positions, velocity and imu offset 
%                       for the begining of those meas
% [Px
%  Py
%  Vx;
%  Vy;
%  Bx;
%  By];

% returns a new state estimate which should be the MAP estimate (MMSE?)

%effective = velocity given to vehicle from wheel
%actual    = actual speed of wheel (might be slipping)
%cmd       = velocity sent to controller

%for Ur_eff and Ul_eff
% Ux_eff = Ux_act - slip
% Ux_eff_hat = Ux_act_est - slip_est
% Ux_act_est = [Ux_act_est + K(Ux_mea - Ux_act_est)] ...
%              + (dUx/ dt) * dt <-control input accel (known)
% slip_est   = ? comes from slip model? actual - effective

%K=0;
Kx = .051;
Ky = .051;
Kth = .2;%0;%.8;
Kdx = .5;
Kdy = .5;
Kom = .5;
KUl = .4;
KUr = .4;
Kd2x = .5;
Kd2y = .5;
%KSl = .5;
%KSr = .5;

%plus puts you forward in time, right before next measurement
%review MAP estimate lecture
%receding horizon
%for now, slip_est = 0
Ul = inputs(1); %known (what we asked the wheels to do)
Ur = inputs(2); %known
Ul_hat_prev = prev_est(7); %estimate actual wheel speed here
Ur_hat_prev = prev_est(8);
%difference of inputs requires reasonable inputs (no big jumps asked)
Ul_accel = (Ul - prev_est(7))/dt;%should use center(or forward) difference of inputs
Ur_accel = (Ur - prev_est(8))/dt;% using back difference for now
Ul_accel = max(-.4, min(Ul_accel, .4)); %clamp for now too
Ur_accel = max(-.4, min(Ur_accel, .4));
prev_Ul_accel = .9*Ul_accel;
prev_Ur_accel = .9*Ur_accel;

Ul_mea = meas(7);
Ur_mea = meas(8);%                          \/ is this right?
% Ul_hat_plus = Ul_hat_prev + KUl*(Ul_mea - (Ul_hat_prev)); %measurement update
% Ur_hat_plus = Ur_hat_prev + KUr*(Ur_mea - (Ur_hat_prev)); %
% Ul_hat_plus = Ul_hat_plus + Ul_accel*dt; %time update
% Ur_hat_plus = Ur_hat_plus + Ur_accel*dt;
Ul_hat = Ul_hat_prev + KUl*(Ul_mea - (Ul_hat_prev + Ul_accel*dt)); %measurement update
Ur_hat = Ur_hat_prev + KUr*(Ur_mea - (Ur_hat_prev + Ur_accel*dt)); %
Ul_hat_plus = Ul_hat + Ul_accel*dt; %time update
Ur_hat_plus = Ur_hat + Ur_accel*dt;
Slip_l = 0; %should try to estimate these (Use learned slip model)
Slip_r = 0;
Ul_eff_hat = (Ul_hat_plus - Slip_l);
Ur_eff_hat = (Ur_hat_plus - Slip_r);

%for omega, omega = (Ur_eff - Ul_eff)/AxelLen
%so need pomega / pUr and pomega / pUl
%and 
  AxelLen = .62;
 omega_mea = meas(3);
 %omega_model_est = (Ur_eff_hat - Ul_eff_hat)/AxelLen; 
 omega_est_prev = prev_est(6); %is this used?
 omega_est = omega_est_prev + Kom*(omega_mea - (omega_est_prev + dt*(Ur_accel - Ul_accel)/AxelLen));
 %omega_est_plus = omega_est + omega_model_est;
 omega_est_plus = omega_est + dt*(Ur_accel - Ul_accel)/AxelLen;
% omega_est_plus = omega_est + (Ur_eff_hat + (dUr)*dt - Ul_eff_hat - (dUl)*dt)/AxelLen 

 theta_mea = meas(6);
 theta_est = prev_est(3);
 theta_est = theta_est + Kth*(angleDiff(theta_mea,theta_est + omega_est_plus*dt));
 theta_est = theta_est + omega_est_plus*dt; %theta_est_plus
 theta_est = angleDiff(theta_est,0);
% ^^ need care for angle wrap-around

%dx,dy are local dx,dy (from wheel vels) transformed by theta
%need instant turn radius, world rot mat, then dPos:
%  R = AxelLen/2 * (Ur + Ul)/(Ur - Ul);
%  rot = [cos(w*dt), -sin(w*dt);sin(w*dt),cos(w*dt)];
%  dPos = rot*[0;-R] + [0;R];
%so using our estimates:
 R_est = AxelLen/2 * (Ur_eff_hat + Ul_eff_hat) / (Ur_eff_hat - Ul_eff_hat);
%  ^^ Need care for zero point turns (Ur = Ul)
%  rot_est: use omega_est * dt for w*dt

rot_est = [cos(omega_est_plus*dt), -sin(omega_est_plus*dt);
           sin(omega_est_plus*dt),  cos(omega_est_plus*dt)];
       
deltaPos_est = rot_est*[0;-R_est] + [0;R_est]; %local coord
%TODO ^ add side disturbance velocity from slip model to local y
wrot = [cos(theta_est), -sin(theta_est);
        sin(theta_est),  cos(theta_est)];
deltaPosW_est = wrot*deltaPos_est;

%x and y then:

 d2x_est = prev_est(11);
 d2y_est = prev_est(12);
 d2x_mea = meas(1);
 d2y_mea = meas(2);
 x_accel_expected_increase = cos(theta_est)*dt*.5*((Ul_accel - prev_Ul_accel)  +(Ur_accel - prev_Ur_accel));
 y_accel_expected_increase = sin(theta_est)*dt*.5*((Ul_accel - prev_Ul_accel)  +(Ur_accel - prev_Ur_accel));
 d2x_est = d2x_est + Kd2x*(d2x_mea - (d2x_est + x_accel_expected_increase)); %measurement update
 d2y_est = d2y_est + Kd2y*(d2y_mea - (d2y_est + y_accel_expected_increase)); %measurement update
 d2x_est_plus = d2x_est + x_accel_expected_increase; %time update
 d2y_est_plus = d2y_est + y_accel_expected_increase; %time update
 
 %build least squares sensor value estimate
 window_len = size(prev_mea_pos_imu);
 imu_A = [eye(window_len(2),window_len(2)), ones(window_len(2),1)];
 for index = 1:window_len(2)-1
     imu_A(index+1,index) = -1;
 end
 pos_A = dt * eye(window_len(2), window_len(2)+1);
 offset_gain = 1;
 off_A = [zeros(1, window_len(2)),offset_gain];
 big_A = [imu_A;pos_A;off_A]; %d2x,d2y,x,y,bx,by
 
 big_bx = [dt*prev_mea_pos_imu(3,1) + p_v_and_b(3); %imu and v0
           dt*prev_mea_pos_imu(3,2:end)';         %imu
           prev_mea_pos_imu(1,1) - p_v_and_b(1);
           prev_mea_pos_imu(1,2:end)' - prev_mea_pos_imu(1,1:end-1)'; %dpos
           offset_gain*p_v_and_b(5)];                           %imu offset
 big_by = [dt*prev_mea_pos_imu(4,1) + p_v_and_b(4);
           dt*prev_mea_pos_imu(4,2:end)';
           prev_mea_pos_imu(2,1) - p_v_and_b(2);
           prev_mea_pos_imu(2,2:end)' - prev_mea_pos_imu(2,1:end-1)';
           offset_gain*p_v_and_b(6)];
 new_mea_x = (big_A'*big_A) \ big_A' * big_bx; %least squares from meas
 new_mea_y = (big_A'*big_A) \ big_A' * big_by;
 dx_mea = new_mea_x(window_len(2)); %ultimate velocity from soln
 dy_mea = new_mea_y(window_len(2));
 Bx = new_mea_x(end);
 By = new_mea_y(end);
 dx_est = prev_est(4);
 dy_est = prev_est(5);
 %we dont have a true dx_mea or dy_mea?
 dx_est = dx_est + Kdx*(dx_mea - (dx_est + d2x_est_plus*dt ));
 dy_est = dy_est + Kdy*(dy_mea - (dy_est + d2y_est_plus*dt ));

 dx_est = dx_est + d2x_est_plus*dt; % deltaPosW_est(1)/dt;
 dy_est = dy_est + d2y_est_plus*dt; %  deltaPosW_est(2)/dt;
 
 x_mea = meas(4);
 y_mea = meas(5);
 x_est = prev_est(1);
 y_est = prev_est(2);
 x_est = x_est + Kx*(x_mea - (x_est + deltaPosW_est(1)));
 x_est = x_est + deltaPosW_est(1);
 y_est = y_est + Ky*(y_mea - (y_est + deltaPosW_est(2)));
 y_est = y_est + deltaPosW_est(2);
 
 state_estimate = [x_est;  y_est;  theta_est;
                   dx_est; dy_est; omega_est_plus;
                   Ul_hat_plus;  Ur_hat_plus;
                   Slip_l; Slip_r;
                   d2x_est_plus;d2y_est_plus;
                   Bx;By];
 
% similar for y




end