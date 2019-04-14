function state_estimate = propagateEstimate(inputs, prev_est, meas, dt)
%propagateEstimate
% given the current inputs (wheel velocitiy commands): [Ul; Ur],
% the previous estimate of the state:
%  [x;  y;  theta;
%   dx; dy; omega;
%   Ul_act; Ur_act;
%   Slip_l; Slip_r;
%   d2x;    d2y];
% and the measurement vector: 
%  [d2x; d2y; omega;
%   x;     y; theta;
%   Ul_mea;  Ur_mea]
% and the delta_time: dt

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
Kdx = .8;
Kdy = .8;
Kom = .8;
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

 d2x_est = prev_est(9);
 d2y_est = prev_est(10);
 d2x_mea = meas(1);
 d2y_mea = meas(2);
 x_accel_expected_increase = cos(theta_est)*dt*.5*((Ul_accel - prev_Ul_accel)  +(Ur_accel - prev_Ur_accel));
 y_accel_expected_increase = sin(theta_est)*dt*.5*((Ul_accel - prev_Ul_accel)  +(Ur_accel - prev_Ur_accel));
 d2x_est = d2x_est + Kd2x*(d2x_mea - (d2x_est + x_accel_expected_increase)); %measurement update
 d2y_est = d2y_est + Kd2y*(d2y_mea - (d2y_est + y_accel_expected_increase)); %measurement update
 d2x_est_plus = d2x_est + x_accel_expected_increase; %time update
 d2y_est_plus = d2y_est + y_accel_expected_increase; %time update
 
 dx_mea = deltaPosW_est(1)/dt; %this should come from sensor data
 dy_mea = deltaPosW_est(2)/dt;
 dx_est = prev_est(4);
 dy_est = prev_est(5);
 %we dont have a true dx_mea or dy_mea?
 dx_est = dx_est + Kdx*(dx_mea - (dx_est + d2x_est_plus*dt));
 dy_est = dy_est + Kdy*(dy_mea - (dy_est + d2y_est_plus*dt));

 dx_est = dx_est + d2x_est_plus*dt;
 dy_est = dy_est + d2y_est_plus*dt;
 
 x_mea = meas(4);
 y_mea = meas(5);
 x_est = prev_est(1);
 y_est = prev_est(2);
 x_est = x_est + Kx*(x_mea - (x_est + deltaPosW_est(1)));
 x_est = x_est + dx_est*dt;
 y_est = y_est + Ky*(y_mea - (y_est + deltaPosW_est(2)));
 y_est = y_est + dy_est*dt;
 
 state_estimate = [x_est;  y_est;  theta_est;
                   dx_est; dy_est; omega_est_plus;
                   Ul_hat_plus;  Ur_hat_plus;
                   Slip_l; Slip_r;
                   d2x_est_plus;d2y_est_plus];
 
% similar for y




end