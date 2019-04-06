function state_estimate = propagateEstimate(inputs, prev_est, meas, dt)
%propagateEstimate
% given the current inputs (wheel velocitiy commands): [Ul; Ur],
% the previous estimate of the state:
%  [x;  y;  theta;
%   dx; dy; omega;
%   Ul_act; Ur_act;
%   Slip_l; Slip_r]
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
Kx = .5;
Ky = .5;
Kth = .5;
Kdx = .5;
Kdy = .5;
Kom = .5;
KUl = .5;
KUr = .5;
%KSl = .5;
%KSr = .5;

%for now, slip_est = 0
Ul = inputs(1); %known (what we asked the wheels to do)
Ur = inputs(2); %known
Ul_hat_prev = prev_est(7); %estimate actual wheel speed here
Ur_hat_prev = prev_est(8);
Ul_accel = (Ul - prev_est(7))/dt;%should use center(or forward) difference of inputs
Ur_accel = (Ur - prev_est(8))/dt;% using back difference for now

Ul_mea = meas(7);
Ur_mea = meas(8);
Ul_hat_plus = Ul_hat_prev + KUl*(Ul_mea - (Ul_hat_prev)); 
Ur_hat_plus = Ur_hat_prev + KUr*(Ur_mea - (Ur_hat_prev + Ur_accel*dt));
Ul_hat_plus = Ul_hat_plus + Ul_accel*dt;
Ur_hat_plus = Ur_hat_plus + Ur_accel*dt;
Slip_l = 0; %should try to estimate these
Slip_r = 0;
Ul_eff_hat = (Ul_hat_plus - Slip_l);
Ur_eff_hat = (Ur_hat_plus - Slip_r);

%for omega, omega = (Ur_eff - Ul_eff)/AxelLen
%so need pomega / pUr and pomega / pUl
%and 
 omega_mea = meas(3);
 omega_est = (Ur_eff_hat - Ul_eff_hat)/AxelLen; %is this omega_est_plus?
 omega_est_prev = prev_est(6); %is this used?
 omega_est = omega_est + Kom*(omega_mea - omega_est);
% omega_est = omega_est + (Ur_eff_hat + (dUr)*dt - Ul_eff_hat - (dUl)*dt)/AxelLen 

 theta_est = prev_est(6);
 theta_est = theta_est + Kth*(theta_mea - theta_est);
 theta_est = theta_est + omega_est*dt;
% ^^ need care for angle reacharound

%dx,dy are local dx,dy (from wheel vels) transformed by theta
%need instant turn radius, world rot mat, then dPos:
%  R = AxelLen/2 * (Ur + Ul)/(Ur - Ul);
%  rot = [cos(w*dt), -sin(w*dt);sin(w*dt),cos(w*dt)];
%  dPos = rot*[0;-R] + [0;R];
%so using our estimates:
 R_est = AxelLen/2 * (Ur_eff_hat + Ul_eff_hat) / (Ur_eff_hat - Ul_eff_hat);
%  ^^ Need care for zero point turns (Ur = Ul)
%  rot_est: use omega_est * dt for w*dt

rot_est = [cos(omega_est*dt), -sin(omega_est*dt);
           sin(omega_est*dt),  cos(omega_est*dt)];
       
dPos_est = rot_est*[0;-R_est] + [0;R_est];

%TODO
%x and y then:
dx_mea = dPos_est(1); %is this ok? treating as measurement?
dy_mea = dPos_est(2);
 dx_est = prev_est(4);
 dy_est = prev_est(5);
 %we dont have a true dx_mea or dy_mea?
 dx_est = dx_est + Kdx*(dx_mea - dx_est);
 dy_est = dy_est + Kdy*(dy_mea - dy_est);
 d2x_est = meas(1); %dont have model?
 d2y_est = meas(2); %TODO do from model, you cann...
 dx_est = dx_est + d2x_est*dt;
 dy_est = dy_est + d2y_est*dt;
 
 x_mea = meas(4);
 y_mea = meas(5);
x_est = prev_est(1);
y_est = prev_est(2);
 x_est = x_est + Kx*(x_mea - x_est);
 x_est = x_est + dx_est*dt;
 y_est = y_est + Ky*(y_mea - y_est);
 y_est = y_est + dy_est*dt;
 
 state_estimate = [x_est;  y_est;  theta_est;
                   dx_est; dy_est; omega_est;
                   Ul_hat_plus;  Ur_hat_plus;
                   Slip_l; Slip_r];
 
% similar for y




end