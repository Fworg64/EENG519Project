function state_estimate = propagateEstimate(inputs, prev_est, meas, dt)
%propagateEstimate
% given the current inputs (wheel velocitiy commands): [Ul; Ur],
% the previous estimate of the state: [x;y;theta;dx;dy;dtheta;Ul_eff;Ur_eff]
% and the measurement vector: [d2x; d2y; omega,x; y; theta; Ul_mea; Ur_mea]
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

%for now, slip_est = 0

%for omega, omega = (Ur_eff - Ul_eff)/AxelLen
%so need pomega / pUr and pomega / pUl
%and 
% omega_est = (Ur_eff_hat - Ul_eff_hat)/AxelLen 
% omega_est = omega_est + K(omega_mea - omega_est)
% omega_est = omega_est + (Ur_eff_hat + (dUr)*dt - Ul_eff_hat - (dUl)*dt)/AxelLen 

% theta_est = theta_est + K(theta_mea - theta_est)
% theta_est = theta_est + omega_est*dt
% ^^ need care for angle reacharound

%dx,dy are local dx,dy (from wheel vels) transformed by theta
%need instant turn radius, world rot mat, then dPos:
%  R = AxelLen/2 * (Ur + Ul)/(Ur - Ul);
%  rot = [cos(w*dt), -sin(w*dt);sin(w*dt),cos(w*dt)];
%  dPos = rot*[0;-R] + [0;R];
%so using our estimates:
%  R_est = AxelLen/2 * (Ur_eff_est + Ul_eff_est) / (Ur_eff_est - Ul_eff_est)
%  ^^ Need care for zero point turns (Ur = Ul)
%  rot_est: use omega_est * dt for w*dt

%x and y then:
% x_est = x_est + K(x_mea - x_est)
% x_est = x_est + dx*dt
% similar for y




end