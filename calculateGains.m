%calculate gains for the system with specified variances

%first theta and omega (they are linear with input wheel accelerations)

Phi = [0, 1; 0, 0];
C = eye(2,2); 
R = diag([POStheta_variance, IMUomega_variance]); %variance of [pos sensor theta, imu omega]
Q = diag([  .01^2, .06^2]);   %variance of disturbance on [theta; omega (input)] 
                           %note that while theta has no real dist,
                           % we can model imu bias as disturbing the theta
                           % estimate. This prevents the gains from falling
                           % to zero.
                           %omega disturbance is error in omega from wheels
                           % not doing exactly what is assumed

[P_angle] = dare(Phi, C, Q, R)
gains_angle = diag(P_angle*C' / (C*P_angle*C' + R))

%then wheel vels
Phi_wheel = [0, 1;0, 0]; %wheel vel integrates accel
C_wheel = [1, 0; 0, 0]; %measure vel
Q = diag([ .01^2, .02^2]); %input noise on vel and accel
R = diag([Ul_read_variance, 100]); % measurement noise on vel mea
P_wheel = dare(Phi_wheel, C_wheel,Q , R)
gains_wheel = diag((P_wheel * C_wheel') / (C_wheel*P_wheel*C_wheel' + R))