function [state_cov,x] = propagateCovState(x, U, state_cov_in, meas, delta_time)
%PROPATECOVCALCGAINS Summary of this function goes here
%   x is 11 state vector U = [Ul, Ur]

%% Set up estimator
syms theta omega Ul Ur dx dy d2x d2y d3x d3y dUl dUr d2Ul d2Ur
dt = .02; AxelLen = .62;
dx = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (cos(theta)*sin((Ur - Ul) / AxelLen)... %take dt off and then do expm?
                                 + sin(theta)*cos((Ur - Ul) / AxelLen) - sin(theta));
dy = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (sin(theta)*sin((Ur - Ul) / AxelLen)...
                                 - cos(theta)*cos((Ur - Ul) / AxelLen) + cos(theta));
                             
%thankfully:
%omega = (Ur - Ul) / AxelLen
%alpha;% = (dUr - dUl)/AxelLen;
%beta;% = (d2Ur - d2Ul)/AxelLen;

%d2x = diff(dx, Ul)*dUl + diff(dx, Ur)*dUr...
%    + diff(dx, theta)*(Ur - Ul) / AxelLen;
%d2y = diff(dy, Ul)*dUl + diff(dy, Ur)*dUr...
%    + diff(dy, theta)*(Ur - Ul) / AxelLen;
firstordermodel_dx = matlabFunction(dx);
firstordermodel_dy = matlabFunction(dy);

d2x = (firstordermodel_dx(Ul +dUl*dt, Ur +dUr*dt, theta + dt*(Ur-Ul)/AxelLen)...
    - firstordermodel_dx(Ul - dUl*dt, Ur -dUr*dt, theta -dt*(Ur-Ul)/AxelLen))/(2*dt);
d2y = (firstordermodel_dy(Ul +dUl*dt, Ur +dUr*dt, theta + dt*(Ur-Ul)/AxelLen)...
    - firstordermodel_dy(Ul - dUl*dt, Ur -dUr*dt, theta -dt*(Ur-Ul)/AxelLen))/(2*dt);

firstordermodel_d2x = matlabFunction(d2x);
firstordermodel_d2y = matlabFunction(d2y);

d3x = (firstordermodel_d2x(Ul + dUl*dt, Ur + dUr*dt,...
                           dUl + d2Ul*dt, dUr + d2Ur*dt,...
                           theta + dt*(Ur-Ul)/AxelLen)...
    - firstordermodel_d2x(Ul - dUl*dt, Ur - dUr*dt,...
                           dUl - d2Ul*dt, dUr - d2Ur*dt,...
                           theta - dt*(Ur-Ul)/AxelLen))/(2*dt);
d3y = (firstordermodel_d2y(Ul + dUl*dt, Ur + dUr*dt,...
                           dUl + d2Ul*dt, dUr + d2Ur*dt,...
                           theta + dt*(Ur-Ul)/AxelLen)...
    - firstordermodel_d2y(Ul - dUl*dt, Ur - dUr*dt,...
                           dUl - d2Ul*dt, dUr - d2Ur*dt,...
                           theta - dt*(Ur-Ul)/AxelLen))/(2*dt);
% 
% d3x = diff(d2x, Ul)*dUl      + diff(d2x, Ur)*dUr...
%     + diff(d2x, dUl)*d2Ul    + diff(d2x, dUr)*d2Ur...
%     + diff(d2x, theta)*(Ur - Ul) / AxelLen;
%     %+ diff(d2x, omega)*(dUr - dUl)/AxelLen;...
%     %+ diff(d2x, alpha)*(d2Ul - d2Ur)/AxelLen;
% 
% d3y = diff(d2y, Ul)*dUl      + diff(d2y, Ur)*dUr...
%     + diff(d2y, dUl)*d2Ul    + diff(d2y, dUr)*d2Ur...
%     + diff(d2y, theta)*(Ur - Ul) / AxelLen;
%     %+ diff(d2y, omega)*alpha...
%     %+ diff(d2y, alpha)*(d2Ul - d2Ur)/AxelLen;



%now take partial wrt each state (given the current input)
%state = [x,y,theta, dx, dy, d2x, d2y, Ul, Ur, dUl, dUr]
phi = [0, 0, diff(dx,theta), 0, 0, 0, 0, diff(dx, Ul),diff(dx,Ur), 0, 0; %dx = dx
       0, 0, diff(dy,theta), 0, 0, 0, 0, diff(dy, Ul),diff(dy,Ur), 0, 0; %dy = dy
       0, 0, 0, 0, 0, 0, 0, -1/AxelLen, 1/AxelLen, 0, 0; %dth = om
       0, 0, diff(d2x,theta), 0, 0, 0, 0, ...
       diff(d2x,Ul), diff(d2x,Ur), diff(d2x,dUl), diff(d2x,dUr); %ddx = d2x
       0, 0, diff(d2y,theta), 0, 0, 0, 0, ...
       diff(d2y,Ul), diff(d2y,Ur), diff(d2y,dUl), diff(d2y,dUr); %ddy = d2y
       0, 0, 0, 0, 0, ...
       0, 0, diff(d3x, Ul), diff(d3x, Ur),...
             diff(d3x, dUl),diff(d3x, dUr); %d2x
       0, 0, 0, 0, 0, ...
       0, 0, diff(d3y, Ul), diff(d3y, Ur),...
             diff(d3y, dUl),diff(d3y, dUr); %d2y
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0; %Ul %make it slow down a bit
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1; %Ur %on its own
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0; %dUl
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];%dUr
 gamma = [0, 0; %x
          0, 0; %y
          0, 0; %theta
          0, 0; %dx
          0, 0; %dy
          diff(d3x, d2Ul), diff(d3x, d2Ur); %d2x
          diff(d3y, d2Ul), diff(d3y, d2Ur); %d2y
          0, 0; %Ul
          0, 0; %Ur
          1, 0; %dUl
          0, 1]; %dUr

 %now can turn into matlab function and use
 %need care for Ul = Ur
 phi_state = matlabFunction(phi);
  %reads Ul, Ur, d2Ul, d2Ur, dUl, dUr, theta as state
 gamma_state = matlabFunction(gamma);
 
 state_cov = state_cov_in;
 
  %input noise
 Q = diag([.002, .002, .0002, ... %x, y, theta
           .005,  .005, .005, .005,... %dx, dy, d2x, d2y
           .002, .002, .002, .002].^2); %Ul, Ur, dUl, dUr
 %measurement noise
 R = diag([.05, .05, .05,... %x, y, theta
           .01, .01, .02, .02, .01].^2); %d2x, d2y, Ul, Ur, omega
 %measurement mat
 C = [eye(3), zeros(3, 8); %x, y, theta
      zeros(4, 5), eye(4, 6); % d2x, d2y, Ul, Ur
      zeros(1, 7), -1/AxelLen, 1/AxelLen, 0, 0]; %omega

     [A_k, B_k] = robust_phi_state(x, U, phi_state, gamma_state);
     p = ss(A_k, B_k, eye(11), zeros(11,2));
     pd = c2d(p, delta_time); %Get DT linearization
     x = pd.A*x + pd.B*U'; %Propagate estimate
     x(3) = angleDiff(x(3),0);
     state_cov = pd.A*state_cov*pd.A' + Q; %Propagate covariance
     K = state_cov*C' / (C*state_cov*C' + R); %Calculate gains
     
     x = x + K*(meas - C*x); %Update estimate with measurement
     state_cov = (eye(11) - K*C)*state_cov; %Update covariance wrt gains
     if abs(x(3) - meas(3)) > 1.8*pi
         x(3) = meas(3)
     end
end

 function [phi, gamma] = robust_phi_state(x, u, phi_func, gamma_func)
 vel_adj = x(8);  
 if (abs(x(8) - x(9)) < .00001) 
       vel_adj = x(8) + .00002;
 end
       phi = phi_func(vel_adj, x(9), u(1), u(2), x(10), x(11),x(3));
       gamma = gamma_func(vel_adj, x(9), u(1), u(2), x(10), x(11),x(3));
 end

