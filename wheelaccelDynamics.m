%
% [x,y, theta, dx, dy, omega, d2x, d2y, Ul, Ur, dUl, dUr]

%domega = dUr - dUl

%dUl = input L
%dUr = input R

%d3x = 
%d3y = 

syms theta omega Ul Ur dx dy d2x d2y d3x d3y dUl dUr d2Ul d2Ur
dt = .02; AxelLen = .62;

dx = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (cos(theta)*sin(omega*dt)...
                                 + sin(theta)*cos(omega*dt) - sin(theta));
dy = (AxelLen/2)*(Ur + Ul)/(Ur - Ul) * (sin(theta)*sin(omega*dt)...
                                 - cos(theta)*cos(omega*dt) + cos(theta));
                             
%thankfully:
%alpha;% = (dUr - dUl)/AxelLen;
%beta;% = (d2Ur - d2Ul)/AxelLen;

d2x = diff(dx, Ul)*dUl + diff(dx, Ur)*dUr...
    + diff(dx, theta)*omega + diff(dx, omega)*(dUr - dUl)/AxelLen;
d2y = diff(dy, Ul)*dUl + diff(dy, Ur)*dUr...
    + diff(dy, theta)*omega + diff(dy, omega)*(dUr - dUl)/AxelLen;

d3x = diff(d2x, Ul)*dUl      + diff(d2x, Ur)*dUr...
    + diff(d2x, theta)*omega + diff(d2x, omega)*(dUr - dUl)/AxelLen...
    + diff(d2x, dUl)*d2Ul    + diff(d2x, dUr)*d2Ur;
    %+ diff(d2x, omega)*(dUr - dUl)/AxelLen;...
    %+ diff(d2x, alpha)*(d2Ul - d2Ur)/AxelLen;

d3y = diff(d2y, Ul)*dUl      + diff(d2y, Ur)*dUr...
    + diff(d2y, theta)*omega + diff(d2y, omega)*(dUr - dUl)/AxelLen...
    + diff(d2y, dUl)*d2Ul    + diff(d2y, dUl)*d2Ul;
    %+ diff(d2y, omega)*alpha...
    %+ diff(d2y, alpha)*(d2Ul - d2Ur)/AxelLen;



%now take partial wrt each state (given the current input)
%state = [x,y,theta, dx, dy, omega, d2x, d2y, Ul, Ur, dUl, dUr]
phi = [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0; %dx = dx
       0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0; %dy = dy
       0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0; %dth = om
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0; %ddx = d2x
       0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0; %ddy = d2y
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0,-1, 1; %1/axel len really
       0, 0, diff(d3x, theta), 0, 0, diff(d3x,omega), ...
       0, 0, diff(d3x, Ul), diff(d3x, Ur), diff(d3x, dUl),...
       diff(d3x, dUr); %d2x
       0, 0, diff(d3y, theta), 0, 0, diff(d3y,omega), ...
       0, 0, diff(d3y, Ul), diff(d3y, Ur), diff(d3y, dUl),...
       diff(d3y, dUr); %d2y
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
       0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

 %now can turn into matlab function and use
 %need care for Ul = Ur
 phi_k = matlabFunction(phi)
 %reads Ul, Ur, d2Ul, d2Ur, dUl, dUr, omega, theta as state
 %make sure Ul != Ur
 %phi_k is a function of state
 %Extended Kalman filter EQ's
 state_cov = diag([1, 1, 1,...
                   .1, .1, .1,... 
                   .1, .1, .1, .1,...
                   .1, .1].^2);
       
       
       
       
       
       