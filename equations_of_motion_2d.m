%% Ignore any coupling between two axis, too complicated :(

%% Setup symbolic vars
clear all, close all
syms g l I_flywheel m_top positive
syms theta dtheta ddtheta phi dphi ddphi tau real

%% d/dt handle
x = [theta; phi];
dx = [dtheta; dphi];
ddx = [ddtheta; ddphi];

dt = @(r) jacobian(r,[x;dx])*[dx;ddx];

%% Use Lagrangian to find equation of motion

I_f = I_flywheel;

T = 1/2*m_top*l^2*dtheta^2 + 1/2*I_f*dphi^2;
U = m_top*g*l*cos(theta);

L = T - U;

% EOM Phi
dL_ddphi = diff(L, dphi);
dL_ddphi_dot = dt(dL_ddphi);
dL_dphi = simplify(diff(L, phi));

e_phi = dL_ddphi_dot - dL_dphi;

% EOM Theta
dL_ddtheta = diff(L, dtheta);
dL_ddtheta_dot = dt(dL_ddtheta);
dL_dtheta = simplify(diff(L, theta));

e_theta = dL_ddtheta_dot - dL_dtheta;

eqns = [e_phi == -tau, e_theta == tau];
vars = [ddphi ddtheta];
sol = solve(eqns, vars);

eqn_phi = simplify(sol.ddphi);
eqn_theta = simplify(sol.ddtheta);

%% Create f(x,u), and find del_x*f(x,u), del_u*f(x,u)

nonlinearSystem = [dtheta;
                   eqn_theta;
                   eqn_phi];
                
x = [theta; dtheta; dphi];
u = [tau];
del = @(r) jacobian(r,[x;u]);

fsym = del(nonlinearSystem);
Bsym = fsym(:, 4:4);
Asym = fsym(:, 1:3);
fA = matlabFunction(Asym);
fB = matlabFunction(Bsym);

%% Construct linearized system around origin
g = 9.81;
l = .2; m_top = 1.350; I_flywheel = 0.002;

feqn_phi = matlabFunction(subs(eqn_phi));
feqn_theta = matlabFunction(subs(eqn_theta));

theta = 0;
dphi = 0; dtheta = 0;
tau = 0;


A = double(fA(g,l,theta));
A = [A,[0 0 0]'];
A = [A;[0 0 -1 0]];

B = double(fB(I_flywheel,l,m_top));
B = [B;[0]];

C = eye(4);

%% Check observability, controllability
sys_ol = ss(A,B,C,0);
Co = ctrb(sys_ol);
rank(Co)
cond(Co)

Ob = obsv(sys_ol);
rank(Ob)
cond(Ob)

%% LQR controller design

A_2d = [A zeros(size(A));
        zeros(size(A)) A];
B_2d = [B zeros(size(B));
        zeros(size(B)) B];
C_2d = eye(8);


max_rads = 40;
max_torque = 1.5;
max_angle = 1;

Q_1d = [1/(max_angle^2) 0 0 0;
        0 .001 0 0;
        0 0 1/(max_rads^2) 0;
        0 0 0 1/(20^2)];
    
Q = [Q_1d zeros(size(Q_1d));
     zeros(size(Q_1d)) Q_1d];

R = 1/(max_torque^2)*eye(2);

sys = ss(A_2d, B_2d, C_2d, 0);

K = lqr(sys,Q,R)

%% Check CL response 

% Initial State Vector [theta1, dtheta1, dphi1, q1, theta2, dtheta2, dphi2, q2]'
x0 = [.1 0 0 0 .1 0 0 0]';

% Time vector
simulationTime = 5;
sampleFreq = 500;
timeVector = (0:1:sampleFreq*simulationTime)/sampleFreq;

% K(1,4) = 0; %% remove integrator
% K(2,8) = 0
[t, x] = ode45(@(t,x) flight_simulator(t, x, feqn_phi, feqn_theta, K), timeVector, x0);


% Get states
theta1 = x(:,1);
dtheta1 = x(:,2);
dphi1 = x(:,3);
theta2 = x(:,4);
dtheta2 = x(:,5);
dphi2 = x(:,6);

for i=1:length(x)
    torque(i,:) = -K*x(i,:)';
    
    if (dphi1(i) > 40) 
        torque(i,1) = 0;
    end

    if (dphi2(i) > 40) 
        torque(i,2) = 0;
    end
    
end

torque = min(max(torque, -1.5), 1.5);
torque1 = torque(:,1);

%Plot
figure;
subplot(4,1,1);
plot(t, theta1);
title('theta1 vs time');
xlabel('time');
ylabel('theta1');
subplot(4,1,2);
plot(t, dtheta1);
title('dtheta1 vs time');
xlabel('time');
ylabel('dtheta1');
subplot(4,1,3);
plot(t, dphi1);
title('dphi1 vs time');
xlabel('time');
ylabel('dphi1');
subplot(4,1,4);
plot(t, torque1);
title('torque vs time');
xlabel('time');
ylabel('torque');




function dxdt = flight_simulator(t, x, feqn_phi, feqn_theta, K)
theta1 = x(1);
dtheta1 = x(2);
dphi1 = x(3);
q1 = x(4);
theta2 = x(5);
dtheta2 = x(6);
dphi2 = x(7);
q2 = x(8);
u = -K*x;
u = min(max(u, -1.5), 1.5);

if (dphi1 > 40) 
    u(1) = 0;
end

if (dphi2 > 40) 
    u(2) = 0;
end

ddtheta1 = feqn_theta(u(1), theta1);
ddtheta2 = feqn_theta(u(2), theta2);
ddphi1 = feqn_phi(u(1));
ddphi2 = feqn_phi(u(2));
dq1 = -dphi1;
dq2 = -dphi2;

dxdt = [dtheta1; ddtheta1; ddphi1; dq1; dtheta2; ddtheta2; ddphi2; dq2];
end
