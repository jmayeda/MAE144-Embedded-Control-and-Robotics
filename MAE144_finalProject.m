%% ==================== MAE 144 Final Project ========================== %%
% Solve the equations of motion for the MIP, linearize and simplify. 
% Convert the equations of motion into transfer functions in the s domain
% using the laplace transform, then convert them to the discrete time
% domain into difference equations.

clear; close all; clc; 

%% ----------------------- Define Variables ---------------------------- %%
% Given constants of the MIP
sbar = 0.003;     % Stall Torque (without gearbox) [Nm]
Vb = 7.4;         % Battery Voltage [V] 
wf = 1760;        % Free Spin Frequency (without gearbox) [rad/s] 
G = 35.55555;     % Gearbox Ratio [-] 
Im = 3.6e-8;      % Motor armature of inertia [Kg*m^2]
r = 34e-3;        % radius [m] 
mw = 0.027;       % mass of wheels [kg] 
mb = 0.180;       % mass of the body [kg] 
l = 47.7e-3;      % length to center of mass [kg] 
Ib = 0.000263;    % MIP body moment of inertia [Kg*m^2]
g = 9.81;         % acceleration due to gravity [m/s^2]

e = 2*sbar*G;     % Stall torque of two motors
k = e/(wf/G);     % motor constant = stall torque/free spin freq.

Iw = 2*((mw*r^2)/2+G^2*Im);

% Define some intermediate variables 
a1 = Iw+(mw+mb)*r^2;
a2 = mb*r*l;
b1 = Ib+mb*l^2;
b2 = mb*g*l;

%% ------------- Transfer function G1(s) = theta(s)/U(s) --------------- %%
% Model the inner loop dynamics of the MIP angle theta versus the motor
% duty cycle input u

s = tf('s');

% Through some algebra, we get: 
G1_num = (-e*(a1+a2)*s);
G1_den = (a1*b1+a2^2)*s^3+(k*(a2+b1))*s^2-(a1*b2)*s-k*b2;

% Make G1 monic
G1_num = (1/(a1*b1+a2^2))*G1_num;
G1_den = (1/(a1*b1+a2^2))*G1_den;
G1 = G1_num/G1_den

G1_poles = pole(G1);
G1_zeros = zero(G1);

% Plot impulse response as sanity check
figure(1); 
impulse(G1);
title('Impulse Response: G_1(s) = \Theta(s)/U(s)');

% Root locus 
figure(2);
rlocus(G1);
title('Root Locus: G_1(s) = \Theta(s)/U(s)');

%% -------------------- Controller D1 for G1 --------------------------- %%
% Poles of G1(s) = {9.4677623, -8.68666402, -13.4075987}
% Zeros of G1(s) = {0}
% The system has a single unstable pole.

% Lead controller with p/z=100 for extra phase bump at crossover
% Lag controller with a pole on the LHP side of zero to handle the 
% plant zero at zero.
D1_lead = tf([1 10],[1 100]);       
D1_lag = tf([1 12],[1 0]);
K1 = -10;
D1 = K1*D1_lag*D1_lead;

% Open loop and closed loop system
L1 = minreal(D1*G1);         % Open loop system
T1 = feedback(L1, 1);        % Closed loop system
[z1,p1,k1] = zpkdata(T1,1);  % Closed loop poles, zeros
 
% Discretize the transfer function using MATLAB 'c2d' with zero order hold
dT1 = 0.01;  
D1_zoh = c2d(D1, dT1, 'zoh');
D1_tustin = c2d(D1, dT1, 'tustin');

% Check the system to ensure that we meet our design requirements 
figure(3)
subplot(2,2,1)
rlocus(L1);  % Open-loop RL with Lead-Lag Control
title('Open-loop Root Locus w/ Lead-Lag Control');
subplot(2,2,2)
bode(L1); hold on;
bode(K1*D1_lag*G1); hold off;
[Gm1,Pm1,Wgm1,Wpm1] = margin(L1)
legend('w/ Lead Cont. (p/z = 100)', 'w/ only Lag Cont.')
title('Open-loop Bode Plot G_1(s)*D_1(s)')
subplot(2,2,3)
step(T1)
title('Closed-loop Step Response')
subplot(2,2,4)
bode(T1)
title('Closed-loop Bode Plot')

%% ------------ Transfer function G2(s) = phi(s)/theta(s) -------------- %%
% From algebra...
G2_num = -a2*b1*s^2 - b2;
G2_den = (a1+a2)*s^2;
G2_num = (1/(a1+a2)) * G2_num;
G2_den = (1/(a1+a2)) * G2_den;
G2 = G2_num/G2_den;
G2

% Poles and Zeros of G2
G2_poles = pole(G2);
G2_zeros = zero(G2);

%% --------------------- Controller D2(s) for G2(s) -------------------- %%
tr2 = 1;          % Rise time for outer loop
wc2 = 1.8/tr2;    % Crossover frequency by the second order desing guides 
K2 = -1;           % Outer loop gain

% 2nd order Lead controller with p/z=100. Following the procedure outlined
% in NR Example 19.4. Use Lead controllers to "cancel out" unstable plant
% poles at s = 0. Place them slightly into the LHP.
D2_lead = ((s+0.4)/(s+10))^2; 
D2 = K2*D2_lead;

% Open loop and closed loop system
L2 = minreal(D2*G2);          % Open loop system
T2 = feedback(L2, 1);         % Closed loop system
[z2,p2,k2] = zpkdata(T2,1);   % Closed loop poles, zeros

% Discretize the transfer function using MATLAB 'c2d' with zero order hold
dT2 = 0.05; % inner loop runs at 20 Hz --> dT = 0.05s 
D2_zoh = c2d(D2, dT2, 'zoh');
D2_tustin = c2d(D2, dT2, 'tustin');

figure(4)
subplot(2,2,1)
rlocus(L2);  % Open-loop RL with Lead-Lag Control
title('Open-loop Root Locus w/ Lead-Lag Control');
subplot(2,2,2)
bode(L2); 
[Gm2,Pm2,Wgm2,Wpm2] = margin(L2)
title('Open-loop Bode Plot L_2(s)')
subplot(2,2,3)
step(T2)
title('Closed-loop T_2(s) Step Response')
subplot(2,2,4)
bode(T2)
title('Closed-loop T_2(s) Bode Plot')

%% --------------- Final Closed Loop Control Performance --------------- %%
CL3 = feedback(D2*D1*G1*G2,1)
step(CL3)
