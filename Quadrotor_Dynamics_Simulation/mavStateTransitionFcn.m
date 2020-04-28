function x_next = mavStateTransitionFcn(x,uu)

dt = 0.005; % Time step size

pn    = x(1);
pe    = x(2);
pd    = x(3);
u     = x(4);
v     = x(5);
w     = x(6);
phi   = x(7);
theta = x(8);
psi   = x(9);
p     = x(10);
q     = x(11);
r     = x(12);
fx    = 0;
fy    = 0;
fz    = 0;
ell   = 0;
m     = 0;
n     = 0;
w1    = uu(1);
w2    = uu(2);
w3    = uu(3);
w4    = uu(4);

P.gravity = 9.8331;
P.d = 0.223;
   
%physical parameters of airframe
P.mass = 1.0230;
P.ct   = 1.4865e-07;    % Motor Thrust Coefficient
P.cq   = 2.9250e-09;    % Motor Torque Coefficient

% Moments about the x y and z axis
P.Jx   = 0.0095;
P.Jy   = 0.0095;
P.Jz   = 0.0186;


P.Jm   = 3.7882e-06;
P.Jb   = [0.0095,0,0;0,0.0095,0;0,0,0.0186];
P.Jbinv = [105.2643,0,0;0,105.2643,0;0,0,53.8329];
%P.Jxz  = 0.1204;
P.Jxz = 0.0;
P.dctcq = [...
    0,3.3037e-08,0,-3.3037e-08;...
    -3.3037e-08,0,3.3037e-08,0;...
    -2.9250e-09,2.9250e-09,-2.9250e-09,2.9250e-09...
    ];

% Motor Parameters
P.motor_m = 0.23; % kg      0.0730;
P.motor_dm = 0.2223;
P.motor_h = 0.0318;
P.motor_r = 0.0140;

% ESC Parameters
P.ESC_m = 0.009; %kg        0.0300;
P.ESC_a = 0.0254;
P.ESC_b = 0.0572;
P.ESC_ds = 0.0826;

% HUB (?) Parameters
P.HUB_m = 0.4310;
P.HUB_r = 0.0564;
P.HUB_H = 0.0429;

% Arm Parameters
P.arms_m = 0.0450; % Mass
P.arms_r = 0.0325; % Radius(?)
P.arms_L = 0.1857; % Length
P.arms_da = 0.0508;


P.T       = 0.0760;
P.minThr  = 5;
P.cr      = 80.5840;
P.b       = 976.20;
P.plusConfig = 1;

% initial conditions
P.pn0    =   0; % initial North position
P.pe0    =   0; % initial East position
P.pd0    =   0; % initial Down position (negative altitude)
P.u0     =   0; % initial velocity along body x-axis
P.v0     =   0; % initial velocity along body y-axis
P.w0     =   0; % initial velocity along body z-axis
P.phi0   =   0; % initial roll angle
P.theta0 =   0; % initial pitch angle
P.psi0   =   0; % initial yaw angle
P.p0     =   0; % initial body frame roll rate
P.q0     =   0; % initial body frame pitch rate
P.r0     =   0; % initial body frame yaw rate




W = [w1; w2; w3; w4];
    
Dist_tau = [ell; m; n];
Dist_F = [fx; fy; fz];

tau_motorGyro = [q*P.Jm*2*pi/60*(-w1-w3+w2+w4); p*P.Jm*2*pi/60*(w1+w3-w2-w4); 0]; % Note: 2*pi/60 required to convert from RPM to radians/s
Mb = (P.dctcq*(W.^2))+ tau_motorGyro + (Dist_tau);  % Mb = [tau1 tau2 tau3]'

% Thrust due to motor speed
% Force should be in units of Newtons for simplicity in calculating
% the acceleration in the angular velocity state equation
Fb = [0; 0; sum(P.ct*(W.^2))];   %[0, 0, sum(ct*w.^2)]'

% Obtain dP dQ dR
omb_bi = [p; q; r];
OMb_bi = [ 0,-r, q;
           r, 0,-p;
          -q, p, 0];

b_omdotb_bi = P.Jbinv*(Mb-OMb_bi*P.Jb*omb_bi);

Rib = [...
    cos(theta)*cos(psi)...
    (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))...
    (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi));
    cos(theta)*sin(psi)...
    (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))...
    (cos(phi)*sin(theta)*sin(psi) - sin(phi)*sin(psi));
    -sin(theta) sin(phi)*cos(theta) cos(phi)*cos(theta)...
    ];

Rbi = Rib';
ge = [0; 0; -P.gravity];
gb = Rbi*ge;
Dist_Fb = Rbi*Dist_F;

% Compute Velocity and Position derivatives of body frame
vb = [u;v;w];
b_dv = (1/P.mass)*Fb+gb+Dist_Fb-OMb_bi*vb; % Acceleration in body frame (FOR VELOCITY)
i_dp = Rib*vb;

pndot = i_dp(1);
pedot = i_dp(2);
pddot = i_dp(3);

udot = b_dv(1);
vdot = b_dv(2);
wdot = b_dv(3);

phidot = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
thetadot = cos(phi)*q - sin(phi)*r;
psidot = (sin(phi)/cos(theta))*q + (cos(phi)/cos(theta))*r;

pdot = b_omdotb_bi(1);
qdot = b_omdotb_bi(2);
rdot = b_omdotb_bi(3);

x_next = [0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0; 0];
x_next(1) = pn + pndot*dt;
x_next(2) = pe + pedot*dt;
x_next(3) = pd + pddot*dt;
x_next(4) = u + udot*dt;
x_next(5) = v + vdot*dt;
x_next(6) = w + wdot*dt;
x_next(7) = phi + phidot*dt;
x_next(8) = theta + thetadot*dt;
x_next(9) = psi + psidot*dt;
x_next(10) = p + pdot*dt;
x_next(11) = q + qdot*dt;
x_next(12) = r + rdot*dt;
end