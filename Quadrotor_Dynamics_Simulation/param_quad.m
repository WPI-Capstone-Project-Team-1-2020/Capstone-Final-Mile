P.gravity = 9.8331;
P.d = 0.223;
   
%physical parameters of airframe
P.mass = 1.0230;
P.ct   = 1.4865e-07;
P.cq   = 2.9250e-09;
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
P.motor_m = 0.0730;
P.motor_dm = 0.2223;
P.motor_h = 0.0318;
P.motor_r = 0.0140;
P.ESC_m = 0.0300;
P.ESC_a = 0.0254;
P.ESC_b = 0.0572;
P.ESC_ds = 0.0826;
P.HUB_m = 0.4310;
P.HUB_r = 0.0564;
P.HUB_H = 0.0429;
P.arms_m = 0.0450;
P.arms_r = 0.0325;
P.arms_L = 0.1857;
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



