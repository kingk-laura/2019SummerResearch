%volvo_parameters.m

m = 2205; %kg
ms = 1800;
m_u = 75;
m = ms + 4*m_u;
mass = ms + 4*m_u;
Lf = 1.318;
Lr = 1.542;
Lw = 1.75;
Cs = 828*180/pi;  %N/rad
hs = 0.68;
hr = 0.8;
reff = 0.3543;
rwheel = reff;
Iwheel = 1.6/2;
Iz = 3000;
Iy = ms*Lf*Lr;
Ix = ms*Lw*Lw;
ks = 20000;
kt = 200000;
bs = 1300*3;
Ca = 0.3693;
g = 9.81;

Cf = 60000;
Cr = 60000;
radius = 300;
active_steering_factor = 1.0; %0.8;
psi_dot_bound = 0.3; %Should be made a function of rollover index

type_of_steering = 1; %step
%type_of_steering = 2; %fishhook
%type_of_steering = 3; %counter-steering
type_of_steering = 4; %Closed-loop lateral control, state feedback

