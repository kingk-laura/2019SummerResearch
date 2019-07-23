%volvo_parameters.m

m = 2205; %kg  %Total vehicle mass
ms = 1800; %kg % Sprung mass
m_u = 75; %kg %Unsprung mass
m = ms + 4*m_u; %kg %Total vehicle mass
mass = ms + 4*m_u; %kg %Total vehicle mass
Lf = 1.318; %Distance - Front tires to c.g.
Lr = 1.542; %Distance - Rear tires to c.g.
Lw = 1.75; %Track width
Cs = 828*180/pi;  %N/rad %Tire cornering stiffness %NOT NEEDED
hs = 0.68; %Height of c.g.
hr = 0.8; %Roll center distance from c.g.
reff = 0.3543; %Effective tire radius
rwheel = reff; %Tire radius
Iwheel = 1.6/2; %Tire moment of inertia
Iz = 3000;      %Vehicle yaw moment of inertia
Iy = ms*Lf*Lr;  %Vehicle pitch moment of inertia
Ix = ms*Lw*Lw;  %Vehicle roll mement of inertia
ks = 20000;     %Ssuension stiffness
kt = 200000;    %Tire stiffness
bs = 1300*3;    %Suspension damping
Ca = 0.3693;    %Aerodynamic drag coefficient
g = 9.81;       %Acceleration due to gravity

Cf = 60000;     %Front tire cornering stiffness
Cr = 60000;     %Rear tire cornering stiffness
radius = 300;   % NOT NEEDED
active_steering_factor = 1.0; %0.8; %NOT NEEDED
psi_dot_bound = 0.3; %Should be made a function of rollover index %NOT NEEDED

type_of_steering = 1; %step % NOT NEEDED
%type_of_steering = 2; %fishhook
%type_of_steering = 3; %counter-steering

