function f = xprime(t,x)

vehicle_parameters;
global ROLLOVER ROLLOVER_WARNING sum_error t_old delta_old new_sum initial_vehicle_speed surint x_vec road_curv_index road_curv_vec road_curv_index_max

old_sum = new_sum;

%Wheel speeds
%x(1) wheel speed front left
%x(2) wheel speed front right
%x(3) wheel speed rear left
%x(4) wheel speed rear right

%Unsprung mass
%x(5) unsprung mass front left position
%x(6) unsprung mass front right position
%x(7) unsprung mass rear left position
%x(8) unsprung mass rear right position
%x(9) unsprung mass front left velocity
%x(10) unsprung mass front right velocity
%x(11) unsprung mass rear left velocity
%x(12) unsprung mass rear right velocity

%Sprung mass
%x(13) sprung mass vertical position 
%x(14) sprung mass roll angle
%x(15) sprung mass pitch angle
%x(16) longitudinal speed
%x(17) lateral speed
%x(18) vertical speed
%x(19) yaw rate
%x(20) roll rate
%x(21) pitch rate
%x(22) yaw angle
%x(23) integral of steering angle

%14 dof = 28 states, 4 states for wheel angular position not used
%2 other states not used: longitudinal position, lateral position
%Hence, 22 states in all, 23rd state - integral of steering angle

%Additional states added to allow steering control with lateral position feedback
%x(24)  lateral position error e_1
%x(25)  lateral error derivative e_1_dot
%x(26)  yaw angle error e_2
%x(27)  yaw angle error derivative e_2_dot
%x(28)  longitudinal position

%Intermediate variables
% Roll angle = phi
phi = x(14);
phi_dot = x(20);
if( (phi > pi/2) | (phi < -pi/2) ) phi = (pi/2)*sign(phi); phi_dot = 0.0; end

% Heave (sprung mass) = zs
zs =  x(13);
zs_dot = x(18);

%Longitudinal speed = Vx
Vx = x(16);

%Rotational wheel speeds
wfl = x(1);
wfr = x(2);
wrl = x(3);
wrr = x(4);

%Yaw rate = psi_dot, lateral velocity = ydot
psi_dot = x(19);
ydot = x(17);

%Pitch angle = theta
theta = x(15);
zu_fl = x(5); zu_dot_fl = x(9);
zu_fr = x(6); zu_dot_fr = x(10);
zu_rl = x(7); zu_dot_rl = x(11);
zu_rr = x(8); zu_dot_rr = x(12);

%Suspension forces
%Should use sin(phi) instead of phi, for large roll angles
Fsusp_fl = -0.5*ms*g*Lr/(Lf+Lr) -ks *(zs-phi*Lw/2 - zu_fl) - bs*(zs_dot -phi_dot*Lw/2 - zu_dot_fl);
Fsusp_fr = -0.5*ms*g*Lr/(Lf+Lr) -ks *(zs+phi*Lw/2 - zu_fr) - bs*(zs_dot +phi_dot*Lw/2 - zu_dot_fr);
Fsusp_rl = -0.5*ms*g*Lf/(Lf+Lr) -ks *(zs-phi*Lw/2 - zu_rl) - bs*(zs_dot -phi_dot*Lw/2 - zu_dot_rl);
Fsusp_rr = -0.5*ms*g*Lf/(Lf+Lr) -ks *(zs+phi*Lw/2 - zu_rr) - bs*(zs_dot +phi_dot*Lw/2 - zu_dot_rr);

Fsusp_fl_dyn = -ks *(zs-phi*Lw/2 - zu_fl) - bs*(zs_dot -phi_dot*Lw/2 - zu_dot_fl);
Fsusp_fr_dyn = -ks *(zs+phi*Lw/2 - zu_fr) - bs*(zs_dot +phi_dot*Lw/2 - zu_dot_fr);
Fsusp_rl_dyn = -ks *(zs-phi*Lw/2 - zu_rl) - bs*(zs_dot -phi_dot*Lw/2 - zu_dot_rl);
Fsusp_rr_dyn = -ks *(zs+phi*Lw/2 - zu_rr) - bs*(zs_dot +phi_dot*Lw/2 - zu_dot_rr);


%Define slip ratios
if( (max(rwheel*wfl,Vx) > 0.01) )
    slip_ratio_fl = (rwheel*wfl-Vx)/(max(rwheel*wfl,Vx));
else
    slip_ratio_fl = 0.0;
end

if( (max(rwheel*wfr,Vx) > 0.01) )
    slip_ratio_fr = (rwheel*wfr-Vx)/(max(rwheel*wfr,Vx));
else
    slip_ratio_fr = 0.0;
end

if( (max(rwheel*wrl,Vx) > 0.01) )
    slip_ratio_rl = (rwheel*wrl-Vx)/(max(rwheel*wrl,Vx));
else
    slip_ratio_rl = 0.0;
end

if( (max(rwheel*wrr,Vx) > 0.01) )
    slip_ratio_rr = (rwheel*wrr-Vx)/(max(rwheel*wrr,Vx));
else
    slip_ratio_rr = 0.0;
end

if(type_of_steering == 0) %Zero input
delta_needed = 0;
end
if(type_of_steering == 1) %Step input
delta_needed = steering(t,t_old,delta_old,Vx);
end
if(type_of_steering == 2) %Fishhook
delta_needed = fishhook_steering(t,t_old,delta_old,Vx);
end
if(type_of_steering == 3) %Counter-steering, used in rollover prevention
   delta_needed = steering(t,t_old,delta_old,Vx);
end
if(type_of_steering == 4 | type_of_steering == 5) %Full state feedback
    e1 = x(24);
    e1_dot = x(25);
    e2 = x(26);
    e2_dot = x(27);
    elat_vec = [x(24);x(25);x(26);x(27);];
delta_needed = steering_sfb(elat_vec); %Feedback of all lateral states
end

if(type_of_steering == 3)
    delta_original = delta_needed;
    [delta_needed,new_sum] = counter_steering(t,delta_original,phi,phi_dot,old_sum);
end

%Variables needed for the next iteration
delta_old = delta_needed;
t_old = t; 

delta = active_steering_factor*delta_needed;
%active_steering_factor is normally set to 1, except tunable for counter-steering


%Define slip angles
slip_angle_fl = delta - (ydot + Lf*psi_dot)/Vx;
slip_angle_fr = delta - (ydot + Lf*psi_dot)/Vx;
slip_angle_rl = - (ydot - Lr*psi_dot)/Vx;
slip_angle_rr = - (ydot - Lr*psi_dot)/Vx;

Fz = 1.0*0.25*(ms + 4*m_u)*9.81;
%Find lateral and longitudinal tire forces
[Ftire_lat_fl,Ftire_long_fl] = tire_model(slip_angle_fl,slip_ratio_fl,Fz);
[Ftire_lat_fr,Ftire_long_fr] = tire_model(slip_angle_fr,slip_ratio_fr,Fz);
[Ftire_lat_rl,Ftire_long_rl] = tire_model(slip_angle_rl,slip_ratio_rl,Fz);
[Ftire_lat_rr,Ftire_long_rr] = tire_model(slip_angle_rr,slip_ratio_rr,Fz);

%Vertical road profile
zr_fl = 0.0;
zr_fr = 0.0;
zr_rl = 0.0;
zr_rr = 0.0;


%State equations
f = zeros(size(x));

%Wheel speeds
%x(1) wheel speed front left
%x(2) wheel speed front right
%x(3) wheel speed rear left
%x(4) wheel speed rear right

%NO cruise control
if(type_of_cruise_control == 1)
Tdrive = 0.0;   %If there is no aerodynamic drag or rolling resistance, there is no need of drive torque
end

%Cruise Control - Call modular cruise_control.m function
if(type_of_cruise_control == 2 | type_of_cruise_control == 3)
Vx_des = initial_vehicle_speed; 
Tdrive = cruise_control(Vx,Vx_des);
end

% Allocation of drive torque to the individual wheels
Td_fl = 0.25*Tdrive;
Td_fr = 0.25*Tdrive;
Td_rl = 0.25*Tdrive;
Td_rr = 0.25*Tdrive;


%Rollover index
ay_meas = f(17)+Vx*psi_dot;
Rindex = rollover_index(ay_meas,phi,hr,hs,Lw);

mass= ms+4*m_u;

%Electronic Stability Control - ESC
%Tb = 0; %NO ESC
if(type_of_ESC == 1)
    Tb =0;
end

%ESC - Call the modular ESC function, default or user-provided
if(type_of_ESC == 2 | type_of_ESC == 3)
Ftire_lat_vec = [Ftire_lat_fl;Ftire_lat_fr;Ftire_lat_rl;Ftire_lat_rr;];
esc_par_vec = zeros(8,1);
esc_par_vec(1) = mass;
esc_par_vec(2) = Lf;
esc_par_vec(3) = Lr;
esc_par_vec(4) = Cf;
esc_par_vec(5) = Cr;
esc_par_vec(6) = Lw;
esc_par_vec(7) = Iz;
esc_par_vec(8) = reff;
Tb = ESC(psi_dot,Vx,delta_needed,Ftire_lat_vec,Lw,Lf,Lr,Iz,reff,esc_par_vec);
end

% Allocation of differential braking to provide yaw torque commanded by ESC
if(Tb <= 0)
    Tb_fl = 0;
    Tb_fr = -Tb;
    Tb_rl = 0;
    Tb_rr = -Tb;
end
if(Tb > 0)
    Tb_fl = Tb;
    Tb_fr = 0;
    Tb_rl = Tb;
    Tb_rr = 0;
end

%[Tb_fl Tb_fr Tb_rl Tb_rr]


%Wheel dynamics
f(1) = (Td_fl -Tb_fl - Ftire_long_fl*reff)/Iwheel;
f(2) = (Td_fr -Tb_fr - Ftire_long_fr*reff)/Iwheel;
f(3) = (Td_rl -Tb_rl - Ftire_long_rl*reff)/Iwheel;
f(4) = (Td_rr -Tb_rr - Ftire_long_rr*reff)/Iwheel;


%Unsprung mass
%x(5) unsprung mass front left position
%x(6) unsprung mass front right position
%x(7) unsprung mass rear left position
%x(8) unsprung mass rear right position
%x(9) unsprung mass front left velocity
%x(10) unsprung mass front right velocity
%x(11) unsprung mass rear left velocity
%x(12) unsprung mass rear right velocity

f(5) = x(9);
f(6) = x(10);
f(7) = x(11);
f(8) = x(12);
F_tire_fl_static = 0.5*ms*g*Lr/(Lf+Lr) + m_u*g;
F_tire_fr_static = 0.5*ms*g*Lr/(Lf+Lr) + m_u*g;
F_tire_rl_static = 0.5*ms*g*Lf/(Lf+Lr) + m_u*g;
F_tire_rr_static = 0.5*ms*g*Lf/(Lf+Lr) + m_u*g;

f(9) = (-Fsusp_fl_dyn - kt*(x(5)-zr_fl) - F_tire_fl_static)/m_u;
f(10) = (-Fsusp_fr_dyn - kt*(x(6)-zr_fr) - F_tire_fr_static)/m_u;
f(11) = (-Fsusp_rl_dyn - kt*(x(7)-zr_rl) - F_tire_rl_static)/m_u;
f(12) = (-Fsusp_rr_dyn - kt*(x(8)-zr_rr) - F_tire_rr_static)/m_u;

%Sprung mass
%x(13) sprung mass vertical position 
%x(14) sprung mass roll angle
%x(15) sprung mass pitch angle
%x(16) longitudinal speed
%x(17) lateral speed
%x(18) vertical speed
%x(19) yaw rate
%x(20) roll rate
%x(21) pitch rate

f(13) = zs_dot;
f(14) = phi_dot;
f(15) = x(21);

%Longitudnal dynamics
Ftraction = Ftire_long_fl + Ftire_long_fr + Ftire_long_rl + Ftire_long_rr;
F_aero = 0*Ca*Vx*Vx; %No drive torque
F_rolling_res = 0;   %No drive torque
f(16) = (Ftraction - F_aero - F_rolling_res)/m;

%Lateral dynamics
Ftire_lat = Ftire_lat_fl + Ftire_lat_fr + Ftire_lat_rl + Ftire_lat_rr;
%f(17) = (Ftire_lat - m*Vx*psi_dot)/m;
%Lateral equation is written later after writing the roll acceleration equation

%Vertical heave dynamics
Fsusp_total = Fsusp_fl_dyn + Fsusp_fr_dyn + Fsusp_rl_dyn + Fsusp_rr_dyn;
f(18) = (Fsusp_total - ms*g)/ms;

%Yaw dynamics
Torque_tire_yaw = Lf*(Ftire_lat_fl + Ftire_lat_fr) - Lr*(Ftire_lat_rl + Ftire_lat_rr); %Lateral forces only
%Lateral forces and differential longitudinal forces
Torque_tire_yaw = Torque_tire_yaw + Lw/2*(Ftire_long_fr-Ftire_long_fl) + Lw/2*(Ftire_long_rr-Ftire_long_rl); 
% if(t>5 & t<5.1) 
%     disp('t Delta Yaw moment')
%     [t delta Torque_tire_yaw]
% end

f(19) = (Torque_tire_yaw)/Iz;

%Roll dynamics
Torque_susp_roll = Lw/2*(Fsusp_fr + Fsusp_rr - Fsusp_fl - Fsusp_rl);
f(20) = (Torque_susp_roll + ms*g*hs*sin(phi) + Ftire_lat*hr*cos(phi))/(Ix+ms*hr*hr);
External_moment = 0.0;
%if( (t>4.0) & (t<6.1) ) External_moment = 10000.0; end
%For counter-steering, need to use the following parameters
%f(20) = (Torque_susp_roll*1.0 + 5.68*ms*g*hs*sin(phi) + Ftire_lat*hr*cos(phi) + External_moment)/(Ix+ms*hr*hr);



%Complete roll over
%rollover  = 0;
if( (zu_fl - zr_fl)> (F_tire_fl_static/kt)) ROLLOVER = 1; end
if( (zu_fr - zr_fr)> (F_tire_fr_static/kt)) ROLLOVER = 1; end
if( (zu_rl - zr_rl)> (F_tire_rl_static/kt)) ROLLOVER = 1; end
if( (zu_rr - zr_rr)> (F_tire_rr_static/kt)) ROLLOVER = 1; end

if( (zu_fl - zr_fl)> 0 ) ROLLOVER = 1; end
if( (zu_fr - zr_fr)> 0 ) ROLLOVER = 1; end
if( (zu_rl - zr_rl)> 0 ) ROLLOVER = 1; end
if( (zu_rr - zr_rr)> 0 ) ROLLOVER = 1; end


if( ROLLOVER == 1)
    t
f(20) = (m*g*hs*sin(phi) + Ftire_lat*hr*cos(phi))/(Ix+ms*hr*hr);  %-m*g*hs; %*sin(phi);
 %if( (phi >= pi/2) | (phi < -pi/2) ) f(20) = 0.0; f(14) = 0.0; end
end



%Lateral dynamics, including the effect of roll on lateral acceleration
f(17) = (Ftire_lat - m*Vx*psi_dot)/m ;  %+ ms*hr*(f(20)*cos(phi)-phi_dot*phi_dot*sin(phi))/m;


% Pitch dynamics
Torque_susp_pitch = -Lf*(Fsusp_fl + Fsusp_fr) + Lr*(Fsusp_rl + Fsusp_rr);
Torque_tire_pitch = - Ftraction*hs;
f(21) = (Torque_susp_pitch + Torque_tire_pitch*cos(theta) + ms*g*hs*cos(theta))/Iy;

% Yaw angle
f(22) = x(19);

%Integral of steering angle
f(23) = delta;


% if(t>4.99 & t<5.01) 
%     disp('t Delta Delta_Old Delta_derivative Yaw moment')
%     [t delta delta_counter_old delta-delta_counter_old delta_derivative Torque_tire_yaw]
% end


% Additional states for steering control using lateral position feedback

%psi_des_dot
% x_vec and road_vec are longitudinal position and corressponding road curvature changed values
% road_curv_index
% x(28)
% x_vec(road_curv_index)
if(road_curv_index < road_curv_index_max)
if( x(28) >= x_vec(road_curv_index+1) )
    road_curv_index = road_curv_index+1;
end
end
road_curv = road_curv_vec(road_curv_index);
psi_des_dot = x(16) * road_curv;

%Lateral position error dynamics e_1
f(24) = x(25); 
f(25) = f(17) + x(16)*(x(19)-psi_des_dot);

% Yaw angle error dynamics
f(26) = x(19)-psi_des_dot;

% Absolute yaw angle dynamics (NOT yaw angle error), but x(22)is already yaw angke
f(27) = x(19);

%Longitudinal position dynamics
f(28) = x(16); 

% Road related states: x(29), x(30) and x(31)
% x(29) psi_des
% x(30) road_x
% x(31) road_y
f(29) = psi_des_dot;
f(30) = x(16) * cos(x(29));
f(31) = x(16) * sin(x(29));