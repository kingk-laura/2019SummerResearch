function Tb = ESC(yaw_rate,Vx,delta_needed,Ftire_lat_vec,Lw,Lf,Lr,Iz,reff,par_vec)

%Electronic Stability Control
%vehicle_parameters;
mass = par_vec(1);
Lf = par_vec(2);
Lr = par_vec(3);
Cf = par_vec(4);
Cr = par_vec(5);
Lw = par_vec(6);
Iz = par_vec(7);
reff = par_vec(8);


Ftire_lat_fl = Ftire_lat_vec(1,1);
Ftire_lat_fr = Ftire_lat_vec(2,1);
Ftire_lat_rl = Ftire_lat_vec(3,1);
Ftire_lat_rr = Ftire_lat_vec(4,1);

yaw_rate_des = 1.0*desired_yaw_rate(Vx,delta_needed,Lf,Lr,mass,Cf,Cr);

psi_dot_bound = 1.0;
if(abs(yaw_rate_des)>psi_dot_bound)
    yaw_rate_des = psi_dot_bound*sign(yaw_rate_des);
end

surface = yaw_rate - yaw_rate_des;
eta = 10; %50000;
Mpsi_b = (-Lf*(Ftire_lat_fl+Ftire_lat_fr)+Lr*(Ftire_lat_rl+Ftire_lat_rr)-eta*Iz*surface)/(2);
Tb = reff*2*Mpsi_b/Lw;

%Trial, P and PI control
% surint = surint + surface;
% Mpsi_b = (-eta*Iz*surface - eta*0.1*Iz*surint)/(2);
% Tb = reff*2*Mpsi_b/Lw;


%ESC + Lower Level Controller Allocation for Rollover Prevention
%if(abs(Rindex) > 0.5)
%Tb = reff*2*Mpsi_b/Lw + (Rindex-0.5)*300;
%end