[sim_size,junk]=size(t);

Ftire_lat_fl_vec = zeros(sim_size,1);
Ftire_lat_fr_vec = zeros(sim_size,1);
Ftire_lat_rl_vec = zeros(sim_size,1);
Ftire_lat_rr_vec = zeros(sim_size,1);
Ftire_long_fl_vec = zeros(sim_size,1);
Ftire_long_fr_vec = zeros(sim_size,1);
Ftire_long_rl_vec = zeros(sim_size,1);
Ftire_long_rr_vec = zeros(sim_size,1);
delta_vec = zeros(sim_size,1);
delta_needed_vec = zeros(sim_size,1);
delta_original_vec = zeros(sim_size,1);

t_old = -0.005;
delta_old = 0.0;
old_sum =0.0;

for ijk = 1:sim_size
    Vx = x(ijk,16);
    psi_dot = x(ijk,19);
    phi = x(ijk,14);
    phi_dot = x(ijk,20);
    wfl = x(ijk,1);
    wfr = x(ijk,2);
    wrl = x(ijk,3);
    wrr = x(ijk,4);
    ydot = x(ijk,17);
    if(type_of_steering == 1)
    delta_needed = steering(t(ijk,1),t_old,delta_old,Vx);
    delta_original = delta_needed;
    end
    if(type_of_steering == 2)
    delta_needed = fishhook_steering(t(ijk,1),t_old,delta_old,Vx);
    delta_original = delta_needed;
    end
    if(type_of_steering == 3)
    delta_needed = steering(t(ijk,1),t_old,delta_old,Vx);
    end
    t_old = t(ijk,1);
    delta_old = delta_needed;
    if(type_of_steering == 3)
    delta_original = delta_needed;
    [delta_needed,new_sum] = counter_steering(t(ijk,1),delta_original,phi,phi_dot,old_sum);
    old_sum = new_sum;
    end
    delta = active_steering_factor*delta_needed;
%     if(t(ijk,1)>4.95 & t(ijk,1)<5.1)
%         disp('t delta delta_original')
%         [t(ijk,1) delta delta_original]
%     end

%Obtain delta from delta_sim
    %delta = delta_sim(ijk,1);

    delta_vec(ijk,1) = delta;
    delta_needed_vec(ijk,1) = delta_needed;
    delta_original_vec(ijk,1) = delta_original;
    %Define slip angles
    slip_angle_fl = delta - (ydot + Lf*psi_dot)/Vx;
    slip_angle_fr = delta - (ydot + Lf*psi_dot)/Vx;
    slip_angle_rl = - (ydot - Lr*psi_dot)/Vx;
    slip_angle_rr = - (ydot - Lr*psi_dot)/Vx;
    
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

[Ftire_lat_fl_vec(ijk,1),Ftire_long_fl_vec(ijk,1)] = tire_model(slip_angle_fl,slip_ratio_fl);
[Ftire_lat_fr_vec(ijk,1),Ftire_long_fr_vec(ijk,1)] = tire_model(slip_angle_fr,slip_ratio_fr);
[Ftire_lat_rl_vec(ijk,1),Ftire_long_rl_vec(ijk,1)] = tire_model(slip_angle_rl,slip_ratio_rl);
[Ftire_lat_rr_vec(ijk,1),Ftire_long_rr_vec(ijk,1)] = tire_model(slip_angle_rr,slip_ratio_rr);



end

force_limit = 0.6*0.25*(ms + 4*m_u)*9.81;

%Lateral front left tire force = Ftire_lat_fl_vec
%Lateral front right tire force = Ftire_lat_fr_vec
%Lateral rear left tire force = Ftire_lat_rl_vec
%Lateral rear right tire force = Ftire_lat_rr_vec
%Longitudinal front left tire force = Ftire_long_fl_vec
%Longitudinal front right tire force = Ftire_long_fr_vec
%Longitudinal rear left tire force = Ftire_long_rl_vec
%Longitudinal rear right tire force = Ftire_long_rr_vec








