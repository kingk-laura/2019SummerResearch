function yaw_rate_des = desired_yaw_rate(Vx,delta,Lf,Lr,mass,Cf,Cr);

%vehicle_parameters;
%mass= ms+4*m_u;
yaw_rate_des = 1*Vx .*delta ./(   Lf+Lr+(   mass*Vx .*Vx*(Lr*Cr-Lf*Cf)  )   /(2*Cf*Cr*(Lf+Lr))   );