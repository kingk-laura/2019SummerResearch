function delta = steering(t,t_old,delta_old,Vx)
vehicle_parameters;
tsamp = t - t_old;
tau_st = 1.0;
delta_raw = 0;
delta_ss = (Lf+Lr)/radius + m*Vx^2/radius * (Lr/(2*Cf*(Lf+Lr))-Lf/(2*Cr*(Lf+Lr)));
if(t>5.0) delta_raw = delta_ss; end
%Filtering of steering input
if(tsamp ~=0)
delta = ( -(1-tau_st/tsamp)*delta_old + delta_raw )*tsamp/tau_st;
else
    delta = delta_old;
end