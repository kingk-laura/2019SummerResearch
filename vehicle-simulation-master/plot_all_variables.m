%plot_all_variables.m

plot(t,x(:,16));
xlabel('time (sec)')
ylabel('speed (m/s)')
title('Vehicle speed')
pause
plot(t,x(:,19));
xlabel('time (sec)')
ylabel('yaw rate (rad/s)')
title('Yaw rate')
pause
plot(t,x(:,14)*180/pi,'r',t_ref,roll_angle_ref*180/pi,'b');
xlabel('time (sec)')
ylabel('roll (deg)')
title('Roll angle')
legend('Actual','Reference');
pause

if(type_of_steering == 1 | type_of_steering == 2)
plot(Xglobal,Yglobal,'r',Xglobal_ref,Yglobal_ref,'b');
%axis([0 700 -50 650]);
xlabel('X (m)');
ylabel('Y (m)');
legend('Actual','Reference');
pause
end

if(type_of_steering == 4)
plot(Xglobal,Yglobal,'r',x(:,30),x(:,31),'b');
xlabel('X (m)');
ylabel('Y (m)');
title('red: vehicle trajectory  blue:road')
pause
end


plot(t,yaw_rate_des_vec,'b',t,x(:,19),'r');
title('Yaw rate: red - actual    blue - desired');
pause
plot(t,beta);
title('Slip angle')
pause
plot(t,delta_vec,'r',t_ref,delta_ref,'b');
xlabel('time (sec)')
ylabel('delta (rad)')
title('Steering angle')
legend('Actual','Reference');
pause

%Plot rollover index
plot(t,Rindex_vec,'r',t_ref,Rindex_ref,'b');
xlabel('time(sec)')
title('Rollover index')
legend('Actual','Reference');

%Plot tire forces
plot(t,Ftire_lat_fl_vec,t,force_limit*ones(sim_size,1),'r',t,-force_limit*ones(sim_size,1),'r');
plot(t,Ftire_lat_fl_vec);
title('Lateral front left tire force')
pause
plot(t,Ftire_lat_fr_vec);
title('Lateral front right tire force')
pause
plot(t,Ftire_lat_rl_vec);
title('Lateral rear left tire force')
pause
plot(t,Ftire_lat_rr_vec);
title('Lateral rear right tire force')
pause
plot(t,Ftire_long_fl_vec);
title('Longitudinal front left tire force')
pause
plot(t,Ftire_long_fr_vec);
title('Longitudinal front right tire force')
pause
plot(t,Ftire_long_rl_vec);
title('Longitudinal rear left tire force')
pause
plot(t,Ftire_long_rr_vec);
%plot(t,Ftire_long_rr_vec,t,force_limit*ones(sim_size,1),'r',t,-force_limit*ones(sim_size,1),'r');
title('Longitudinal rear right tire force')
