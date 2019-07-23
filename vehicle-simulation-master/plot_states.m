%plot_states
%load reference;
if(type_of_steering == 1) load reference_step; end
if(type_of_steering == 4) load reference_step; end
% If 15 m/s instead of 30 m/s, then load reference_step_15mps INSTEAD
if(type_of_steering == 2) load reference_fishhook; end
Vx_vec = x(:,16);
psi_dot_vec = x(:,19);
y_dot = x(:,17);
phi_vec = x(:,14);


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
beta = atan(x(:,17) ./x(:,16)); %Ratio of lateral and longitudinal speeds
yaw_angle = x(:,22);
Xglobal_dot = x(:,16) .* cos(yaw_angle+beta);
Yglobal_dot = x(:,16) .* sin(yaw_angle+beta);
%sys_integrate = tf(1,[1 0]); %Pure Integrator
%Xglobal = lsim(sys_integrate,Xglobal_dot,t,0);
%Yglobal = lsim(sys_integrate,Yglobal_dot,t,0);
sim('integrator');

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

tau_diff = 0.02;
sim('differentiator');
plot_tire_forces;
%break
pause
yaw_rate_des_vec = desired_yaw_rate(Vx_vec,delta_needed_vec);
for jklm=1:sim_size
if(type_of_ESC == 2)
if(abs(yaw_rate_des_vec(jklm,1))>psi_dot_bound)
    yaw_rate_des_vec(jklm,1) = psi_dot_bound*sign(yaw_rate_des_vec(jklm,1));
end
end
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
ay_meas_vec  = y_dotdot + Vx_vec .* psi_dot_vec;
Rindex_vec = rollover_index(ay_meas_vec,phi_vec);
plot(t,Rindex_vec,'r',t_ref,Rindex_ref,'b');
xlabel('time(sec)')
title('Rollover index')
legend('Actual','Reference');
