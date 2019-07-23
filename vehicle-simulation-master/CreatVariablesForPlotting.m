%Taken from "plot_states.m"
%load reference;
if(type_of_steering == 1) load reference_step; end
% If 15 m/s instead of 30 m/s, then load reference_step_15mps INSTEAD
if(type_of_steering == 2) load reference_fishhook; end
Vx_vec = x(:,16);       %Vehicle speed
psi_dot_vec = x(:,19);  %Yaw rate, rad/sec
y_dot = x(:,17);        %Lateral velocity
phi_vec = x(:,14);      %Roll angle, radians

% The variable "roll_angle_ref" is the reference roll angle for comparison


beta = atan(x(:,17) ./x(:,16)); %Slip angle, ratio of lateral and longitudinal speeds
yaw_angle = x(:,22); %Yaw angle
Xglobal_dot = x(:,16) .* cos(yaw_angle+beta);   
Yglobal_dot = x(:,16) .* sin(yaw_angle+beta);   
%sys_integrate = tf(1,[1 0]); %Pure Integrator
%Xglobal = lsim(sys_integrate,Xglobal_dot,t,0);
%Yglobal = lsim(sys_integrate,Yglobal_dot,t,0);
sim('integrator');
%Global x coordinate of vehicle = Xglobal
%Global y coordinate of vehicle = Yglobal
%Both are created automatically by running the Simulink program "integrator"


tau_diff = 0.02;
sim('differentiator');
%Replaced: plot_tire_forces;
Create_tire_forces_for_plotting;
yaw_rate_des_vec = desired_yaw_rate(Vx_vec,delta_needed_vec);
for jklm=1:sim_size
if(abs(yaw_rate_des_vec(jklm,1))>psi_dot_bound)
    yaw_rate_des_vec(jklm,1) = psi_dot_bound*sign(yaw_rate_des_vec(jklm,1));
end
end

%Desired yaw rate = yaw_rate_des_vec

%Steering angle = delta_vec
%Reference steering angle for comparison = delta_ref


%Plot rollover index
ay_meas_vec  = y_dotdot + Vx_vec .* psi_dot_vec;
Rindex_vec = rollover_index(ay_meas_vec,phi_vec);  %Rollover index

