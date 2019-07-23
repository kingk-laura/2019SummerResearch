clear;

%Parameters
vehicle_parameters;
global ROLLOVER ROLLOVER_WARNING sum_error t_old delta_old new_sum initial_vehicle_speed surint x_vec road_curv_index road_curv_vec road_curv_index_max
surint = 0;
ROLLOVER = 0;
ROLLOVER_WARNING = 0;
sum_error = 0;
t_old = -0.001;
delta_old = 0;
new_sum = 0.0;
old_phi = 0.0;
delta_counter_old = 0.0;
delta_derivative_old = 0.0;

%Initial conditions
x0 = zeros(31,1);

initial_vehicle_speed = 30; %For step steering %15.0; %40.0; %15.0; %30.0; %40.0
%initial_vehicle_speed = 30.0; %For counter-steering
if(type_of_steering == 2)
initial_vehicle_speed = 15.0; %For fishhook steering
end
initial_wheel_speed = initial_vehicle_speed/rwheel;
x0(1,1) = initial_wheel_speed;
x0(2,1) = initial_wheel_speed;
x0(3,1) = initial_wheel_speed;
x0(4,1) = initial_wheel_speed;
x0(16,1) = initial_vehicle_speed;

% Road curvature specification
road_curv_index = 1;

x_vec = [0 300 1000]';
road_curv_vec = [0 1/500 1/1000]';
road_curv_index_max = 3;

x_vec = [0 initial_vehicle_speed*5]';
road_curv_vec = [0 1/300]';
road_curv_index_max = 2;

%Simulation set up
t0 = 0.0;
tsamp = 0.01;
tfinal = 40.0;
npts = 1200;

%Simulation
tspan = [t0 tfinal];
options=odeset('MaxStep',0.02);
%options=odeset('MaxStep',0.1);
[t1,x1] = ode45(@xprime,tspan,x0,options);

display('Finished Simulation')
t = t0:0.005:tfinal; t=t';
x = interp1(t1,x1,t);
%display('Finished Interpolation');

%plot_states;

setup_for_plotting;
display('Finished Setting Up Variables for Plots')
if(yes_to_plotting == 1)
    plot_all_variables;
end

datam = [t Xglobal Yglobal yaw_angle x(:,14)];
%save rolldata.txt datam -ascii
csvwrite('rolldata.txt',datam);
%type rolldata.txt

dataroad = [t x(:,30) x(:,31) x(:,29)];
if( (type_of_steering == 1) | (type_of_steering == 2))
    dataroad = [t_global_ref Xglobal_ref Yglobal_ref yaw_angle_ref];
    dataroad = [t Xglobal Yglobal yaw_angle];
    % Road is the same as the vehicle's trakectory
    % There is no road, really
end 
csvwrite('roaddata.txt',dataroad);

pdata = [t Vx_vec psi_dot_vec y_dot beta yaw_angle ay_meas_vec Xglobal Yglobal Ftire_lat_fl_vec Ftire_lat_fr_vec Ftire_lat_rl_vec Ftire_lat_rr_vec Ftire_long_fl_vec Ftire_long_fr_vec Ftire_long_rl_vec Ftire_long_rr_vec slip_ratio_fl_vec slip_ratio_fr_vec slip_ratio_rl_vec slip_ratio_rr_vec slip_angle_fl_vec slip_angle_fr_vec slip_angle_rl_vec slip_angle_rr_vec];
csvwrite('plotdata.txt', pdata);
