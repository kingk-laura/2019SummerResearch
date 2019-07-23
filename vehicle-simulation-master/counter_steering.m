function [delta,new_sum] = counter_steering(t,delta_original,roll_angle,roll_angle_derivative,old_sum)



%Counter_steering starts

%Without suspension
% ks_gain = -33.0;  %-29 works
% desired_tilt_angle = ks_gain* delta_original;
% tilt_error = roll_angle - desired_tilt_angle;
% sum = old_sum + tilt_error;
% kp_gain = 5*10*0.1; %5 works
% kd_gain = sqrt(kp_gain)*2.0; %1 works
% ki_gain = -0.001*0;
% delta = - kp_gain* tilt_error -kd_gain* roll_angle_derivative - ki_gain*sum + delta_original*0.1; %+ ki_gain*sum;

%With suspension
%Multiply the mgh*sin(phi) term by 5.75 in xprime.m, use all gains of same value as the without suspension case
ks_gain = -33.0*1.0;  %
desired_tilt_angle = ks_gain* delta_original;
tilt_error = roll_angle - desired_tilt_angle;
sum = old_sum + tilt_error;
kp_gain = 5*10*0.1; %5 works
kd_gain = sqrt(kp_gain)*5.0; %1 works
ki_gain = -0.001*0;
delta = 0.0;
if(t>5)
delta = - kp_gain* tilt_error -kd_gain* roll_angle_derivative - ki_gain*sum + delta_original*0.1; %+ ki_gain*sum;
delta = - kp_gain* tilt_error -kd_gain* roll_angle_derivative - ki_gain*sum + delta_original*0.0 + 0*2.5*roll_angle; %+ ki_gain*sum;
end

%Open-loop initial counter-steering 
% if( (t>5) & (t<6))
% delta = -1*0.1*sin(2*pi*(t-5)*0.5) + delta_original;
% else
%     delta = delta_original;
% end



new_sum = sum;

%Without counter_steering
% delta = delta_original;
% delta = -1* delta_original;
% new_sum = 0.0;