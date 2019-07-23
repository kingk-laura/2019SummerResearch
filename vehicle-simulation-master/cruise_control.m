function Tdrive = cruise_control(Vx,Vx_des);

global ROLLOVER ROLLOVER_WARNING sum_error t_old delta_old new_sum initial_vehicle_speed surint

Tdrive = 10*(-1000*(Vx-Vx_des)-0.01*200*sum_error);
sum_error = sum_error + Vx - Vx_des;