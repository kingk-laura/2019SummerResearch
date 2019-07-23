function Rindex = rollover_index(ay,phi,hr,hs,Lw)

%vehicle_parameters;
g = 9.81;
Rindex = ay*2*hr/(Lw*g);
Rindex = Rindex + (2*hs/Lw)*tan(phi);