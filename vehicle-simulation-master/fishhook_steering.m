function delta = fishhook_steering(t,t_old,delta_old,Vx)
%vehicle_parameters;
tsamp = t - t_old;

delta = 0.0;
if(t>=5.0)
    Amp = 0.1; %Angle at 0.3g
    ramp_rate = (720*pi/180)/150;
    ramp_rate2 = 0.123*ramp_rate;
    t1 = 5 + Amp/ramp_rate;
    t2 = 5+ Amp/ramp_rate + 0.25;
    t3 = 5+ Amp/ramp_rate + 0.25 + 2*Amp/ramp_rate;
    t4 = 5+ Amp/ramp_rate + 0.25 + 2*Amp/ramp_rate + 3;
    t5 = t4 + Amp/ramp_rate2;
    %Step 1
    if(t <= t1) 
        delta = ramp_rate*(t-5);
    end
    %Step 2 Dwell
    if(t> t1 & t<= t2)
        delta = Amp;
    end
    %Step 3- Negative ramp
    if(t> t2 & t <= t3)
        delta = Amp - ramp_rate*(t-t2);
    end
    %Step 4 Dwell
    if(t> t3 & t<= t4)
        delta = -Amp;
    end
    %Step 5 - Ramp back to zero
    if(t> t4 & t<=t5)
        delta = -Amp + ramp_rate2*(t-t4);
    end
    %Step 6 Zero
    if(t>t5)
        delta = 0.0;
    end
end


