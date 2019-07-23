function [Ft_lat,Ft_long] = tire_model(slip_angle,slip_ratio,Fz)
%tire_model.m

%vehicle_parameters;

Ktire = 81030.0/4; %N
Ft_long = Ktire*slip_ratio;

Cs = 60000;
Ft_lat = Cs*slip_angle;
mu_friction = 1;  %0.5; %1; %0.3; %1;
%Fz = 1.0*0.25*(ms + 4*m_u)*9.81;

%Dugoff model
sigma = slip_ratio;
alfa = slip_angle;
Csigma = Ktire;
Calfa = Cs;
if(abs(sigma)>0)
lambda = mu_friction*Fz*(1+abs(sigma))/(2*sqrt(Csigma*Csigma*sigma*sigma+Calfa*Calfa*tan(alfa)*tan(alfa)));
else
    lambda=100.0;
end
if (lambda < 1) f_lambda =lambda*(2-lambda); end
if(lambda >= 1) f_lambda = 1; end
Ft_long = Csigma * sigma* f_lambda/(1+abs(sigma)) ; 
Ft_lat = Calfa * tan(alfa)* f_lambda/(1+abs(sigma)) ;

 % Below is the Magic Formula Model

% x = slip;
% %Use Sh to calculate x from xbig: x = xbig+Sh; where xbig = slip;
% 
% model_case = 1;
% 
% if(model_case == 1)
%     %Longitudinal parameters
%     Sv = 0.0;
%     Sh = 0.0;
%     xm = 0.01;
%     D = 0.9*xm*Ktire;
%     C = 1.65;
%     B = Ktire/(C*D);  % BCD = Ktire
% end
% 
% if(model_case == 2)
%     %Lateral parameters
%     C = 1.3;
% end
% 
% if(model_case == 3)
%     %Aligning torque parameters
%     C = 2.4;
% end
% 
% E = (B*xm - tan(pi/(2*C)))/(B*xm - atan(B*xm));
% 
% y = D*sin(C*atan(B*x-E*(B*x-atan(B*x))));
% 
% ybig = y + Sv;
% %Use Sh to calculate x from xbig: x = xbig+Sh;
% 
% Ft = ybig;