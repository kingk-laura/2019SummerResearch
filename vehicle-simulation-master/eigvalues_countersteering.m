clear all;
vehicle_parameters;
lambda = 0*0.25*Cf;
V = 30;

A = zeros(4,4);
B = zeros(4,1);
%States are ydot, psi_dot, phi and phi_dot

hold on

vsize = 140;
eigs_cl = zeros(vsize,4);

for jklm=1:vsize
V = (jklm+1)*1*0.25;
if(jklm < 31) V = 0.03*jklm;
else
    V = (jklm-27)*0.25;
end
    
%ydot dynamics
A(1,2) = -1/V;
B(1,1) = Cf/(m*V);
A(1,1) = (-Cf-Cr)/(m*V);
A(1,2) = A(1,2) + (-Cf*Lf+Cr*Lr)/(m*V);
A(1,3) = -lambda/m;

%psi_dot dynamics
B(2,1) = Lf*Cf/Iz;
A(2,1) = (-Lf*Cf+Lr*Cr)/(V*Iz);
A(2,2) = (-Lf*Lf+Cf-Lr*Lr*Cr)/(V*Iz);
A(2,3) = -lambda*(Lf-Lr)/Iz;

%phi and phidot dynamics
A(3,4) = 1.0;
B(4,1) = (hr*Cf)/(Ix+m*hr*hr);
A(4,1) = (-hr*Cf/V -hr*Cr/V)/(Ix+m*hr*hr);
A(4,2) = (-hr*Cf*Lf/V + hr*Cr*Lr/V)/(Ix+m*hr*hr);
A(4,3) = (-hr*lambda - 0.5*ks*Lw*Lw)/(Ix+m*hr*hr);
A(4,4) = (-0.5*bs*Lw*Lw)/(Ix+m*hr*hr);


%eig(A)

%Feedback control by counter-steering
kp = -1;
kd = -sqrt(-kp);
K = [0 0 -kp -kd];
Acl = A - B*K;
eigs_cl(jklm,:) = eig(Acl)';
plot(real(eig(Acl)),imag(eig(Acl)),'o',real(eig(Acl)),imag(eig(Acl)))
end
hold off
pause
close all
hold on
% for jklm = 1:4
% plot(real(eigs_cl(:,jklm)),imag(eigs_cl(:,jklm)),'o');
% end

plot(real(eigs_cl(:,1)),imag(eigs_cl(:,1)),'r',real(eigs_cl(:,1)),imag(eigs_cl(:,1)),'ro');
plot(real(eigs_cl(:,2)),imag(eigs_cl(:,2)),'b',real(eigs_cl(:,2)),imag(eigs_cl(:,2)),'b*');
plot(real(eigs_cl(:,3)),imag(eigs_cl(:,3)),'g',real(eigs_cl(:,3)),imag(eigs_cl(:,3)),'go');
plot(real(eigs_cl(:,4)),imag(eigs_cl(:,4)),'c',real(eigs_cl(:,4)),imag(eigs_cl(:,4)),'c*');

pause
close all

sorted_real = zeros(vsize,4);
sored_imag = zeros(vsize,4);
for hklm = 1:vsize
[sorted_real(hklm,:),ijk] = sort(real(eigs_cl(hklm,:)));
sorted_imag(hklm,:) = imag(eigs_cl(hklm,ijk));
end

%plot(sorted_real,sorted_imag);
hold on
plot(sorted_real(:,1),sorted_imag(:,1),'r',sorted_real(:,1),sorted_imag(:,1),'ro');
plot(sorted_real(:,2),sorted_imag(:,2),'b',sorted_real(:,2),sorted_imag(:,2),'b*');
plot(sorted_real(:,3),sorted_imag(:,3),'g',sorted_real(:,3),sorted_imag(:,3),'go');
plot(sorted_real(:,4),sorted_imag(:,4),'c',sorted_real(:,4),sorted_imag(:,4),'c*');
plot(sorted_real(1,:),sorted_imag(1,:),'ks','Linewidth',2)
plot(sorted_real(vsize,:),sorted_imag(vsize,:),'kd','Linewidth',2)
text(-1500,1.5,'squares: Low Speed 0.1 m/s');
text(-1500,1.0,'diamonds: High Speed 30 m/s');
pause
axis([-10 6 -2.5 2.5]);
text(2,2,'squares: Low Speed 0.1 m/s');
text(2,1.5,'diamonds: High Speed 30 m/s');
hold off
