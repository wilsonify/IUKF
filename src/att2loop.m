%%% MATLAB CODE 11.02 +++++++++++++++++++++++++++++++++++++++
% att2loop-Timing loop function to determine 2-D attitude
% using signals from the gyroscope and accelerometer of
% an Inertia Measurement Module (IMU)
%
%SYNTAX:[PAd,xAm,KGd] =     %att2loop(GyroXYZ,AcceleroNORM,dt,N,tstmp,Q,R,P0);
%
function [PAd,xAm,KGd] = att2loop(GyroXYZ,AcceleroNORM,dt,N,tstmp,Q,R,P0);
% Kalman Filter Initialization
H = eye(4);
x = [1 0 0 0]';
P = P0;
u = zeros(N,1); % There are no â€œControl Inputsâ€?
PAd = zeros(4,N); %Storage for all PA matrix diagonals
xAm = zeros(4,N); %Storage for all posterior state vect.
KGd = zeros(4,N); %Storage for all KG matrix diagonals
% PTP: Matrix where we'll save the Phi, Theta and Psi
% angles calculated directly from accelerometer readings
PTP = zeros(N,3);
% filtPTP: Matrix where we'll save the Phi, Theta and Psi
% angles obtained from the quaternion output of the KF
filtPTP = zeros(N,3);
for t=1:1:N %% BEGINNING OF THE TIMING LOOP
% Reading the 3 signals from the gyroscopes
a = -GyroXYZ(t,3);
b = -GyroXYZ(t,1);
c = GyroXYZ(t,2);
OMEGA = [0 -a -b -c;
a 0 c -b;
b -c 0 a;
c b -a 0];
F = eye(4) + (dt/2) * OMEGA;
G = [0; 0; 0; 0];
% Calculate Euler Angles from accelerometer measurement
ax = AcceleroNORM(t,3);
ay = AcceleroNORM(t,1);
theta = asin( ax );
phi = asin( -ay/(cos(theta)) );
psi = 0;
tht2 = theta / 2;
phi2 = phi /2;
psi2 = psi /2;
% Convert Euler Angles into Quaternion
z = [ cos(phi2)*cos(tht2)*cos(psi2) + sin(phi2)*sin(tht2)*sin(psi2);
sin(phi2)*cos(tht2)*cos(psi2)-cos(phi2)*sin(tht2)*sin(psi2);
cos(phi2)*sin(tht2)*cos(psi2) + sin(phi2)*cos(tht2)*sin(psi2);
cos(phi2)*cos(tht2)*sin(psi2)-sin(phi2)*sin(tht2)*cos(psi2)];
% KALMAN FILTER Iteration
[PA, xA, KG] =onedkf(F,G,Q,H,R,P,x,u(t),z);
%[PA, xA, KG] =onedkfkg0(F,G,Q,H,R,P,x,u(t),z); with KG=0 
PAd(1:4,t) = diag(PA); % stores current Diag. of PA 
KGd(1:4,t) = diag(KG); % stores current Diag. of KG 
x = xA; %The posterior estimate will be used for the rest
% of the steps
P = PA;
filtPhi = atan2( 2*(x(3)*x(4) + x(1)*x(2)) , 1-2*(x(2)^2 + x(3)^2) );
filtTheta = -asin( 2*(x(2)*x(4)-x(1)*x(3)) );
filtPsi = atan2( 2*(x(2)*x(3) + x(1)*x(4)) , 1-2*(x(3)^2 + x(4)^2) );
% storing results for display after completion of loop
PTP(t,:) = (180/pi)*[phi, theta, psi];
filtPTP(t,:) = (180/pi)*[filtPhi, filtTheta, filtPsi];
fPhi2 = filtPhi/2;
fPsi2 = filtPsi/2;
fTht2 = filtTheta/2;
fQuat(t,:)=[cos(fPhi2)*cos(fTht2)*cos(fPsi2) + sin(fPhi2)*sin(fTht2)*sin(fPsi2);
sin(fPhi2)*cos(fTht2)*cos(fPsi2)-cos(fPhi2)*sin(fTht2)*sin(fPsi2);
cos(fPhi2)*sin(fTht2)*cos(fPsi2) + sin(fPhi2)*cos(fTht2)*sin(fPsi2);
cos(fPhi2)*cos(fTht2)*sin(fPsi2)-sin(fPhi2)*sin(fTht2)*cos(fPsi2)]';
end %% END OF THE TIMING LOOP
xAm = transpose(fQuat); % Save a matrix with all 
% the posterior state vectors
% Plotting
gray6 = [0.6, 0.6, 0.6];
figure;
sp(1)=subplot(4,1,1); hold on;
plot(tstmp,-GyroXYZ(:,3),'Color',gray6,'Linewidth',1);
plot(tstmp,-GyroXYZ(:,1),'k--','Linewidth',1);
plot(tstmp,GyroXYZ(:,2),'r','Linewidth',1);
title('Gyroscope Measurement');
ylabel('Ang Vel(rad/s)');
axis([tstmp(1) tstmp(end) -3 3]);
set(gca,'Xtick',0:2:50);
legend('Roll','Pitch','Yaw','Location','BestOutside');
grid on;
sp(2)=subplot(4,1,2); hold on;
plot(tstmp,PTP(:,1),'Color',gray6,'Linewidth',1.5);
plot(tstmp,PTP(:,2),'k--','Linewidth',1.5);
title('Euler Angles from Accelerometer Measurement');
ylabel('Angles (deg)');
axis([tstmp(1) tstmp(end) -180 180]);
set(gca,'Ytick',-180:45:180,'Xtick',0:2:50);
legend('Phi(\phi)','Theta(\theta)','Location','BestOutside');
grid on;
sp(3)=subplot(4,1,3); hold on;
plot(tstmp,filtPTP(:,1),'Color',gray6,'Linewidth',1.5);
plot(tstmp,filtPTP(:,2),'k--','Linewidth',1.5);
title('Euler Angles Output from Kalman Filter');
xlabel('Time (seconds)');
ylabel('Angles (deg)');
axis([tstmp(1) tstmp(end) -180 180]);
set(gca,'Ytick',-180:45:180,'Xtick',0:2:50);
legend('Phi(\phi)','Theta(\theta)','Location','BestOutside');
grid on;
sp(4)=subplot(4,1,4); hold on;
plot(tstmp,fQuat(:,1),'b','Linewidth',1);
plot(tstmp,fQuat(:,2),'k--','Linewidth',1);
plot(tstmp,fQuat(:,3),'Color',gray6,'Linewidth',1);
plot(tstmp,fQuat(:,4),'r','Linewidth',1);
title('Quaternion Output from Kalman Filter');
xlabel('Time (seconds)');
axis([tstmp(1) tstmp(end) -1.1 1.1]);
set(gca,'Xtick',0:2:50);
legend('w','x','y','z','Location','BestOutside');
grid on;
linkaxes(sp,'x');
end % end of att2lopp
%%% MATLAB CODE 11.02 +++++++++++++++++++++++++++++++++++++++