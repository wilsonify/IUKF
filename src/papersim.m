%%% MATLAB CODE 10.02 +++++++++++++++++++++++++++++++++++++++
% papersim-function to simulate the fall of a paper wad
% taking into account variable air resistance and 
% implementing Kalman Filter to obtain height estimates
%
% SYNTAX:[XAVECT,PAVECT,KGVECT] = 
% papersim(gback,gsd,y0tr,x0,P0,R,DT,iter);
function [XAVECT,PAVECT,KGVECT] =papersim(gback,gsd,y0tr,x0,P0,R,DT,iter);
rng(12345,'v5normal'); % Resets the Random Number
% Generator to ver5. Normal, seed =12345;
% Calculate true heights with variable air friction
g = 9.81;
gsd2 = gsd^2; %variance of the fluctuations in actualg
DT2 = DT ^2;
DT3 = DT ^3;
DT4 = DT ^4;
% Creating matrix Q according to Equation 10.14
Q =[(gsd2 * DT4 /4),(gsd2 * DT3 /2);(gsd2 * DT3 /2),(gsd2 * DT2)];
noiseg = randn(1,iter) * gsd; % creating the fluctuations for actualg
actualg = (ones(1,iter) * (g-gback) ) + noiseg;
ytr = zeros(1,iter);
% Create the 'true' time series of heights ytr
F = [ 1 , DT ; 0 , 1 ];
G = eye(2);
% H = [1 , 0];
% create u(t) in advance
u11coeff = DT2 / (-2);
u21coeff = (-1) * DT;
u = zeros(2,iter);
for t = 1:iter
u(:,t)=[(u11coeff * actualg(t));(u21coeff * actualg(t))];
end
% Create 'TRUE' height series, iterating over Eq. 10.7
y = [y0tr ; 0];
for t = 1:iter
ynext = F * y + G * u(:,t);
ytr(1,t) = ynext(1,1); % preserve in vector ytr only the
% first value in ynext, which is the height
y = ynext; % feed back the result in the model for
%next iteration
end
%%%% NOW SET UP TO CALL paperloop :
% create a z time series with the laser height
% measurements, including measurement noise
mnoise = randn(1, iter) * (-sqrt(R));
z = ytr + mnoise;
% Run the timing loop
[XAVECT, PAVECT, KGVECT]=paperloop(z,u, x0, P0, Q, R, DT, iter);
% PLOT SOME RESULTS
HeightFromKF = XAVECT(1,:);
gray6 = [0.6, 0.6, 0.6];
figure; plot(z,'Color',gray6);
hold on
plot(HeightFromKF,'k','Linewidth',1.5);
plot(ytr, 'y','Linewidth',1.5);
hold off; grid;
title('True height, KF-estimated height and rangefinder values');
ylabel('meters')
xlabel('Kalman Filter Iterations');
legend('z','HeightFromKF','ytr','Location','southwest');
% Studying the evolution of the variance of the y
% estimate (xA(1,1)):
figure; plot(PAVECT,'k','Linewidth',1.5); grid;
title('Variance of KF-estimated height');
ylabel('squared meters');
xlabel('Kalman Filter Iterations');
% Now plotting the evolution of the 1st element in KG
figure; plot(KGVECT(1,:),'k','Linewidth',1.5); grid;
title('Evolution of the first element of KG (KG1) in this example');
xlabel('Kalman Filter Iterations');
end
%%% MATLAB CODE 10.02 +++++++++++++++++++++++++++++++++++++++