%%% MATLAB CODE 09.01 +++++++++++++++++++++++++++++++++++++++
% gizmosim.m—top-level function for simulation of univariate Kalman Filter
% which estimates an inaccessible voltage from rotation
% speed in the hypothetical gizmo described in Chapter 4
%
% SYNTAX: [XAVECT, PAVECT, KGVECT] = gizmosim(xtru, x0,
% P0, Q, R, iter);
function [XAVECT,PAVECT,KGVECT] = gizmosim(xtru,x0,P0,Q,R,iter);
rng(12345,'v5normal'); % Resets the Random Number
% Generator to ver5. Normal, seed = 12345;
% No u, but there is external noise represented by 
% Q = variance of noise
% Creation of vector of TRUE voltage values:
xtrue_nonoise = ones(iter,1) * xtru;
extnoise = randn(iter,1) * ( sqrt(Q) );
xtrue = xtrue_nonoise + extnoise;
% Creation of vector of MEASURED values, taking 
% into account that the measuring system ADDS noise to
% each TRUE VALUE measured:
measerror = randn(iter,1) * (sqrt(R));
zvect = (xtrue * 5) + measerror;
[XAVECT,PAVECT,KGVECT]=gizmoloop(xtrue,zvect,x0,P0,Q,R,iter);
zvecdiv5 = zvect ./5 ;
gray6 = [0.6, 0.6, 0.6];
figure;plot(xtrue,'Color',gray6,'LineWidth',1.5);hold on;
plot(XAVECT,'k','LineWidth',1.5 );
plot(zvecdiv5,'k-.','LineWidth',1.5);
hold off; title('xtrue, xA and z/5');axis([0,100,0,9]);
xlabel('Kalman Filter Iterations');
legend('xtrue','xA','zvecdiv5','Location','southeast');
grid off;
figure; plot(PAVECT,'k','LineWidth',1.5);title('PA');grid;
xlabel('Kalman Filter Iterations');
figure; plot(KGVECT,'k','LineWidth',1.5);
title('KG values');grid;
xlabel('Kalman Filter Iterations');
end
%%% MATLAB CODE 09.01 +++++++++++++++++++++++++++++++++++++++
