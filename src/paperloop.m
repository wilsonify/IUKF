%%% MATLAB CODE 10.01 +++++++++++++++++++++++++++++++++++++++
% paperloop.m—timing loop for simulation of Kalman
% Filter applied to falling motion for the estimation
% of instantaneous height and speed for the hypothetical % falling paper wad as discussed in Chapter 5.
%
% SYNTAX:[XAVECT,PAVECT,KGVECT]=
% paperloop(zvect,u,y0,P0,Q,R,DT,iter);
function[XAVECT,PAVECT,KGVECT]=paperloop(zvect,u,y0,P0,Q,R,DT,iter);
F = [ 1 , DT ; 0 , 1 ];
G = eye(2);
H = [1 , 0];
x = y0;
P = P0;
% Measurement time series
z = zvect;
% Set up vectors to store selected elements of xA and PA % from all iterations
PAVECT = zeros(1, iter); %we will only store the
% variance of yk fr0m PA
XAVECT = zeros(2,iter); %we will store both yk and y’k
KGVECT = zeros(2, iter); %we will store both values in
% KG, which will be 2x1 vector
for t = 1:iter %%% ----- START OF TIMING LOOP
% [PA, xA, KG] = onedkf(F,G,Q,H,R,P,x,u(:,t),z(t));
[PA, xA, KG] = onedkfkg0(F,G,Q,H,R,P,x,u(:,t),z(t));
PAVECT(t) = PA(1,1); % we are only storing the variance
% of the first state variable
% which is yk, located in cell (1,1) of PA
XAVECT(:, t) = xA; % we will store the estimates of 
% both state variables contained in xA
KGVECT(:,t) = KG;
% pass results as inputs to the NEXT iteration:
P = PA;
x = xA;
end %%% ----- END OF TIMING LOOP
end
%%% MATLAB CODE 10.01 +++++++++++++++++++++++++++++++++++++++