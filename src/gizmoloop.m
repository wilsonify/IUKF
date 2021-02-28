%%% MATLAB CODE 09.02 +++++++++++++++++++++++++++++++++++++++
% gizmoloop.m—timing loop for simulation of univariate
% Kalman Filter for the estimation of an inaccessible 
% voltage from rotation speed in the hypothetical 
% gizmo described in Chapter 4
%
% SYNTAX:[XAVECT, PAVECT,KGVECT]
% =gizmoloop(xtrue,zvect,x0,P0,Q,R,iter);
function [XAVECT, PAVECT, KGVECT]=gizmoloop(xtrue,zvect,x0,P0,Q,R,iter);
F = 1;
G = 0;
H = 5;
x = x0;
P = P0;
% Input time series
u = zeros(iter,1);
z = zvect;
% Set up vectors to store outputs (all iterations)
PAVECT = zeros(iter,1);
XAVECT = zeros(iter,1);
KGVECT = zeros(iter,1);
for t = 1:iter %%% ----- START OF TIMING LOOP
 [PA, xA, KG] = onedkf(F,G,Q,H,R,P,x,u(t),z(t));
% [PA, xA, KG] = onedkfkg0(F,G,Q,H,R,P,x,u(t),z(t));
PAVECT(t) = PA;
XAVECT(t) = xA;
KGVECT(t) = KG;
% pass results as inputs to the NEXT iteration:
P = PA;
x = xA;
end %%% ----- END OF TIMING LOOP
end
%%% MATLAB CODE 09.02 +++++++++++++++++++++++++++++++++++++++