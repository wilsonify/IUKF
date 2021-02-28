%%% MATLAB CODE 08.01 +++++++++++++++++++++++++++++++++++++++
% % % % % % % % % % % % % % % % % % % % % % %
% Function onedkf-Implements A SINGLE ITERATION OF
% THE DISCRETE-TIME KALMAN FILTER ALGORITHM
%
% SYNTAX: [PA, xA, KG] = onedkf(F,G,Q,H,R,P,x,u,z);
% Uses(receives) matrices F, G for the model equations
% Uses(receives) the process noise covariance matrix, Q.
% Uses(receives) matrix H for the measurement equation.
% Uses(receives) the measurement noise cov. matrix, R.
% The following are expected to change in every % iteration:
% Receives the state vector, x, and its cov. matrix, P,
% from the previous iteration of the algorithm.
% Receives the current vector of inputs, u.
% Receives the current vector of measurements, z.
% Performs the prediction and Correction Phases.
% Returns the POSTERIOR estimation of state vector, xA
% and its covariance matrix, PA.
% It also returns the calculated KG matrix.
%
function [PA, xA, KG] = onedkf(F,G,Q,H,R,P,x,u,z);
%%% PREDICTION PHASE, using the MODEL:
% Equation (8.1)- The MODEL predicts the new x:
FT = transpose(F);
xM = F * x + G * u ;
% Equation (8.2)- The MODEL predicts the new P:
PM = F * P * FT + Q ;
%% Chge. of variables, to clearly separate the 2 phases:
xB = xM ; % Equation 8.3
PB = PM ; % Equation 8.4
%%% CORRECTION (UPDATE) PHASE
% -Finding POSTERIOR (A = After) parameters,
% from PRIOR (B = before), through Bayesian
% estimation:
HT = transpose(H);
% First calculate the Kalman Gain, KG, for this iteration
% (Equation 8.5):
KG = PB * HT * ( inv( H * PB * HT + R) ) ;
% Equation (8.6)-Calculate POSTERIOR ESTIMATE of the
% state vector
xA = xB + KG * (z-H * xB) ;
% Equation (8.7)-Calc. POSTERIOR ESTIMATE of state	% vector's covar. mtx.
PA = PB-KG * H * PB;
end
% % % % % % % % % % % % % % % % % % % % % % %
%%% MATLAB CODE 08.01 +++++++++++++++++++++++++++++++++++++++