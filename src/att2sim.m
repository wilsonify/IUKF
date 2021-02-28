%%% MATLAB CODE 11.01 +++++++++++++++++++++++++++++++++++++++
% att2sim.m-function to simulate estimation of attitude
% from gyroscope and accelerometer measurements read from 
% a file pre-recorded with an IMU.
%
% SYNTAX: [PAd,xAm,KGd] = att2sim(FileNum,Q,R,P0);
%
function [PAd,xAm,KGd] = att2sim(FileNum,Q,R,P0);
DataRefNo = num2str(FileNum);
% Kalman Filter Application to IMUs
% clear all;
% DataRefNo = '129';
[label,tstmp,Stillness,GyroXYZ,AcceleroXYZ,IMUquat,MagnetoXYZ] = readRecordingFile(['data',DataRefNo,'.txt']);
N = length(tstmp); % Number of Samples
SR = N/tstmp(end); % SR = Sampling Rate
dt = 1/SR; % Sampling time
% Normaliztion of the accelerations
for i=1:1:N
AcceleroNORM(i,:) = AcceleroXYZ(i,:)./norm(AcceleroXYZ(i,:));
end
[PAd,xAm,KGd] = att2loop(GyroXYZ, AcceleroNORM,dt,N,tstmp,Q,R,P0);
end %-end of att2sim
%%% MATLAB CODE 11.01 +++++++++++++++++++++++++++++++++++++++

