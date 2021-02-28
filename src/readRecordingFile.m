%%% MATLAB CODE 11.03 +++++++++++++++++++++++++++++++++++++++
% readRecordingFile – Function to open and read a
% text data file, written with the data recorded
% by the IMU, during its operation.
function [label,t,Stillness,GyroXYZ,AcceleroXYZ,IMUquat,MagnetoXYZ] = readRecordingFile(FILENAME)
label=FILENAME;
fileID = fopen(FILENAME);
readCell=textscan(fileID,'%f %f %f %f %f %f %f %f %f %f %f %f %f %f %f','delimiter',',');
fclose(fileID);

t = readCell{1};
Stillness = readCell{2};
GyroXYZ = [readCell{3},readCell{4},readCell{5}];
AcceleroXYZ = [readCell{6},readCell{7},readCell{8}];
IMUquat = [readCell{9},readCell{10},readCell{11},readCell{12}];
MagnetoXYZ = [readCell{13},readCell{14},readCell{15}];
end	% end of readRecordingFile
%%% MATLAB CODE 11.03 +++++++++++++++++++++++++++++++++++++++
