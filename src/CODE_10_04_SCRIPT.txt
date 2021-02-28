%%%% MATLAB CODE 10.04 ++++++++++++++++++++++++++++++++++++++
%% ZOOM INTO THE KF HEIGHT GAUSSIAN EVOLUTION FOR THE
%FIRST 100 ITERATIONS
TIME100 = TIME(:, 1:100);
HEIGHT100 = HEIGHT( :, 1:100);
WATFALL100 = WATFALL(:, 1:100);
figure; waterfall(TIME100',HEIGHT100',WATFALL100'); colormap('winter'); colorbar;
xlabel('Kalman Iterations');
ylabel('Height in meters')
%%%% MATLAB CODE 10.04 ++++++++++++++++++++++++++++++++++++++