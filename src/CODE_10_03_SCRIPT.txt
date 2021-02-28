%%%% MATLAB CODE 10.03 +++++++++++++++++++++++++++++++++++++
% Plot waterfall from height mean and variance
%
esth = XAVECT(1,:);
sdh = sqrt( PAVECT(1,:) );
szeh = length(esth);
n = linspace(0, (szeh-1), szeh );
% Plot the height estimates as waterfall plot:
hmin = 55;
hmax = 65;
hnumgauss = ((hmax-hmin) * 10) + 1;
hstep = (hmax-hmin)/(hnumgauss-1) ;
WATFALL = zeros(hnumgauss,szeh);
for t = 1:szeh
[valsx,resgauss] = calcgauss(hmin,hnumgauss,hmax, esth(t),sdh(t));
WATFALL(:,t) = resgauss;
end
% Create a mesh grid for waterfall contour plots:
[TIME,HEIGHT] = meshgrid(1:1:szeh, hmin:hstep:hmax);
% WATERFALL PLOT (following matlab instructions for “column-oriented data analysis')
figure; waterfall(TIME',HEIGHT',WATFALL'); colormap('winter'); colorbar;
xlabel('Kalman Iterations');
ylabel('Height in meters')
figure; contour3(TIME',HEIGHT',WATFALL', 50);view(2)
colormap('winter'); colorbar;
xlabel('Kalman Iterations');
ylabel('Height in meters')
%%%% MATLAB CODE 10.03 ++++++++++++++++++++++++++++++++++++++
