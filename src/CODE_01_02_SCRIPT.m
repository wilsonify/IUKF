%%% MATLAB CODE 01.02 +++++++++++++++++++++++++++++++++++++++
rng(12345,'v5normal');	% Resets the Random Number
%Generator to ver5. Normal, seed = 12345;
% Generate x
x = randn(10000,1);
n = linspace(0,9999,10000)';
% Obtain y by the transformation y = 2 x + 1.5
y = 2 * x + 1.5;
% Plot first 100 points of both time series (same
%graphical scale)
figure;
subplot(2,1,1);
plot(n(1:100),x(1:100),'k');
axis([0,99,-9, 9]);
grid;
xlabel('Discrete time index t');
ylabel('x(t)')
subplot(2,1,2)
plot(n(1:100),y(1:100),'k');
axis([0,99,-9, 9]);
grid;
xlabel('Discrete time index t');
ylabel('y(t)')
% NORMALIZED HISTOGRAMS
N = 10000;
EDGES = [-9.0:0.1:9];
binwidth = 0.1;
NormFactor = N * binwidth ;
figure;
subplot(2,1,1);
bar(EDGES,((histc(x,EDGES))/NormFactor),1.02,'k');grid;
axis([-9,9,0, 0.5]);
xlabel('x');
ylabel('Normalized Histogram of x')
subplot(2,1,2);
bar(EDGES,((histc(y,EDGES))/NormFactor),1.02,'k');grid;
axis([-9,9,0, 0.5]);
xlabel('y');
ylabel('Normalized Histogram of y')
%%% MATLAB CODE 01.02 +++++++++++++++++++++++++++++++++++++++