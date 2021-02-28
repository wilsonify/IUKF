%%% MATLAB CODE 01.01 +++++++++++++++++++++++++++++++++++++++
rng(12345,'v5normal');	% Resets the Random Number
                                                % Generator to ver5. Normal, seed = 12345;
N = 10000; % Total number of signal samples
x = randn(N,1);
% NOTE: M' yields the conjugate transpose of matrix M. 
% If M is real, M' just transposes it
n = linspace(0,(N-1),N)';
figure; plot(n,x,'k'); grid;
xlabel(' Discrete Time Index t');
ylabel('x(t)');
% For the Normalized Histogram
% [N x binwidth]/NormFactor = total_area/F = 1.0 
% -> NormFactor = N x binwidth
binwidth = 0.1;
EDGES = [-3.0:binwidth:3];
NormFactor = N * binwidth ;
figure; bar(EDGES,((histc(x,EDGES))/NormFactor),0.95,'k');grid;
xlabel('x');
ylabel('Normalized histogram of x');
%%% MATLAB CODE 01.01 +++++++++++++++++++++++++++++++++++++++