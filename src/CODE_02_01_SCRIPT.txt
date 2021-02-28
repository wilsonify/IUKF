%%% MATLAB CODE 02.01 +++++++++++++++++++++++++++++++++++++++
% Creation of 2-time series with given means, variances
%and correlation 
% Resetting RNG to ver5 Normal, seed = 12345.
% MEANS at mu and COVARIANCE MTX Sigma
% mu = [1,2]; Sigma = [1 .5; .5 2];
rng(12345,'v5normal');
mu = [1,2];Sigma = [1 .5; .5 2];
R=chol(Sigma);
Nsamp =100000; %Create Nsamp samples, each of the series
dat = repmat(mu,Nsamp,1) + randn(Nsamp,2) * R; 
% The time series x1 & x2 are columns 1 & 2 of dat
% First Figure: Show PRISMS of 2-D histogram in
% perspective view
binside = 0.1;
figure;
x1edges = [-2.0:binside:6.0];
x2edges = [-2.0:binside:6.0];
edges{1} = x1edges;
edges{2} = x2edges;
hist3(dat,'Edges',edges);%DRAW hist. ,perspective view
N = hist3(dat, 'Edges', edges ); %ON SAME DATA, get
%histogram values in matrix N for later use
xlabel(' x1 '); ylabel(' x2 ');
% Display default 3D perspective view—NOT NORMALIZED:
view(3);
% NORMALIZATION and other forms of visualization
NormFactor = binside * binside * Nsamp;
N1 = N'/NormFactor;
% Second Figure: 3D contour levels in perspective 
figure;
[n1r, n1c] = size(N1);
x1edges = [-2.0:0.1:6.0]; %These assignments are REPEATED
% here for clarity
x2edges = [-2.0:0.1:6.0]; %These assignments are REPEATED	% here for clarity
X1edgesm = repmat(x1edges, n1c, 1);
X2edgesm = repmat(x2edges',1,n1r);
contour3(X1edgesm,X2edgesm,N1,30);
colormap('winter');
grid on; xlabel('x1'); ylabel('x2'); view(3); colorbar;
% Third Figure; show the TOP VIEW (2D) of the contour
% plots for Normalized 2D Histogram
figure;
contour3(X1edgesm,X2edgesm,N1,30);
colormap('winter');
grid on; xlabel('x1'); ylabel('x2'); view(2); colorbar;
% Fourth figure 2D DENSITY PLOT
figure;
scatter( dat(:,1) , dat(:,2), 1 ) ;
grid on; xlabel(' x1 '); ylabel(' x2 '); view(2);
axis([ -2, 6 , -2 , 6])
%%% MATLAB CODE 02.01 +++++++++++++++++++++++++++++++++++++++