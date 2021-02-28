%%% MATLAB CODE 02.03 +++++++++++++++++++++++++++++++++++++++
% MULTIPLICATION OF 2 BIVARIATE GAUSSIAN DISTRIBUTIONS
MU0 = [1;1];
SIG0 = [1, 0.25; 0.25 , 1];
MU1 = [6;6];
SIG1 = [0.25, -0.025; -0.025, 0.25];
%%
%% Visualizing the first 2-D Gaussian: MU0, SIG0
[X01, X02, P0] = msh2dg(-2, 0.2, 8,MU0,SIG0);
%% Visualizing the second 2-D Gaussian: MU1 , SIG1
[X11, X12, P1] = msh2dg(-2, 0.2, 8, MU1, SIG1);
%
%% USING FORMULAS FROM Ahrendt, Bromiley (“FORMULAS A”)
% Covariance matrix of the product is:
SIGPA = inv( (inv(SIG0)) + (inv(SIG1)) )
% Mean Vector of the Product Is:
MUPA =SIGPA * (((inv(SIG0)) * MU0)+((inv(SIG1)) * MU1))
%
% % NOW USING THE FORMULAS WITH K (“FORMULAS B”)
%% The K matrix is:
K = SIG0 * ( inv( SIG0 + SIG1) )
% Covariance matrix of the product is:
SIGPB = SIG0 - ( K * SIG0)
% Mean Vector of the Product Is:
MUPB = MU0 + ( K * (MU1 - MU0))
%%
%% WE CONFIRM MUPA = MUPB and SIGPA = SIGPB
%%
%% Visualizing the product:
PP = P0.* P1; % Calculates the numerical point'to'point
% products of the distributions
figure;mesh(X11,X12,PP);
figure; contour(X11,X12,PP);grid
xlabel('x1');ylabel('x2');
%%% MATLAB CODE 02.03 +++++++++++++++++++++++++++++++++++++++
