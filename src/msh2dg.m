%%% MATLAB CODE 02.02 +++++++++++++++++++++++++++++++++++++++
% msh2dg-creates & meshes a 2D Gaussian dist. x1, x2
%
% SYNTAX [X1, X2, P] = msh2dg(str, stp, nd, MU, SIGMA);
% MU is the vector of MEANS (e.g.,  [1;2])
% SIGMA is the COVARIANCE MATRIX (e.g., [1,0.5;0.5,2])
% (Covariance matrix is explained in Section 2.3)
% str(start),stp(step),nd (end) in MESHGRID(both x1 ,x2)
% Values used for example figure: -2, 0.2, 6
function [X1, X2, P] = msh2dg(str, stp, nd, MU, SIGMA);
var1 = SIGMA(1,1);
var2 = SIGMA(2,2);
sig1 = sqrt(var1);
sig2 = sqrt(var2);
ro = SIGMA(1,2)/(sig1 * sig2);
mu1 = MU(1);
mu2 = MU(2);
[X1,X2] = meshgrid(str:stp:nd, str:stp:nd);
Z1 = (1/var1) * ( (X1-mu1).^2);
Z2 = (2 * ro/(sig1 * sig2)) *( (X1-mu1) .* (X2-mu2) );
Z3 = (1/var2) * ( (X2-mu2).^2);
Z = Z1 - Z2 + Z3;
PFACTOR = 1/( sqrt(1 - (ro ^2)) * 2 * pi * sig1 * sig2 );
edenom = (-2) * (1 - (ro^2));
P = PFACTOR * (exp( Z ./ edenom));
% MESH PLOT (standard)
figure; mesh(X1, X2, P);;xlabel('x1');ylabel('x2'); zlabel('P');
% SIMPLE (TOP VIEW) CONTOUR PLOT
figure; contour(X1, X2, P); grid on
xlabel('x1'); ylabel('x2'); zlabel('P');
% 3-D CONTOUR PLOT
figure; contour3(X1, X2, P); grid on
c = 0.8;
surface(X1,X2,P,'EdgeColor',[c,c,c],'FaceColor','none')
xlabel('x1'); ylabel('x2'); zlabel('P');
end % end of function msh2gd.m
%%% MATLAB CODE 02.02 +++++++++++++++++++++++++++++++++++++++