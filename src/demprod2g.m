%%% MATLAB CODE 07.01 +++++++++++++++++++++++++++++++++++++++
% function demprod2g.m demonstrates the product of 2
% univariate gaussians
%
% FOR BOTH UNIVARIATE GAUSSIANS
% Receives the start and end x values to calculate
% Receives the number of evaluations to do in the range
% Receives the MEAN and VARIANCE of the first Gaussian
% Receives the MEAN and VARIANCE of the second Gaussian
% Plots both original gaussians (top pane)
% Plots the (point-to-point) product of the gaussians
% (bottom pane)
%
% SYNTAX:
% [prodx,prodgauss]= demprod2g(stx, numfx, endx, mu1,
%  var1, mu2,var2);
%
function [prodx,prodgauss]= demprod2g(stx,numfx,endx, mu1,var1,mu2,var2);
sigm1 = sqrt(var1);
sigm2 = sqrt(var2);
[valsx,vgauss1] =calcgauss(stx,numfx,endx,mu1, sigm1);
[valsx,vgauss2] =calcgauss(stx,numfx,endx, mu2,sigm2);
prodx = valsx;
prodgauss = vgauss1 .* vgauss2;
% Plotting
figure;
subplot(2,1,1); plot(prodx,vgauss1,'b--');
hold on
plot(prodx,vgauss2,'r');
hold off
title('Original Individual Gaussians');
grid
set(gca,'XMinorTick','on')
grid minor
set(gca,'Xtick',stx:1:endx)
grid off
grid on
subplot(2,1,2); plot(prodx,prodgauss,'b');
title('Resulting Product Gaussian');
grid
set(gca,'XMinorTick','on')
grid minor
set(gca,'Xtick',stx:1:endx)
grid off
grid on
end
%%% MATLAB CODE 07.01 +++++++++++++++++++++++++++++++++++++++