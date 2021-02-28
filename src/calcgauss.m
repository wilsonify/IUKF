%%% MATLAB CODE 07.02 +++++++++++++++++++++++++++++++++++++++
% calcgauss.m
% evaluates a normal pdf from the theoretical formula,
% providing start x, number of x, end x
% and mean and std.
% RETURNS the vector of values calculated (for plot)
%
% SYNTAX: [valsx,resgauss] = calcgauss( startx, numofx,
%endx, mu, sigm);
%
%
function [valsx,resgauss] =calcgauss(startx,numofx, endx,mu,sigm);
%
% Vector of evaluation values (abcisas)
gapx = (endx - startx)/(numofx - 1);
valsx = zeros(numofx, 1);
for i = 1:numofx
valsx(i,1) = startx + (i * gapx);
end
coef = 1/(sqrt(2 * pi * sigm^2));
dnm = 2 * sigm^2;
resgauss = coef .* exp(((-1) * (valsx-mu).^2)./ dnm);
end
%%% MATLAB CODE 07.02 +++++++++++++++++++++++++++++++++++++++
