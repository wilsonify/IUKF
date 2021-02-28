% Function graphimures - displays 4 columns of
% the ascii output file RTATT2IMUOUT.txt
% versus the first column(time), for comparison
%
% SYNTAX: DATA = graphimures( filename , csela , cselb, cselc, cseld);
% The trace for csela is black solid line; cselb is black dash-dot
% The trace for cselc is gray solid line; cseld is gray dash-dot
%
function DATA = graphimures( filename , csela , cselb, cselc, cseld);

DATA = load(filename,'-ascii');
t = DATA(:,1);
CA = DATA(:,csela);
CB = DATA(:,cselb);
CC = DATA(:,cselc);
CD = DATA(:,cseld);
lgnds = ['T-i-m-e-';'Phi--Acc';'ThetaAcc';'Psi--Acc';'Phi---KF';'Theta-KF';'Psi---KF'];
gray6 = [0.6  0.6  0.6];

figure; 
plot(t,CA,'k','Linewidth',1.5);grid on;
hold on;
plot(t,CB,'k-.','Linewidth',1.5);
plot(t,CC,'Color',gray6,'Linewidth',1.5);
plot(t,CD,'Color',gray6,'Linestyle','-.','Linewidth',1.5);
hold off
 legend(lgnds(csela,:),lgnds(cselb,:),lgnds(cselc,:),lgnds(cseld,:),'Location','Southeast');grid on;
ylabel('degrees'); xlabel('time in seconds')

end    % end of function graphimures