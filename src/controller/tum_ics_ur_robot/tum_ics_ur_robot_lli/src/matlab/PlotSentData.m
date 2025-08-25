function PlotSentData( xi, xf )
%PLOTSENTDATA Summary of this function goes here
%   Detailed explanation goes here

MULT_jointstate = 1/1e8;

figure(1)
% subplot(6,1,1);
plot(xf(:,4),xf(:,5),'b -');
hold on
grid on
plot(xi(:,4),MULT_jointstate*xi(:,5),'r -');



end

