% Testar modelo din�mico em uma trajet�ria em tempo real

close all
clear
clc

try
    fclose(instrfindall);
end
% Rotina para buscar pasta raiz
addpath(genpath(pwd))

A = ArDrone;

tmax = 60; % Tempo Simula��o em segundos
X = zeros(1,19); % Dados correntes da simula��o

figure(1)
axis([-3 3 -3 3 0 3])
grid on
A.mCADplot
drawnow
pause(2)
disp('Start............')

% =========================================================================
t = tic;
tc = tic;
tp = tic;

XX = [];
TT = [];
while toc(t) < tmax
    if toc(tc) > 1/30
        TT = [TT toc(tc)];
        
        tc = tic;
        
        w = 0.025;
        % Trajet�ria desejada
        tt = toc(t);
        A.pPos.Xd(1) = 1*sin(2*pi*w*tt);            % x
        A.pPos.Xd(7) = 1*2*pi*w*cos(2*pi*w*tt);     % dx
        A.pPos.Xd(2) = 1*sin(2*pi*2*w*tt);          % y
        A.pPos.Xd(8) = 1*2*pi*2*w*cos(2*pi*2*w*tt); % dy
        A.pPos.Xd(3) = 1;                           % z       
                        
        % Controlador
        A.rGetSensorData
        A = cUnderActuatedController(A);        
        A.rSendControlSignals;
        
        XX = [XX [A.pPos.Xd; A.pPos.X; tt]];
        
    end
    if toc(tp) > 0.3
        tp = tic;
        A.mCADplot;
        drawnow
    end
    
end

figure
subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
legend('\phi_{Des}','\phi_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
legend('\theta_{Des}','\theta_{Atu}')
grid

figure
plot(XX([1,13],:)',XX([2,14],:)')
% axis([-1.5 1.5 -1.5 1.5])
axis equal

figure
subplot(211),plot(XX(end,:),XX([1 13],:)')
legend('x_{Des}','x_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([2 14],:)')
legend('y_{Des}','y_{Atu}')
grid

% figure
% subplot(211),plot(XX(end,:),XX(19,:)')
% legend('x_{Des}','x_{Atu}')
% grid
% subplot(212),plot(XX(end,:),[XX(13,:); [0 diff(XX(13,:))]*30])
% legend('y_{Des}','y_{Atu}')
% grid

