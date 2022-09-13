% Testar modelo dinâmico em uma trajetória em tempo real

close all
clear
clc

try
    fclose(instrfindall);
end
% Rotina para buscar pasta raiz
addpath(genpath(pwd))

A = ArDrone;

tmax = 60; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação

disp('Start............')

% =========================================================================
t = 0;
tc = A.pPar.Ts;
ts = tic;

XX = [];
TT = [];
while t < tmax
    w = 0.025;
    % Trajetória desejada
    tt = t;
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
    
    t = t + tc;
end
Ts = toc(ts);
disp([num2str(Ts) 's para simular ' num2str(tmax) 's'])
disp('Fim ..........')

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

