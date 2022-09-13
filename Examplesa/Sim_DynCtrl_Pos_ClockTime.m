% Testar modelo dinâmico em tempo real

close all
clear
clc

try
    fclose(instrfindall);
end
addpath(genpath(pwd))

A = ArDrone;

disp('Início..........')

% =========================================================================
% Iniciar eta de controle
% Decolar
tmax = 40; % Tempo Simulação em segundos
X = zeros(1,19); % Dados correntes da simulação

t = 0;
tc = A.pPar.Ts;
ts = tic;

XX = [];

while t < tmax
    if t > 3*tmax/4
        A.pPos.Xd(1) = 0;
        A.pPos.Xd(2) = 0;
        A.pPos.Xd(3) = 1;
        A.pPos.Xd(6) = 0;
    elseif t > 2*tmax/4
        A.pPos.Xd(1) = 0;
        A.pPos.Xd(2) = 0;
        A.pPos.Xd(3) = 1;
        A.pPos.Xd(6) = 0;
    elseif t > tmax/4
        A.pPos.Xd(1) = 1;
        A.pPos.Xd(2) = 0;
        A.pPos.Xd(3) = 1;
        A.pPos.Xd(6) = 0;
    else
        A.pPos.Xd(1) = 0;
        A.pPos.Xd(2) = 0;
        A.pPos.Xd(3) = 1;
        A.pPos.Xd(6) = 0;
    end
    
    % Controlador
    A.rGetSensorData
    A = cUnderActuatedController(A);
    A.rSendControlSignals;
    
    XX = [XX [A.pPos.Xd; A.pPos.X; A.pSC.Ud; t]];
    
    t = t + tc;
end
Ts = toc(ts);
disp([num2str(Ts) 's para simular ' num2str(tmax) 's'])
disp('Fim ..........')

% Exibição dos resultados
figure
subplot(211),plot(XX(end,:),XX([4 16],:)'*180/pi)
legend('\phi_{Des}','\phi_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([5 17],:)'*180/pi)
legend('\theta_{Des}','\theta_{Atu}')
grid

figure
subplot(211),plot(XX(end,:),XX([3 15],:)')
legend('z_{Des}','z_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([6 18],:)'*180/pi)
legend('\psi_{Des}','\psi_{Atu}')
grid

figure
subplot(211),plot(XX(end,:),XX(25,:))
legend('\phi_{Des}')
grid
subplot(212),plot(XX(end,:),XX(26,:))
legend('\theta_{Des}')
grid

figure
subplot(211),plot(XX(end,:),XX([1 13],:)')
legend('x_{Des}','x_{Atu}')
grid
subplot(212),plot(XX(end,:),XX([2 14],:)')
legend('y_{Des}','y_{Atu}')
grid

figure
subplot(311),plot(XX(end,:),XX([7 19],:)')
legend('dx_{Des}','dx_{Atu}')
grid
subplot(312),plot(XX(end,:),XX([8 20],:)')
legend('dy_{Des}','dy_{Atu}')
grid
subplot(313),plot(XX(end,:),XX([9 21],:)')
axis([0 tmax -1 1])
legend('dz_{Des}','dz_{Atu}')
grid


