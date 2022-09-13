%%%% Time-delayed Control %%%%

% Universidade Federal de Viçosa - UFV
% Centro de Ciências Exatas e Tecnológicas - CCE
% DEPARTAMENTO DE ENGENHARIA ELÉTRICA - DEL

% Monografia: Análise do atraso de comunicação no controle de quadrotores

% Orientador: Prof. Dr. Alexandre Santos Brandão
% Discente: Leonardo Alves Fagundes Júnior - 86308

%% Script Clock Time Version

% Tarefa: Positioning
% Análise no Espaço de Parâmetros do Controlador!

% Controle Não-Linear
% Paper: High-Level Underactuated Nonlinear Control for Rotorcraft Machines)
%        (Brandão A. S., 2013)

clc, clear, close all

try
    fclose(instrfindall);
end

%% Rotina para buscar pasta raiz - Look for root directory

addpath(genpath(pwd))

%% Cria uma representação para o objeto ArDrone

A = ArDrone; % Quadrotor Atrasado

%% Definindo a Figura que irá rodar a simulação

f1 = figure('Name','Análise no espaço dos Parâmetros','NumberTitle','off');
f1.Position = [-1 2 930 682];
% f1.Position = [1367 11 831 634]; % Segunda tela em Sete Lagoas!
figure(f1);

% title('Task: Position Control')
ax = gca;
ax.FontSize = 12;
xlabel({'$$\textbf{K}_{p}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$\textbf{K}_{d}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
zlabel({'$$t_{delay}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
axis([-0.25 1.25 -0.25 1.25 0.0 1.25])
view(3)
view(50,10)
grid('on')
hold('on')

f2 = figure('Name','Análise no espaço dos Parâmetros: Especificações','NumberTitle','off');
f2.Position = [435 2 930 682];
% f1.Position = [1367 11 831 634]; % Segunda tela em Sete Lagoas!
figure(f2);
ax = gca;
ax.FontSize = 12;
xlabel({'$$\textbf{K}_{p}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$\textbf{K}_{d}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
zlabel({'$$t_{delay}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
axis([-0.25 1.25 -0.25 1.25 0.0 1.25])
view(3)
view(50,10)
grid('on')
hold('on')


pause(1)
disp('Start..........')

% =========================================================================
pause(0.5)
disp('Clock Time version')

%% Início da Análise:

% Cria pasta da simulação
mkdir(pwd,'TrajectoryTrack_Circ');


% Tabela para armazenar parâmetros aceitáveis:
parametersData = [];
parametersDataGeral = [];
limiteErroPoints = [];
limiteIAEPoints = [];
limiteITAEPoints = [];
limiteNormErroPoints = [];
timesMATLAB = zeros(4,1);

% MAPA de possibilidades
MAPA = [];
for idAtraso = 0:1:63          % Amostras atrasadas (de 0 a 1,92 segundos de atraso)
    for Kp = 0:0.75/31:0.75       % Ganho Proporcional
        for Kd = 0:1/31:1      % Ganho Derivativo
            MAPA = vertcat(MAPA,[idAtraso Kp Kd]);
        end
    end
end

size(MAPA)

save MAPA MAPA

load MAPA
Aleatorio = randperm(length(MAPA));
tLacoMed = 0;

tSim = tic();

index = 1;

for ii = 1:length(MAPA)
    tLacoIni = tic;
    clear XX
    
    idAtraso = MAPA(Aleatorio(ii),1);
    Kd       = MAPA(Aleatorio(ii),3);
    Kp       = MAPA(Aleatorio(ii),2);
    
    % ----------------------------
    % Controlador
    K1 = Kd*diag([1 1 1]);     % Ganho derivativo (Kd)
    K2 = 1; %sqrt(4*K1);           % diag([1 1 1]);
    K3 = Kp*diag([1 1 1]);     % Ganho Proporcional (Kp)
    K4 = 1; %sqrt(4*K1*K2)/K3;
    
    %%
    
    A.iControlVariables; % Posição Inicial
    
    % Métricas
    erro_instavel = 5;
    IAE_naoAtrasado = 10;
    ITAE_naoAtrasado = 60;
    
    
    limiteErro     = true;
    limiteIAE      = true;
    limiteITAE     = true;
    limiteNormErro = true;
    
    %% Inicialização das variáveis
    
    tmax = 60;       % Tempo Simulação em segundos
    
    % Tempo de simulação:
    t = 0;           % Simula tempo real
    tc = A.pPar.Ts;  % Time Sampling do ArDrone (timer para controle)
    ts = tic;        % Tempo levado para simular
    tp = 1;          % Tempo para Plotar (simbólico, não tem significado físico!)
    
    % Inicialização dos Índices de Desempenho (drone atrasado):
    IAE = 0;
    ITAE = 0;
    IASC = 0;
    
    % Parâmetros para integração numérica:
    dt = 30e-3;
    
    ddX(1:3,1) = 0;
    dX(1:3,1) = 0;
    X(1:3,1) = A.pPos.X(1:3,1);
    
    % Ponto Desejado (x_d):
    % Variables Initialization
    % Referência 8 no plano
    rx = 1.5;
    ry = 1.5;
    rz = 0.5;
    w  = 2*pi*2/tmax; % 2 voltas
    
    % Dados robôs:
    A.pPos.Xtil = A.pPos.Xd - A.pPos.X;
    erro_inicial = A.pPos.Xtil;
    XX = [A.pPos.Xd' A.pPos.X' A.pSC.Ud' A.pSC.U' A.pPos.Xtil' idAtraso IAE ITAE IASC t];
    kk = 1;
    
    % =========================================================================
    acc_max = 0.85; % Tarefa de posicionamento
    acc_des = acc_max;
    %% Rotina de execução da tarefa
    
    % =========================================================================
    % Iniciar eta de controle
    % Decolar
    
    % Xc = A.pPos.X; % Implementação 1    
    while t < tmax
        % Inicio da realimentação:
        
        % circunferência:
        A.pPos.Xd(1)  = a*cos(w*t);    % posição x
        A.pPos.Xd(2)  = b*sin(w*t);    % posição y
        A.pPos.Xd(3)  = 1.5;           % posição z
        
        A.pPos.Xd(7)  = -a*w*sin(w*t); % velocidade em x
        A.pPos.Xd(8)  = b*w*cos(w*t);  % velocidade em y
        A.pPos.Xd(9)  = 0;
        
        
        
        % -- Implementação 2 (drone envia informações atuais ao controlador
        %                     até kk atingir o idAtraso, então enviará
        %                     dados atrasados ao controle):
        if kk > idAtraso
            Xc = XX(kk-idAtraso,13:24)';
        else
            Xc = A.pPos.X;
        end
        % --
        
        
        % Estratégia (Lei de Controle)
        A.pPos.Xtil = A.pPos.Xd - Xc;
        pos_til = A.pPos.Xtil(1:3,1);
        vel_til = A.pPos.Xtil(7:9,1);
        
        ddX = acc_des + K1*tanh(K2*vel_til) + K3*tanh(K4*pos_til);
        
        % ----------------------------
        % ArDrone
        A.pSC.Ud = [-ddX(2); -ddX(1); ddX(3); 0];
                
        % Índices de desempenho (Atrasado & Não-atrasado)
        IAE = IAE + norm(A.pPos.Xtil(1:3))*dt;
        ITAE = ITAE + norm(A.pPos.Xtil(1:3))*t*dt;
        IASC = IASC + norm(A.pSC.Ud)*dt;

        % Pegando os dados do robo:
        A.rSendControlSignals;
        A.rGetSensorData;
        
        % norm(A.pPos.Xd(1:3) - A.pPos.X(1:3))
        % =========================================================================
        % Métricas de Análise (Critérios de Parada): -> Melhorar a
        % análise das mátricas e restrições!!!
        % 1 - Limite para o Erro:
        if norm(A.pPos.Xd(1:3) - A.pPos.X(1:3)) > erro_instavel % Inválido para esta condição
            limiteErro = false;
            %t = tmax;
        end % erro_intavel = 5 -> Limite para o overshoot, que consideramos instável para essa tarefa!
        
        % 2 - Limite para o IAE:
        if IAE > 30*IAE_naoAtrasado % 15*IAE_naoAtrasado
            limiteIAE = false;
            %t = tmax;
        end % IAE_naoAtrasado = 5.3218 -> Valor obtido em simulação!
        % Deseja-se que o erro de regime transitório não seja muito grande.
        
        % 3 - Limite para o ITAE:
        if ITAE > 50*ITAE_naoAtrasado % 10*ITAE_naoAtrasado
            limiteITAE = false;
            %t = tmax;
        end % ITAE_naoAtrasado = 7.3256 -> Valor obtido em simulação!
        % Deseja-se que o erro de regime permanente não seja
        % muito grande. Que o drone não oscile muito e tenda a convergir.
        
        % 4 - Limite para o Erro de Posição:
        if norm(A.pPos.Xd(1:3) - A.pPos.X(1:3)) > norm(2*erro_inicial(1:3))
            limiteNormErro = false;
            %t = tmax;
        end % Sempre que o erro for maior que o erro inicial!
        % Critério de parada baseado na norma do erro.
        
        % =========================================================================
        
        % Histórico de dados:
        XX = [XX; [A.pPos.Xd' A.pPos.X' [-ddX(2); -ddX(1); ddX(3); 0]' A.pSC.U' (A.pPos.Xd - A.pPos.X)' idAtraso IAE ITAE IASC t]];
        kk = kk + 1;

        %------------------------------------------------------------
        t = t + tc;
        
    end

    if limiteErro && limiteIAE && limiteITAE && limiteNormErro % Condições satisfeitas para este caso,
        parametersData = [parametersData; idAtraso Kp Kd];
        disp(['Kp = ',num2str(Kp),' e Kd = ',num2str(Kd) ' e Tatr = ' num2str(idAtraso*30e-3)])
        
    else
        disp(['não plotou Kp = ',num2str(Kp),' e Kd = ',num2str(Kd), 'atraso = ',num2str(idAtraso)])
    end
    
    %-------- Rodar novamente para verificar a performance no espaço dos parâmetros
    % axis([-0.05 2.05 -0.05 2.05 0.05 2.05])
    if limiteErro && limiteIAE && limiteITAE && limiteNormErro % Condições satisfeitas para este caso,
        parametersDataGeral = [parametersDataGeral; idAtraso Kp Kd IAE ITAE]; % Usado para determinar a melhor combinação Kp e Kd em uma análise posterior
    end
    
    % Plota esferas das especificações:
    if limiteErro % Condições satisfeitas para este caso, adicione a esfera no gráfico de análise de parâmetros
        limiteErroPoints = [limiteErroPoints; idAtraso Kp Kd];

    end
    if limiteIAE % Condições satisfeitas para este caso, adicione a esfera no gráfico de análise de parâmetros
        limiteIAEPoints = [limiteIAEPoints; idAtraso Kp Kd];
        
    end
    
    if limiteITAE % Condições satisfeitas para este caso, adicione a esfera no gráfico de análise de parâmetros
        limiteITAEPoints = [limiteITAEPoints; idAtraso Kp Kd];

    end
    if limiteNormErro % Condições satisfeitas para este caso, adicione a esfera no gráfico de análise de parâmetros
        limiteNormErroPoints = [limiteNormErroPoints; idAtraso Kp Kd];

    end
    
    tLacoFim = toc(tLacoIni);
    tLacoMed = (tLacoMed*(ii-1) + tLacoFim)/ii;
    disp([num2str(ii) ' de ' num2str(length(MAPA)) ' :: tLaço = ' num2str(tLacoFim) ' :: tLaçoMed = ' num2str(tLacoMed)] )
    fprintf('\n');
    

    tS = toc(tSim);

    if index == 4096-1
        timesMATLAB(1) = tS;
    elseif index == 16384-1 
        timesMATLAB(2) = tS;
    elseif index == 36864-1
        timesMATLAB(3) = tS;
    elseif index == 65536-1
        timesMATLAB(4) = tS;
    end    
end

tSim = toc(tSim);

disp('-----------**-----------' )
fprintf('\nEnd...............\n')
disp(['tSimu = ' num2str(tSim) '[sec]'] )


cd(strcat(pwd,'\TrajectoryTrack_Circ'))

elements = {'64x64', '128x128', '192x192', '256x256'};

for idx = 1:4
    disp(['Sims: ' elements{idx} ' | Time: ' num2str(timesMATLAB(idx)) ' [sec]'])
end


%% --- 

% Espaço dos Parâmetors (Volume)
figure(f1), plot3(parametersData(:,2),parametersData(:,3),parametersData(:,1)*30e-3,'o','Color','w','MarkerSize',10,'MarkerEdgeColor','w','MarkerFaceColor','k')
ha1 = get(gca,'Children');                             % this returns the children of the axes

% Espaço dos Parâmetors (Restrições)
figure(f2), pC1 = plot3(limiteErroPoints(:,2),limiteErroPoints(:,3),limiteErroPoints(:,1)*30e-3,'o','Color',[0 0.4470 0.7410],'MarkerSize',10,'MarkerEdgeColor',[0 0.4470 0.7410],'MarkerFaceColor',[0 0.4470 0.7410]);
haC1 = get(gca,'Children');                             % this returns the children of the axes
hold on
figure(f2), pC2 = plot3(limiteIAEPoints(:,2),limiteIAEPoints(:,3),limiteIAEPoints(:,1)*30e-3,'o','Color',[0.4660 0.6740 0.1880],'MarkerSize',8,'MarkerEdgeColor',[0.4660 0.6740 0.1880],'MarkerFaceColor',[0.4660 0.6740 0.1880]);
haC2 = get(gca,'Children');                             % this returns the children of the axes
figure(f2), pC3 = plot3(limiteITAEPoints(:,2),limiteITAEPoints(:,3),limiteITAEPoints(:,1)*30e-3,'o','Color',[0.6350 0.0780 0.1840],'MarkerSize',6,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor',[0.6350 0.0780 0.1840]);
haC3 = get(gca,'Children');                             % this returns the children of the axes
figure(f2), pC4 = plot3(limiteNormErroPoints(:,2),limiteNormErroPoints(:,3),limiteNormErroPoints(:,1)*30e-3,'o','Color',[0.9290 0.6940 0.1250],'MarkerSize',4,'MarkerEdgeColor',[0.9290 0.6940 0.1250],'MarkerFaceColor',[0.9290 0.6940 0.1250]);
haC4 = get(gca,'Children');                             % this returns the children of the axes

drawnow

legend([pC1 pC2 pC3 pC4],{'$max(\tilde{\textbf{d}}) < 10~m$','$$IAE \leq 30~IAE_{sem-atraso}$$',...
    '$ITAE \leq 50~ITAE_{sem-atraso}$','$max(\Vert\textbf{d}\Vert)  \leq \Vert\textbf{d}(0)\Vert $'},...
    'FontSize',18,'interpreter','latex','Position',[0.6 0.77 0.37 0.19])

%% Data!
filename = 'DadosCirc';

% filename = 'DadosTrajCirc';
% filename = 'DadosTrajLemn';
save(filename, 'parametersData', 'parametersDataGeral', 'haC1', 'haC2', 'haC3', 'haC4', 'ha1','timesMATLAB');

name_figa = 'FigCircTraj_Volum';

% name_fig = 'FigTraj_VolumCirc';
% name_fig = 'FigTraj_VolumLemn';

name_fig = strcat(name_figa,'_Vol');
saveFigure(f1, name_fig);     % Salva a figura .fig
saveas(f1, name_fig, 'epsc'); % Salva a figura .eps
saveas(f1, name_fig, 'pdf');  % Salva a figura .pdf
saveas(f1, name_fig, 'png');  % Salva a figura .png

name_fig = strcat(name_figa,'_Conditions');
saveFigure(f2, name_fig);     % Salva a figura .fig
saveas(f2, name_fig, 'epsc'); % Salva a figura .eps
saveas(f2, name_fig, 'pdf');  % Salva a figura .pdf
saveas(f2, name_fig, 'png');  % Salva a figura .png

%%


% Depois da análise estar pronta, salvar as vistas (Kp x Kd; Kp x t_delay; e Kd x t_delay)
f3 = figure('Name','Análise no espaço dos Parâmetros: Especificações e Ganhos','NumberTitle','off');
f3.Position = [435 2 930 682];
figure(f3);


subplot(121)
p(1) = plot(limiteErroPoints(:,2),limiteErroPoints(:,3),'o','Color',[0 0.4470 0.7410],'MarkerSize',10,'MarkerEdgeColor',[0 0.4470 0.7410],'MarkerFaceColor',[0 0.4470 0.7410]); % plot the data again
hold on
p(2) = plot(limiteIAEPoints(:,2),limiteIAEPoints(:,3),'o','Color',[0.4660 0.6740 0.1880],'MarkerSize',8,'MarkerEdgeColor',[0.4660 0.6740 0.1880],'MarkerFaceColor',[0.4660 0.6740 0.1880]); % plot the data again
p(3) = plot(limiteITAEPoints(:,2),limiteITAEPoints(:,3),'o','Color',[0.6350 0.0780 0.1840],'MarkerSize',6,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor',[0.6350 0.0780 0.1840]); % plot the data again
p(4) = plot(limiteNormErroPoints(:,2),limiteNormErroPoints(:,3),'o','Color',[0.9290 0.6940 0.1250],'MarkerSize',4,'MarkerEdgeColor',[0.9290 0.6940 0.1250],'MarkerFaceColor',[0.9290 0.6940 0.1250]); % plot the data again
plot(0.5,0.85,'o','MarkerSize',12,'MarkerEdgeColor',[0.9290 0.6940 0.1250],'MarkerFaceColor','k');
ax = gca;
ax.FontSize = 12;
xlabel({'$$\textbf{K}_{p}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$\textbf{K}_{d}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
axis([-0.05 0.80 -0.05 1.05])
grid on

legend([p(1) p(2) p(3) p(4)],{'$max(\tilde{\textbf{d}}) < 5~m$','$$IAE \leq 30~IAE_{sem-atraso}$$',...
    '$ITAE \leq 50~ITAE_{sem-atraso}$','$max(\Vert\textbf{d}\Vert)  \leq \Vert\textbf{d}(0)\Vert $'},...
    'FontSize',18,'interpreter','latex','Position',[0.208659173220459,0.834516129032258,0.696717170865562,0.098035190615836],'NumColumns',2)


subplot(122)
hold on
plot(parametersData(:,2),parametersData(:,3),'o','Color','w','MarkerSize',10,'MarkerEdgeColor','w','MarkerFaceColor','k') % plot the data again
plot(0.5,0.85,'o','MarkerSize',12,'MarkerEdgeColor','w','MarkerFaceColor','g');
hold off
ax = gca;
ax.FontSize = 12;
xlabel({'$$\textbf{K}_{p}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$\textbf{K}_{d}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
axis([-0.05 0.80 -0.05 1.05])
grid on
box('on')


name_fig = strcat(name_figa,'_KpKd');
saveFigure(f3, name_fig);     % Salva a figura .fig
saveas(f3, name_fig, 'epsc'); % Salva a figura .eps
saveas(f3, name_fig, 'pdf');  % Salva a figura .pdf
saveas(f3, name_fig, 'png');  % Salva a figura .png


% Depois da análise estar pronta, salvar as vistas (Kp x Kd; Kp x t_delay; e Kd x t_delay)
f3 = figure('Name','Análise no espaço dos Parâmetros: Especificações e Ganhos','NumberTitle','off');
f3.Position = [435 2 930 682];
figure(f3);

subplot(121)
plot(limiteErroPoints(:,2),limiteErroPoints(:,1)*30e-3,'o','Color',[0 0.4470 0.7410],'MarkerSize',10,'MarkerEdgeColor',[0 0.4470 0.7410],'MarkerFaceColor',[0 0.4470 0.7410]) % plot the data again
hold on
plot(limiteIAEPoints(:,2),limiteIAEPoints(:,1)*30e-3,'o','Color',[0.4660 0.6740 0.1880],'MarkerSize',8,'MarkerEdgeColor',[0.4660 0.6740 0.1880],'MarkerFaceColor',[0.4660 0.6740 0.1880]) % plot the data again
plot(limiteITAEPoints(:,2),limiteITAEPoints(:,1)*30e-3,'o','Color',[0.6350 0.0780 0.1840],'MarkerSize',6,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor',[0.6350 0.0780 0.1840]) % plot the data again
plot(limiteNormErroPoints(:,2),limiteNormErroPoints(:,1)*30e-3,'o','Color',[0.9290 0.6940 0.1250],'MarkerSize',4,'MarkerEdgeColor',[0.9290 0.6940 0.1250],'MarkerFaceColor',[0.9290 0.6940 0.1250]) % plot the data again
ax = gca;
ax.FontSize = 12;
xlabel({'$$\textbf{K}_{p}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$t_{delay}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
axis([-0.05 0.80 -0.05 1.25])
grid on

subplot(122)
cG = get(ha1,'Zdata'); % obtain the ZData
plot(parametersData(:,2),parametersData(:,1)*30e-3,'o','Color','w','MarkerSize',10,'MarkerEdgeColor','w','MarkerFaceColor','k') % plot the data again
ax = gca;
ax.FontSize = 12;
xlabel({'$$\textbf{K}_{p}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$t_{delay}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
axis([-0.05 0.80 -0.05 1.25])
grid on
box('on')

name_fig = strcat(name_figa,'_KpTdelay');
saveFigure(f2, name_fig);     % Salva a figura .fig
saveas(f3, name_fig, 'epsc'); % Salva a figura .eps
saveas(f3, name_fig, 'pdf');  % Salva a figura .pdf
saveas(f3, name_fig, 'png');  % Salva a figura .png


% Depois da análise estar pronta, salvar as vistas (Kp x Kd; Kp x t_delay; e Kd x t_delay)
f3 = figure('Name','Análise no espaço dos Parâmetros: Especificações e Ganhos','NumberTitle','off');
f3.Position = [435 2 930 682];
figure(f3);

subplot(121)
plot(limiteErroPoints(:,3),limiteErroPoints(:,1)*30e-3,'o','Color',[0 0.4470 0.7410],'MarkerSize',10,'MarkerEdgeColor',[0 0.4470 0.7410],'MarkerFaceColor',[0 0.4470 0.7410]) % plot the data again
hold on
plot(limiteIAEPoints(:,3),limiteIAEPoints(:,1)*30e-3,'o','Color',[0.4660 0.6740 0.1880],'MarkerSize',8,'MarkerEdgeColor',[0.4660 0.6740 0.1880],'MarkerFaceColor',[0.4660 0.6740 0.1880]) % plot the data again
plot(limiteITAEPoints(:,3),limiteITAEPoints(:,1)*30e-3,'o','Color',[0.6350 0.0780 0.1840],'MarkerSize',6,'MarkerEdgeColor',[0.6350 0.0780 0.1840],'MarkerFaceColor',[0.6350 0.0780 0.1840]) % plot the data again
plot(limiteNormErroPoints(:,3),limiteNormErroPoints(:,1)*30e-3,'o','Color',[0.9290 0.6940 0.1250],'MarkerSize',4,'MarkerEdgeColor',[0.9290 0.6940 0.1250],'MarkerFaceColor',[0.9290 0.6940 0.1250]) % plot the data again
ax = gca;
ax.FontSize = 12;
xlabel({'$$\textbf{K}_{d}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$t_{delay}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
axis([-0.05 1.05 -0.05 1.25])
grid on
box('on')

subplot(122)
plot(parametersData(:,3),parametersData(:,1)*30e-3,'o','Color','w','MarkerSize',10,'MarkerEdgeColor','w','MarkerFaceColor','k') % plot the data again
ax = gca;
ax.FontSize = 12;
xlabel({'$$\textbf{K}_{d}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$t_{delay}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');
axis equal
axis([-0.05 1.05 -0.05 1.25])
grid on
box('on')

name_fig = strcat(name_figa,'_KdTdelay');
saveFigure(f3, name_fig);     % Salva a figura .fig
saveas(f3, name_fig, 'epsc'); % Salva a figura .eps
saveas(f3, name_fig, 'pdf');  % Salva a figura .pdf
saveas(f3, name_fig, 'png');  % Salva a figura .png



%% ----- Versão 2

pts = [parametersData(:,2) parametersData(:,3) parametersData(:,1)*30e-3];

ff = figure;
ff.Position = [-1 2 930 682];

rng('default')
% P = rand(30,3);
% plot3(pts(:,1),pts(:,2),pts(:,3),'.','MarkerSize',10)
grid on

k = boundary(pts);
j = boundary(pts,1);
hold on
Vol = trisurf(k,pts(:,1),pts(:,2),pts(:,3));%,'Facecolor','red','FaceAlpha',0.1);
Vol1 = trisurf(j,pts(:,1),pts(:,2),pts(:,3));%,'Facecolor','red','FaceAlpha',0.1); Melhor!!!
% Vol1.Position = [435 2 930 682];
Vol1.FaceAlpha = 0.9;
Vol1.EdgeAlpha = 0.0;

axis([0.05 0.80 -0.05 1.05 0.05 2])

view(3)
view(50,20)
grid('on')

xlabel({'$$\textbf{K}_{p}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
ylabel({'$$\textbf{K}_{d}$$'},'FontSize',18,'FontWeight','bold','interpreter','latex');
zlabel({'$$t_{delay}$$ [s]'},'FontSize',18,'FontWeight','bold','interpreter','latex');


delete(Vol)

plot3(0.5,0.85,0.315,'o','Color','g','MarkerSize',10,'MarkerEdgeColor','w','MarkerFaceColor','g') 


% ---

name_fig = strcat(name_figa,'_surfCurve');
saveFigure(ff, name_fig);     % Salva a figura .fig
saveas(ff, name_fig, 'epsc'); % Salva a figura .eps
saveas(ff, name_fig, 'pdf');  % Salva a figura .pdf
saveas(ff, name_fig, 'png');  % Salva a figura .png


disp('Fim ..........')


