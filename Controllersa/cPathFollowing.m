function [Drone,Path] = cPathFollowing(Drone,Path)


% Constantes do Controlador de Caminhos
Khr1 = [0.2 0 0; 0 0.2  0; 0 0 0.20];
Khr2 = [0.75 0 0; 0 0.75 0; 0 0 0.75];
% Xd = zeros(12,1);

% Navegar para o primeiro ponto do caminho
% Verificar se o erro entre o caminho e a posi��o atual do rob� � inferior
% a um limiar
if norm(Drone.pPos.X(1:3)-Path.X(1:3,Path.Pos)) < 0.05
    Path.Pos = Path.Pos + 1;    
    if Path.Pos > Path.Size
        Path.Pos = 1;
    end
end

% C�lculo do ponto mais pr�ximo ao Rota em coordenadas polares
rho   = norm(Path.X(1:3,Path.Pos)-Drone.pPos.X(1:3));
alpha = atan2(Path.dX(3,Path.Pos),norm(Path.dX(1:2,Path.Pos)));
beta  = atan2(Path.dX(2,Path.Pos),Path.dX(1,Path.Pos));

% C�lculo da velocidade do rob� sobre o Rota
% Velocidade Desejada muda de acordo com o erro de dist�ncia
V = Path.Vmax/(1+2*rho);

Path.dXr = [V*cos(alpha)*cos(beta); V*cos(alpha)*sin(beta); V*sin(alpha)];

Xtil = Path.X(1:3,Path.Pos) - Drone.pPos.X(1:3)

% Modelo Cinem�tico
KM = [cos(Drone.pPos.X(6)) -sin(Drone.pPos.X(6)) 0; sin(Drone.pPos.X(6)) cos(Drone.pPos.X(6)) 0; 0 0 1];
uSC = KM\(Path.dXr + Khr1*tanh(Khr2*Xtil([1 2 3])));

Drone.pPos.Xd(1:3) = Path.X(1:3,Path.Pos);
Drone.pPos.Xd(7:9) = Path.dXr + Khr1*tanh(Khr2*Xtil([1 2 3]));

% % Alterar a velocidade sobre o caminho em fun��o da proximidade tangencial
% 
% 
% % % C�lculo do ponto mais pr�ximo ao Rota em coordenadas polares
% % rho   = norm(Path.Xc(1:3)-Drone.pPos.X(1:3));
% % alpha = atan2(Path.Xc(9),norm(Path.Xc(7:8)));
% % beta  = atan2(Path.Xc(8),Path.Xc(7));
% %
% % % C�lculo da velocidade do rob� sobre o Rota
% % % Velocidade Desejada muda de acordo com o erro de dist�ncia
% % V = Path.Vmax/(1+2*rho);
% %
% % Xtil = Path.Xc - Drone.pPos.X
% %
% % Path.Xr([7 8 9 12]) = [V*cos(alpha)*cos(beta); V*cos(alpha)*sin(beta); V*sin(alpha); Xtil(12)];
% %
% % % Modelo Cinem�tico
% % KM = [cos(Drone.pPos.X(6)) -sin(Drone.pPos.X(6)) 0 0; sin(Drone.pPos.X(6)) cos(Drone.pPos.X(6)) 0 0; 0 0 1 0; 0 0 0 1];
% % uSC = KM\(Path.Xr([7 8 9 12],1) + Khr1*tanh(Khr2*Xtil([1 2 3 6])));
% %
% % % C�lculo do sinal de controle para alterar a velocidade desejada
% % % Atribuindo posi��o desejada
% % Xd([1 2 3 6])  = Path.Xc([1 2 3 6]);
% % Xd([7 8 9 12]) = KM*uSC;
% 
% %Drone.pPos.Xd = Path.Xc;


