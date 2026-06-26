<<<<<<< HEAD
%% TAREFA 1: Analise Estatica Completa do Sistema DC
clear; clc; close all;

%% Secao 1: Parametros do Sistema
Km = 1/3;  % Constante de conversao eletromecanica [V/(rad/s)]
L  = 0.5;  % Indutancia da armadura [H]
R  = 10;   % Resistencia da armadura [Ohms]
B  = 0.1;  % Coeficiente de atrito viscoso [Nms/rad]
J  = 1;    % Momento de inercia do rotor [kg*m^2]
%% Parâmetros Questão 2 (simulação não linear no Simulink)
V_manipulada = 20; % [Volts] variavel manipulada
Tc_perturbacao = 1; % [N.m] torque  de perturbação do sistema 

%% Secao 2: Equacoes Estaticas e Nao-Lineares
% Torque Mecanico (Tm)
calc_Tm = @(i) (5.26 * (1 - exp(-i))) .* (i >= 0) + (-5.26 * (1 - exp(i))) .* (i < 0);

% Velocidade (w) isolada da eq. mecanica: w = (Tm - Tc)/B
calc_w_est = @(Tm, Tc) (Tm - Tc) / B;

% Tensao (V) isolada da eq. eletrica: V = R*i + Km*w
calc_V_est = @(i, w) R * i + Km * w;

%% Secao 3: Geracao dos Vetores Base
% Vamos usar a corrente como base (vetor independente) para evitar equacoes 
% transcendentais na hora de plotar, variando dentro do limite de [-3, 3] A
i_vec = linspace(-3, 3, 500);
Tm_vec = calc_Tm(i_vec);


%% Secao 4: FIGURA 1 - Focada na Variavel de Processo (Velocidade w)
figure('Name', 'Analise de Velocidade', 'Position', [50, 400, 900, 400]);

% --- Grafico 1: w x V (Tc constante) ---
Tc_fixos = [-2, -1, 0, 1, 2]; 
subplot(1, 2, 1); hold on;
for k = 1:length(Tc_fixos)
    w_vec = calc_w_est(Tm_vec, Tc_fixos(k));
    V_vec = calc_V_est(i_vec, w_vec);
    plot(V_vec, w_vec, 'LineWidth', 2, 'DisplayName', ['Tc = ', num2str(Tc_fixos(k)), ' Nm']);
end
title('Velocidade (\omega) x Tensao (V)');
xlabel('Tensao V [Volts]'); ylabel('Velocidade \omega [rad/s]');
xlim([-40 40]); ylim([-30 30]); grid on; legend('Location', 'northwest');

% --- Grafico 2: w x Tc (V constante) ---
V_fixos = [-20, 0, 20]; 
subplot(1, 2, 2); hold on;
for k = 1:length(V_fixos)
    Tc_calc = Tm_vec - (B/Km)*(V_fixos(k) - R * i_vec);
    w_calc = (V_fixos(k) - R * i_vec) / Km;
    plot(Tc_calc, w_calc, 'LineWidth', 2, 'DisplayName', ['V = ', num2str(V_fixos(k)), ' V']);
end
title('Velocidade (\omega) x Perturbacao (Tc)');
xlabel('Torque de Carga Tc [Nm]'); ylabel('Velocidade \omega [rad/s]');
xlim([-2 2]); ylim([-30 30]); grid on; legend('Location', 'northeast');


%% Secao 5: FIGURA 2 - Focada no Esforco do Sistema (Corrente i)
figure('Name', 'Analise de Corrente', 'Position', [1000, 400, 900, 400]);

% --- Grafico 3: i x Tc (V constante) ---
% Sua ideia! Vamos plotar Tc no eixo X (causa) e corrente no eixo Y (consequencia)
subplot(1, 2, 1); hold on;
for k = 1:length(V_fixos)
    % Calculamos qual Tc seria necessario para gerar a combinacao (i, V_fixa)
    Tc_calc = Tm_vec - (B/Km)*(V_fixos(k) - R * i_vec);
    plot(Tc_calc, i_vec, 'LineWidth', 2, 'DisplayName', ['V = ', num2str(V_fixos(k)), ' V']);
end
title('Demanda de Corrente: i x Tc');
xlabel('Perturbacao (Torque de Carga Tc) [Nm]'); 
ylabel('Corrente i [A]');
xlim([-2 2]); ylim([-3 3]); grid on; legend('Location', 'northwest');

% --- Grafico 4: i x V (Tc constante) ---
subplot(1, 2, 2); hold on;
for k = 1:length(Tc_fixos)
    w_vec = calc_w_est(Tm_vec, Tc_fixos(k));
    V_vec = calc_V_est(i_vec, w_vec);
    plot(V_vec, i_vec, 'LineWidth', 2, 'DisplayName', ['Tc = ', num2str(Tc_fixos(k)), ' Nm']);
end
title('Esforco Eletrico: i x V');
xlabel('Tensao V [Volts]'); 
ylabel('Corrente i [A]');
xlim([-40 40]); ylim([-3 3]); grid on; legend('Location', 'northwest');
=======
%% TAREFA 1: Analise e Resolucao de Sistema DC
% Limpeza inicial do ambiente
clear; clc; close all;

%% Secao 1: Parametros do Sistema
% Definicao dos parametros fisicos do motor DC baseados no relatorio
% Os valores serao automaticamente salvos no workspace ao rodar o script.

Km = 1/3;  % Constante de conversao eletromecanica [V/(rad/s) ou Nm/A]
L  = 0.5;  % Indutancia da armadura [H]
R  = 10;   % Resistencia da armadura [Ohms]
B  = 0.1;  % Coeficiente de atrito viscoso [Nms/rad]
J  = 1;    % Momento de inercia do rotor [kg*m^2]

disp('Parametros carregados no workspace com sucesso.');

%% Secao 2: Equacoes Dinamicas e Estaticas
% 2.1 EDOs do Sistema (Comentadas para referencia no Simulink)
% Dinamica Eletrica: di/dt = (V - R*i - Km*w) / L
% Dinamica Mecanica: dw/dt = (Tm - Tc - B*w) / J
% Cinematica:        dth/dt = w

% 2.2 Equacao Nao-Linear do Torque Mecanico (Tm)
% O torque mecanico Tm se relaciona com a corrente atraves de uma relacao nao linear.
% Usamos uma funcao anonima vetorizada lidando com valores positivos e negativos de i.
calc_Tm = @(i) (5.26 * (1 - exp(-i))) .* (i >= 0) + (-5.26 * (1 - exp(i))) .* (i < 0);

% 2.3 Equacoes Estaticas (Regime Permanente: derivadas = 0)
% Da equacao mecanica isolamos a velocidade (w):
calc_w_est = @(Tm, Tc) (Tm - Tc) / B;

% Da equacao eletrica isolamos a tensao (V):
calc_V_est = @(i, w) R * i + Km * w;


%% Secao 3: Vetores de Limites e Plotagem das Curvas Estaticas
% Faixa de valores de operacao estipulada:
% V in [-40, 40] V, i in [-3, 3] A, w in [-30, 30] rad/s, Tc in [-2, 2] Nm

% Criando um vetor de corrente linearmente espacado que respeita os limites [-3, 3]
i_vec = linspace(-3, 3, 200);

% Calculando o Torque Mecanico associado a todo o vetor de corrente
Tm_vec = calc_Tm(i_vec);

% --- Grafico 1: Velocidade vs Tensao (Para valores fixos de Perturbacao Tc) ---
Tc_fixos = [-2, 0, 2]; % Avaliando nos limites e no centro da faixa de operacao

figure('Name', 'Caracteristicas Estaticas', 'Position', [100, 100, 900, 400]);

subplot(1, 2, 1);
hold on;
for k = 1:length(Tc_fixos)
    Tc_atual = Tc_fixos(k);
    w_vec = calc_w_est(Tm_vec, Tc_atual);
    V_vec = calc_V_est(i_vec, w_vec);

    % Plotando apenas os pontos que respeitam os limites de velocidade e tensao
    plot(V_vec, w_vec, 'LineWidth', 2, 'DisplayName', ['Tc = ', num2str(Tc_atual), ' Nm']);
end
title('Velocidade (\omega) x Tensao (V)');
xlabel('Tensao Manipulada V [Volts]');
ylabel('Velocidade \omega [rad/s]');
xlim([-40 40]);
ylim([-30 30]);
grid on;
legend('Location', 'northwest');
hold off;

% --- Grafico 2: Velocidade vs Torque de Carga (Para valores fixos de Tensao V) ---
V_fixos = [-20, 0, 20]; 

subplot(1, 2, 2);
hold on;
for k = 1:length(V_fixos)
    V_atual = V_fixos(k);

    % Isolando algebraicamente Tc em funcao de i e V (no estatico):
    % w = (Tm - Tc)/B e V = R*i + Km*w -> Tc = Tm - (B/Km)*(V - R*i)
    Tc_calc = Tm_vec - (B/Km)*(V_atual - R * i_vec);

    % Calculando o w correspondente para essa Tensao e vetor de corrente
    w_calc = (V_atual - R * i_vec) / Km;

    plot(Tc_calc, w_calc, 'LineWidth', 2, 'DisplayName', ['V = ', num2str(V_atual), ' V']);
end
title('Velocidade (\omega) x Perturbacao (Tc)');
xlabel('Torque de Carga Tc [Nm]');
ylabel('Velocidade \omega [rad/s]');
xlim([-2 2]); % Focando na faixa de operacao de Tc
ylim([-30 30]);
grid on;
legend('Location', 'northeast');
hold off;
>>>>>>> a87fa606661a82320c9e04007f995cc4568e6762
