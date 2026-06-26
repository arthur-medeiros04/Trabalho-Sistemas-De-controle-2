%% TAREFA 1: Analise Estatica Completa do Sistema DC
clear; clc; close all;

%% Secao 1: Parametros do Sistema
Km = 1/3;  % Constante de conversao eletromecanica [V/(rad/s)]
L  = 0.5;  % Indutancia da armadura [H]
R  = 10;   % Resistencia da armadura [Ohms]
B  = 0.1;  % Coeficiente de atrito viscoso [Nms/rad]
J  = 1;    % Momento de inercia do rotor [kg*m^2]

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
Tc_fixos = [-2, 0, 2]; 
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