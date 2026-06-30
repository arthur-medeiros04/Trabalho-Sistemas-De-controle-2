%% TAREFA 1 - Questão 6: CRV com Ação Feed-Forward de Torque (AFFT)
clear; clc; close all;

%% 1. Parametros do Sistema
Ts = 0.1;
K_v = 1.0656; tau_v = 6.4751;
K_tc = -6.3248; tau_tc = 6.3248;

% Plantas Continuas
G_v_s = tf(K_v, [tau_v 1]);
G_tc_s = tf(K_tc, [tau_tc 1]);

% Plantas Discretas
G_v_z = c2d(G_v_s, Ts, 'zoh');
G_tc_z = c2d(G_tc_s, Ts, 'zoh');

[num_z, den_z] = tfdata(G_v_z, 'v');
b = num_z(2);
a = -den_z(2);

%% 2. Reconstruindo o Projeto do CRV (Alocacao de Polos - Item 4)
ts_alvo = 4.5;
polo_s = -3 / (ts_alvo); % Ajuste de 1a ordem
pd = exp(polo_s * Ts);

% Coeficientes do Controlador PI Discreto: C(z) = (q0*z + q1) / (z - 1)
% z^2 + (b*q0 - 1 - a)z + (a + b*q1) = z^2 - 2*pd*z + pd^2
q0 = (1 + a - 2*pd) / b;
q1 = (pd^2 - a) / b;
C_crv = tf([q0 q1], [1 -1], Ts);

%% 3. Projeto do Feed-Forward de Torque (AFFT)
% O controlador Feed-Forward ideal cancela o disturbio: C_ff = - G_tc / G_v
% Como o MATLAB lida perfeitamente com a divisao de FTs, vamos usar a 
% relacao exata discreta:
C_ff = minreal(-G_tc_z / G_v_z);

fprintf('--- Controlador Feed-Forward Projetado ---\n');
disp(C_ff);
% Vai notar que é praticamente um ganho constante de ~5.935 !

%% 4. Analise de Rejeicao de Perturbacao (Comparacao)
% Vamos injetar um degrau de perturbacao Tc = 1 Nm.

% 4.1 Apenas CRV (Solucao do Item 4)
% H_dist1 = G_tc / (1 + C_crv * G_v)
H_dist_CRV = G_tc_z / (1 + C_crv * G_v_z);

% 4.2 CRV + AFFT (Solucao do Item 6)
% Com o FF somado na saida do controlador, a nova FT de perturbacao é:
% H_dist2 = (G_tc + G_v * C_ff) / (1 + C_crv * G_v)
H_dist_AFFT = minreal((G_tc_z + G_v_z * C_ff) / (1 + C_crv * G_v_z));

% 5. Plotagem dos Resultados
figure('Name', 'Analise Feed-Forward (AFFT)', 'Position', [100 100 1000 500]);

t_sim = 0:Ts:30; % Simulando por 30 segundos
[y_crv, t] = step(H_dist_CRV, t_sim);
[y_afft, ~] = step(H_dist_AFFT, t_sim);

subplot(1,2,1);
plot(t, y_crv, 'r', 'LineWidth', 2); hold on;
plot(t, y_afft, 'b', 'LineWidth', 2);
title('Rejeicao de Perturbacao (Tc = 1 Nm)');
xlabel('Tempo [s]'); ylabel('Queda na Velocidade \Delta\omega [rad/s]');
legend('Apenas CRV (Reativo)', 'CRV + AFFT (Proativo)', 'Location', 'southeast');
grid on;
ylim([-6 1]);

subplot(1,2,2);
% Mostrando o Polos e Zeros do CRV+AFFT
pzmap(H_dist_AFFT);
title('Diagrama Polo-Zero (Malha CRV + AFFT)');
grid on;

fprintf('Conclusao Item 6: A malha CRV+AFFT suprime o disturbio quase que perfeitamente.\n');