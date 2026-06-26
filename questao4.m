%% TAREFA 1 - Questão 4: Controle por Lugar das Raizes (Cancelamento)
clear; clc; close all;

%% 1. Parametros do Sistema
K_v = 1.0656; 
tau_v = 6.4751;
G_v_s = tf(K_v, [tau_v 1]);

K_tc = -6.3248;
tau_tc = 6.3248;
G_tc_s = tf(K_tc, [tau_tc 1]);

%% 2. Discretizacao do Sistema (Ts = 0.1s)
Ts = 0.1;
G_v_z = c2d(G_v_s, Ts, 'zoh');
[num_z, den_z] = tfdata(G_v_z, 'v');

b = num_z(2);      % Ganho discreto da planta
a = -den_z(2);     % Polo discreto da planta (Lento)

G_tc_z = c2d(G_tc_s, Ts, 'zoh');

%% 3. Requisitos de Projeto (Polo Desejado de 1a Ordem)
% Requisito: Tempo de acomodacao ts <= 4.5s
% Como usaremos cancelamento, o sistema em malha fechada será de 1a ordem pura.
% O tempo de acomodação (5%) de um sistema de 1a ordem é ts = 3 * tau
ts_mf = 4.5;
tau_mf = ts_mf / 3;

% Polo continuo (sd) e discreto (pd) desejado
sd = -1 / tau_mf;
pd = exp(sd * Ts); 

fprintf('--- Parametros do Projeto (Cancelamento) ---\n');
fprintf('Polo Desejado no Plano Z (pd): %.4f\n', pd);

%% 4. Projeto do Controlador (Cancelamento Polo-Zero)
% Controlador PI: C(z) = K * (z - zc) / (z - 1)

% 4.1 Escolha do Zero (zc) -> Cancelar a dinamica lenta do motor
zc = a; 
fprintf('\n--- Cancelamento ---\n');
fprintf('Zero do controlador colocado sobre o polo da planta (zc = a): %.4f\n', zc);

% 4.2 Calculo do Ganho (K)
% Com o polo 'a' cancelado pelo zero 'zc', a equacao caracteristica fica:
% 1 + K * [b / (z - 1)] = 0  =>  z - 1 + K*b = 0  => z = 1 - K*b
% Igualando ao nosso polo desejado: pd = 1 - K*b
K = (1 - pd) / b;

fprintf('\n--- Ganho Calculado ---\n');
fprintf('Ganho (K): %.4f\n\n', K);

%% 5. Montagem do Controlador e Analise
C_aux = tf([1 -zc], [1 -1], Ts);
C_z = K * C_aux;
L_z = C_aux * G_v_z; % Malha aberta (note que o polo a. e zero zc. se cancelam aqui)

figure('Name', 'Projeto Cancelamento - Lugar das Raizes', 'Position', [100 100 1200 800]);

% 5.1 Lugar das Raizes
subplot(2,2,1);
rlocus(L_z);
hold on;
plot(real(pd), imag(pd), 'rd', 'MarkerSize', 8, 'LineWidth', 2); 
title('Lugar das Raizes (Polo Cancelado)');
axis([0.8 1.05 -0.1 0.1]); 

% 5.2 Resposta ao Degrau (Referencia)
subplot(2,2,2);
H_mf = feedback(C_z * G_v_z, 1);
t_sim = 0:Ts:20;
step(H_mf, t_sim);
title('Seguimento de Referencia (\omega_{ref} = 1 rad/s)');
ylabel('Velocidade \omega [rad/s]');
grid on;

% 5.3 Resposta a Rejeicao de Perturbacao (Tc = 1 Nm)
subplot(2,2,3);
H_dist = G_tc_z / (1 + G_v_z * C_z);
step(H_dist, t_sim);
title('Rejeicao de Perturbacao (Tc = 1 Nm)');
ylabel('Variacao \Delta\omega');
grid on;

% 5.4 Resposta em Frequencia (Bode)
subplot(2,2,4);
margin(C_z * G_v_z);
title('Resposta em Frequencia (Bode)');
grid on;