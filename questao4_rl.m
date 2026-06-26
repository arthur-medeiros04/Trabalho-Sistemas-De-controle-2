%% TAREFA 1 - Questão 4: Controle de Velocidade por Lugar das Raizes (Analitico)
clear; clc; close all;

%% 1. Parametros do Sistema (Obtidos na Questao 3)
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
a = -den_z(2);     % Polo discreto da planta

G_tc_z = c2d(G_tc_s, Ts, 'zoh');

%% 3. Requisitos de Projeto (Polo Desejado)
% Requisito: Tempo de acomodacao ts <= 4.85s (vamos usar 4.5s)
% Requisito: Sem pico (vamos usar zeta = 0.9, overshoot = 0.15% -> invisivel)
ts_mf = 4.5;
zeta = 0.9;
wn = 4 / (zeta * ts_mf); % Frequencia natural

% Polo dominante no Plano S
sd = -zeta * wn + 1i * wn * sqrt(1 - zeta^2);

% Mapeamento para o Plano Z: z = e^(s*Ts)
pd = exp(sd * Ts); 
fprintf('--- Parametros do Projeto ---\n');
fprintf('Polo Desejado no Plano Z (pd): %.4f + %.4fi\n', real(pd), imag(pd));


%% 4. Projeto do Controlador (Metodo Analitico do LGR)
% O controlador PI tem a forma: C(z) = K * (z - zc) / (z - 1)
% Onde (z - 1) é o integrador obrigatorio para erro nulo.

% 4.1 CONDIÇÃO DE FASE para encontrar o Zero (zc)
% A soma das fases em malha aberta deve ser um multiplo impar de 180 (pi rad)
% angle(pd - zc) - angle(pd - a) - angle(pd - 1) = pi
% angle(pd - zc) = pi + angle(pd - a) + angle(pd - 1)

fase_polo_planta = angle(pd - a);
fase_integrador  = angle(pd - 1);
fase_zero_req    = pi + fase_polo_planta + fase_integrador;

% Usando trigonometria simples (tan(theta) = Cateto Oposto / Cateto Adjacente)
% tan(fase_zero_req) = imag(pd) / (real(pd) - zc)
zc = real(pd) - (imag(pd) / tan(fase_zero_req));

fprintf('\n--- Condição de Fase ---\n');
fprintf('Zero calculado (zc): %.4f\n', zc);

% 4.2 CONDIÇÃO DE MÓDULO para encontrar o Ganho (K)
% A magnitude de K * G(z) * C_aux(z) avaliada no polo pd deve ser igual a 1
% | K * b * (pd - zc) / ((pd - a) * (pd - 1)) | = 1

% K = |pd - a| * |pd - 1| / ( b * |pd - zc| )
K = (abs(pd - a) * abs(pd - 1)) / (b * abs(pd - zc));

fprintf('\n--- Condição de Módulo ---\n');
fprintf('Ganho calculado (K): %.4f\n\n', K);


%% 5. Montagem do Controlador e Analise
C_aux = tf([1 -zc], [1 -1], Ts);
C_z = K * C_aux;
L_z = C_aux * G_v_z; % Malha aberta sem K para o plot do rlocus

figure('Name', 'Projeto Analitico - Lugar das Raizes', 'Position', [50 50 1200 800]);

% 5.1 Lugar das Raizes
subplot(2,2,1);
rlocus(L_z);
hold on;
% Marcando o polo desejado exato no gráfico
plot(real(pd), imag(pd), 'rd', 'MarkerSize', 8, 'LineWidth', 2); 
title('Lugar das Raizes com Polo Desejado (Diamante)');
axis([0.8 1.05 -0.1 0.1]); % Zoom na região de interesse

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