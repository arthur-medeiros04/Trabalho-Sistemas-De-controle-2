%% TAREFA 1 - Questão 5: Controle de Posicao (Malha Externa)
clear; clc; close all;

%% 1. Parametros do Sistema (Obtidos na Questao 3)
K_v = 1.0656; 
tau_v = 6.4751;
polo_planta = 1 / tau_v;

% FT da Velocidade
s = tf('s');
G_v_s = K_v / (tau_v * s + 1);

% FT da Perturbacao (Velocidade)
K_tc = -6.3248;
tau_tc = 6.3248;
G_tc_s = K_tc / (tau_tc * s + 1);

%% 2. Planta de Posicao (A sua deducao correta!)
% X(s) / V(s) = (2/s) * G_v_s
G_p_s = (2 / s) * G_v_s;

%% 3. Requisitos de Projeto (Lugar das Raizes no dominio S)
% Requisito 1: ts <= 1.5 * tau_d
ts_max = 1.5 * tau_v; % 9.7126 s
ts_alvo = 9.0; % Vamos focar em 9.0s para ter margem de seguranca

% Requisito 2: Overshoot < 5% -> zeta >= 0.69
zeta = 0.707; % Escolha padrao que resulta em ~4.3% de overshoot

% Calculo do polo dominante desejado (sd)
% ts = 3 / (zeta * wn) -> sigma = 3 / ts
sigma_d = 3 / ts_alvo;
wn = sigma_d / zeta;
wd = wn * sqrt(1 - zeta^2);

sd = -sigma_d + 1i * wd;

fprintf('--- Parametros do Projeto (Controle de Posicao) ---\n');
fprintf('Tempo de acomodacao alvo: %.2f s (Max: %.2f s)\n', ts_alvo, ts_max);
fprintf('Polo Desejado no Plano S (sd): %.4f + %.4fi\n', real(sd), imag(sd));


%% 4. Projeto do Controlador PID (C(s) = K * (s+z1)*(s+z2) / s)
% Precisamos do integrador (1/s) para rejeitar a perturbacao de carga Tc.

% 4.1 Escolha do Primeiro Zero (z1)
% Tática: Cancelar o polo lento do motor para simplificar o LGR
z1 = polo_planta; 
fprintf('\nZero 1 (Cancelamento): %.4f\n', z1);

% 4.2 Condicao de Fase para achar o Segundo Zero (z2)
% A malha aberta parcial (sem z2 e sem K) é:
L_aux = G_p_s * (s + z1) / s;

% A soma das fases em sd deve ser 180 graus (pi rad)
% angle(sd + z2) + angle(L_aux(sd)) = pi
fase_L_aux = angle(evalfr(L_aux, sd));
fase_z2_req = pi - fase_L_aux;

% Trigonometria Corrigida: 
% Para o zero estar no semiplano esquerdo (estavel), a subtracao do real(sd) e invertida.
z2 = imag(sd) / tan(fase_z2_req) - real(sd);

fprintf('Zero 2 (Condicao de Fase): %.4f\n', z2);

% 4.3 Condicao de Modulo para achar o Ganho K
L_aux2 = L_aux * (s + z2);
K = 1 / abs(evalfr(L_aux2, sd));

fprintf('Ganho K (Condicao de Modulo): %.4f\n\n', K);

%% 5. Montagem Final e Analise
C_s = K * (s + z1) * (s + z2) / s;
L_s = C_s * G_p_s; % Malha aberta completa

figure('Name', 'Controle de Posicao (PID) - LGR', 'Position', [50 50 1200 800]);

% 5.1 Lugar das Raizes
subplot(2,2,1);
rlocus(G_p_s * (s+z1)*(s+z2)/s); % Locus variando K
hold on;
plot(real(sd), imag(sd), 'rd', 'MarkerSize', 8, 'LineWidth', 2);
title('Lugar das Raizes (Malha de Posicao)');
axis([-1 0.5 -1 1]);

% 5.2 Resposta ao Degrau (Referencia de Posicao x = 1m)
subplot(2,2,2);
H_mf_ref = feedback(C_s * G_p_s, 1);

% --- CORRECAO: Filtro de Referencia para o Zero de Malha Fechada ---
% O zero z2 do PID aumenta drasticamente o overshoot. 
% Adicionamos um filtro de referencia F(s) = z2 / (s + z2) para anular
% este efeito e fazer o sistema comportar-se como um 2a ordem puro.
F_pref = tf(z2, [1 z2]);
H_mf_ref_filtrada = F_pref * H_mf_ref;

% AUMENTADO O TEMPO DE SIMULAÇÃO PARA 50 SEGUNDOS
t_sim = 0:0.1:50; 
step(H_mf_ref, t_sim); % Plota sem o filtro (overshoot exagerado)
hold on;
step(H_mf_ref_filtrada, t_sim); % Plota com o filtro (overshoot < 5% corrigido)
title('Seguimento de Referencia (Posicao Desejada = 1m)');
ylabel('Posicao Linear x [m]');
legend('Sem Filtro (Pico de ~20%)', 'Com Pre-Filtro (Overshoot < 5%)', 'Location', 'southeast');
grid on;

% 5.3 Rejeicao de Perturbacao (Degrau de Tc = 1 Nm)
% O Tc afeta a velocidade, que depois é integrada para virar posicao.
% Caminho do disturbio: Tc -> G_tc_s -> (2/s) -> Saida X
% Nota: A perturbacao entra DEPOIS do controlador, logo o filtro de 
% referencia Nao afeta a rejeicao de perturbacao!
G_dist_to_X = G_tc_s * (2/s);
H_mf_dist = G_dist_to_X / (1 + C_s * G_p_s);

subplot(2,2,3);
step(H_mf_dist, t_sim);
title('Rejeicao de Perturbacao na Posicao (Tc = 1 Nm)');
ylabel('Variacao na Posicao \Deltax [m]');
grid on;

% 5.4 Resposta em Frequencia
subplot(2,2,4);
margin(L_s);
title('Margens de Estabilidade (Bode)');
grid on;