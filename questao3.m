%% TAREFA 1 - Questão 3: Identificacao de Sistemas (Resposta ao Degrau)
clear; clc; close all;

% Parametros do sistema
Km = 1/3; L = 0.5; R = 10; B = 0.1; J = 1;

% EDO do Sistema Não-Linear (usado para simular via ode45)
% Estado x = [corrente; velocidade]
% u = [Tensao V; Torque Tc]
% Usamos sign(x) e abs(x) para unificar a formula e evitar Inf*0 no ode45
sistema_nl = @(t, x, u) [
    (1/L) * (u(1) - R*x(1) - Km*x(2));
    (1/J) * ( (5.26 * sign(x(1)) * (1 - exp(-abs(x(1))))) - u(2) - B*x(2) )
    ];

%% EXPERIMENTO 1: Degrau em V (Mantendo Tc = 1)
disp('--- Experimento 1: Degrau em V ---');
V_0 = 20; Tc_0 = 1; Delta_V = 1;

% Simula 50s para estabilizar no ponto de operacao
[t1, x1] = ode45(@(t,x) sistema_nl(t, x, [V_0, Tc_0]), [0 50], [0; 0]);
w0_v = x1(end, 2); % Velocidade em equilibrio

% Aplica o degrau (V_0 + Delta_V) a partir dos 50s por mais 50s
[t2, x2] = ode45(@(t,x) sistema_nl(t, x, [V_0 + Delta_V, Tc_0]), [50 100], x1(end,:)');
w_final_v = x2(end, 2);

% Analise da Curva
dw_v = w_final_v - w0_v;
Kv = dw_v / Delta_V; % Ganho

w_63_v = w0_v + 0.632 * dw_v; % Valor de 63.2%
% Encontra o tempo onde a velocidade cruza w_63_v
idx_v = find(x2(:,2) >= w_63_v, 1); 
tau_v = t2(idx_v) - 50; % Desconta o tempo de inicio do degrau

fprintf('Velocidade Inicial (w0): %.3f rad/s\n', w0_v);
fprintf('Ganho (Kv): %.3f (rad/s)/V\n', Kv);
fprintf('Constante de Tempo (tau_v): %.3f s\n', tau_v);
fprintf('Funcao de Transferencia (w/V): %f / (%.3f*s + 1)\n\n', Kv, tau_v);


%% EXPERIMENTO 2: Degrau em Tc (Mantendo V = 20)
disp('--- Experimento 2: Degrau em Tc ---');
Delta_Tc = 0.1; % Degrau de 0.1 Nm

% Simula estabilizacao (reaproveitando o x1 do exp 1)
w0_tc = w0_v; 

% Aplica o degrau (Tc_0 + Delta_Tc)
[t3, x3] = ode45(@(t,x) sistema_nl(t, x, [V_0, Tc_0 + Delta_Tc]), [50 100], x1(end,:)');
w_final_tc = x3(end, 2);

% Analise da Curva
dw_tc = w_final_tc - w0_tc;
Ktc = dw_tc / Delta_Tc; % Ganho (Sera negativo!)

w_63_tc = w0_tc + 0.632 * dw_tc;
% Encontra o tempo onde a velocidade cruza w_63_tc (como esta caindo, usamos <=)
idx_tc = find(x3(:,2) <= w_63_tc, 1); 
tau_tc = t3(idx_tc) - 50;

fprintf('Velocidade Inicial (w0): %.3f rad/s\n', w0_tc);
fprintf('Ganho (Ktc): %.3f (rad/s)/Nm\n', Ktc);
fprintf('Constante de Tempo (tau_tc): %.3f s\n', tau_tc);
fprintf('Funcao de Transferencia (w/Tc): %f / (%.3f*s + 1)\n', Ktc, tau_tc);

%% Plot para ilustracao
figure('Name', 'Identificacao por Degrau', 'Position', [100 100 800 400]);
subplot(1,2,1);
plot([t1; t2], [x1(:,2); x2(:,2)], 'b', 'LineWidth', 2);
hold on; yline(w_63_v, 'r--', '63.2%'); xline(50+tau_v, 'k:');
title('Resposta ao Degrau de +1V'); xlabel('Tempo (s)'); ylabel('\omega (rad/s)'); grid on;

subplot(1,2,2);
plot([t1; t3], [x1(:,2); x3(:,2)], 'b', 'LineWidth', 2);
hold on; yline(w_63_tc, 'r--', '63.2%'); xline(50+tau_tc, 'k:');
title('Resposta ao Degrau de +0.1 Nm (Tc)'); xlabel('Tempo (s)'); ylabel('\omega (rad/s)'); grid on;