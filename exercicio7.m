%% Criação do filtro para o preditor:

% 1. Definir os parâmetros
tau_f = 0.15; % Tau do filtro -> 0.45 freq de corte
Ts = 0.1; % tempo de amostragem escolhido no ex 6

% 2. Criar o filtro contínuo G(s) = 1 / (tau_f * s + 1)
Gf_s = tf(1, [tau_f, 1]); 

% 3. Converter para discreto G(z) usando o método Tustin (Bilinear)
Gf_z = c2d(Gf_s, Ts, 'tustin')