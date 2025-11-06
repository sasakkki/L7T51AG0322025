function simular_reservatorios()
    % =========================================================================
    % 1. DEFINIÇÃO DOS PARÂMETROS DO SISTEMA
    % =========================================================================
    
    % Constantes físicas (Baseadas no E7 - Controle de Nível)
    rho = 1000;  % Densidade da água (kg/m^3)
    g = 9.78;    % Aceleração da gravidade (m/s^2)
    
    % Parâmetros dos reservatórios 


    A1 = 0.095 * 0.095; 
    A2 = 0.095 * 0.095; 
    A3 = 0.095 * 0.095; 
    A = [A1; A2; A3]; % Vetor de áreas
    % --- NOVO: Limite de Transbordamento ---
    h_max = 0.27; % Limite de altura para transbordamento (m)
    
    % Parâmetros de resistência 
    % Estes valores são calculados para serem consistentes com os dados

    h1eq = 0.22 ;             %1 é o primeiro reservatório
    h2eq = 0.138;
    h3eq = 0.053; 
    h1f = 0.015;
    h2f = 0.015;
    h3f = 0.015;
    ti = 0;
    tempf = 321; %s
    dt = tempf - ti;
    q = ((h1eq - h1f)*A1 + (h2eq - h2f)*A2 + (h3eq - h3f)*A3) / dt 
    R1 = (rho * g * (h1eq - h2eq)) / q^2
    R2 = (rho * g * (h2eq - h3eq)) / q^2
    R3 = (rho * g * h3eq) / q^2 
    R = [R1; R2; R3]; % Vetor de resistências
    
    % =========================================================================
    % 2. SIMULAÇÃO 1: RESPOSTA A VAZÃO CONSTANTE (REGIME PERMANENTE)
    % =========================================================================
    
    % Vazão de entrada constante
    Q0 = q; % Usa a vazão média 'q' calculada como vazão de equilíbrio
    
    % Condições iniciais (tanques vazios)
    h0_1 = [0; 0; 0]; % [h1(0); h2(0); h3(0)]
    
    % Período de simulação (10 minutos)
    tspan1 = [0 600]; 
    
    % Resolvendo a EDO
    % Usamos uma função anônima para passar os parâmetros (Q, A, R, ...)
    % para a função da EDO (three_reservoir_ode)
    [t1, h_sol1] = ode45(@(t, h) three_reservoir_ode(t, h, Q0, A, R, rho, g, h_max), tspan1, h0_1);
    
    % Alturas de equilíbrio (último ponto da simulação)
    h_ss = h_sol1(end, :); % h_ss = [h1_ss, h2_ss, h3_ss]
    
    fprintf('--- Simulação 1: Alturas de Equilíbrio (Regime Permanente) ---\n');
    fprintf('h1 (ss): %.4f m\n', h_ss(1));
    fprintf('h2 (ss): %.4f m\n', h_ss(2));
    fprintf('h3 (ss): %.4f m\n', h_ss(3));
    
    % =========================================================================
    % 3. SIMULAÇÃO 2: RESPOSTA A DEGRAU (+20% NA VAZÃO)
    % =========================================================================
    
    % Nova vazão (aumento de 20%)
    Q_step = Q0 * 1.20;
    
    % Condições iniciais (alturas de equilíbrio da Simulação 1)
    h0_2 = h_ss'; % Transpõe para vetor coluna
    
    % Período de simulação (10 minutos)
    tspan2 = [0 600];
    
    % Resolvendo a EDO
    [t2, h_sol2] = ode45(@(t, h) three_reservoir_ode(t, h, Q_step, A, R, rho, g, h_max), tspan2, h0_2);
    
    fprintf('\n--- Simulação 2: Concluída (Resposta a degrau) ---\n');
    
    % =========================================================================
    % 4. PARÂMETROS COMUNS DO CONTROLADOR
    % =========================================================================
    
    % Parâmetros do Controlador PI (PLACEHOLDERS - Requerem sintonia!)
    Kp = 0.005; %  (Ganho Proporcional)
    Ki = 0.0001;% (Ganho Integral)
    
    % Limites da vazão de entrada (Saturação da bomba)
    Q_max = Q0 * 3;  % este valor é um placeholder (Vazão máx. 3x a de equilíbrio)
    Q_min = 0;       % Vazão não pode ser negativa
    
    % Estado inicial do integrador para equilíbrio
    e_int_0 = Q0 / Ki; 
    
    % =========================================================================
    % 5. SIMULAÇÃO 3: CONTROLADO (RASTREAMENTO DE SETPOINT)
    % =========================================================================
    
    % Definição do Setpoint (Degrau no setpoint de h3)
    h3_ref_inicial = h_ss(3);        % Setpoint inicial (nível de equilíbrio)
    h3_ref_final = h3_ref_inicial * 1.20; % Degrau de +20% no setpoint
    t_step = 100; % Tempo (s) em que o degrau é aplicado
    
    % Condições iniciais para o sistema controlado
    % h_aug = [h1, h2, h3, e_int]
    h0_3 = [h_ss(1); h_ss(2); h_ss(3); e_int_0]; % Vetor de 4 estados
    
    % Período de simulação (mais longo para ver a estabilização)
    tspan3 = [0 1200];
    
    % Resolvendo a EDO do sistema controlado
    [t3, h_sol3] = ode45(@(t, h_aug) controlled_reservoir_ode(t, h_aug, A, R, rho, g, ...
                                    Kp, Ki, Q_min, Q_max, h_max, ...
                                    t_step, h3_ref_inicial, h3_ref_final), ...
                                    tspan3, h0_3);
    
    fprintf('\n--- Simulação 3: Concluída (Rastreamento de Setpoint) ---\n');

    % =========================================================================
    % 6. SIMULAÇÃO 4: CONTROLADO (REJEIÇÃO A DISTÚRBIO)
    % =========================================================================
    
    % O Setpoint permanece CONSTANTE
    h3_ref_disturb = h_ss(3); % Setpoint é o equilíbrio
    t_step_disturb = 0; % Sem degrau no setpoint
    
    % Define o distúrbio (5 cm de altura adicionados subitamente a h3)
    h_disturb = 0.05; % este valor é um placeholder
    
    % Condições iniciais:
    % Começa em equilíbrio, mas com h3 5cm ACIMA do setpoint
    h0_4 = [h_ss(1); h_ss(2); h_ss(3) + h_disturb; e_int_0]; 
    
    % Período de simulação
    tspan4 = [0 1200];
    
    % Resolvendo a EDO (mesma função de controle)
    [t4, h_sol4] = ode45(@(t, h_aug) controlled_reservoir_ode(t, h_aug, A, R, rho, g, ...
                                    Kp, Ki, Q_min, Q_max, h_max, ...
                                    t_step_disturb, h3_ref_disturb, h3_ref_disturb), ... % Setpoint inicial e final são os mesmos
                                    tspan4, h0_4);
                                    
    fprintf('\n--- Simulação 4: Concluída (Rejeição a Distúrbio) ---\n');
    
    % =========================================================================
    % 7. PLOTAGEM DOS RESULTADOS
    % =========================================================================
    
    % --- FIGURA 1: SISTEMA NÃO-CONTROLADO ---
    figure; % Cria a primeira figura
    sgtitle('Simulações do Sistema em Malha Aberta (Não Controlado)'); % Título da Figura 1
    
    % Plot Simulação 1
    subplot(2, 1, 1); % Primeiro subplot (2 linhas, 1 coluna, posição 1)
    plot(t1, h_sol1(:, 1), 'b-', 'LineWidth', 1.5');
    hold on;
    plot(t1, h_sol1(:, 2), 'r--', 'LineWidth', 1.5');
    plot(t1, h_sol1(:, 3), 'g:', 'LineWidth', 2');
    yline(h_max, 'm--', 'LineWidth', 1); % Linha de limite
    title('Simulação 1: Resposta a Vazão Constante (Q_0)');
    xlabel('Tempo (s)');
    ylabel('Altura (m)');
    legend('h_1', 'h_2', 'h_3', 'h_{max}', 'Location', 'best');
    grid on;
    
    % Plot Simulação 2
    subplot(2, 1, 2); % Segundo subplot (2 linhas, 1 coluna, posição 2)
    plot(t2, h_sol2(:, 1), 'b-', 'LineWidth', 1.5');
    hold on;
    plot(t2, h_sol2(:, 2), 'r--', 'LineWidth', 1.5');
    plot(t2, h_sol2(:, 3), 'g:', 'LineWidth', 2');
    yline(h_max, 'm--', 'LineWidth', 1); % Linha de limite
    title('Simulação 2: Resposta a Degrau (Q_0 * 1.20) a partir do Regime Permanente');
    xlabel('Tempo (s)');
    ylabel('Altura (m)');
    legend('h_1', 'h_2', 'h_3', 'h_{max}', 'Location', 'best');
    grid on;

    % --- FIGURA 2: SISTEMA CONTROLADO ---
    figure; % Cria a segunda figura
    sgtitle('Simulações do Sistema em Malha Fechada (Controlado)'); % Título da Figura 2

    % Plot Simulação 3 (Rastreamento de Setpoint)
    subplot(2, 1, 1);
    plot(t3, h_sol3(:, 1), 'b-', 'LineWidth', 1.5');
    hold on;
    plot(t3, h_sol3(:, 2), 'r--', 'LineWidth', 1.5');
    plot(t3, h_sol3(:, 3), 'g:', 'LineWidth', 2');
    
    % Cria o vetor do setpoint para plotar a referência
    h3_ref_vec = (t3 >= t_step) * h3_ref_final + (t3 < t_step) * h3_ref_inicial;
    plot(t3, h3_ref_vec, 'k--', 'LineWidth', 1.5');
    yline(h_max, 'm--', 'LineWidth', 1); % Linha de limite
    
    title('Simulação 3: Resposta Controlada (PI) a Degrau no Setpoint de h_3');
    xlabel('Tempo (s)');
    ylabel('Altura (m)');
    legend('h_1', 'h_2', 'h_3 (controlada)', 'Setpoint (h_3)', 'h_{max}', 'Location', 'best');
    grid on;
    
    % Plot Simulação 4 (Rejeição a Distúrbio)
    subplot(2, 1, 2);
    plot(t4, h_sol4(:, 1), 'b-', 'LineWidth', 1.5');
    hold on;
    plot(t4, h_sol4(:, 2), 'r--', 'LineWidth', 1.5');
    plot(t4, h_sol4(:, 3), 'g:', 'LineWidth', 2');
    
    % Cria o vetor do setpoint (constante)
    h3_ref_vec_4 = ones(size(t4)) * h_ss(3);
    plot(t4, h3_ref_vec_4, 'k--', 'LineWidth', 1.5');
    yline(h_max, 'm--', 'LineWidth', 1); % Linha de limite
    
    title('Simulação 4: Resposta Controlada (PI) a Distúrbio em h_3 (+5 cm)');
    xlabel('Tempo (s)');
    ylabel('Altura (m)');
    legend('h_1', 'h_2', 'h_3 (controlada)', 'Setpoint (h_3)', 'h_{max}', 'Location', 'best');
    grid on;
    
end

% =========================================================================
% FUNÇÃO DA EDO (MALHA ABERTA)
% Esta função define o sistema de equações diferenciais
% =========================================================================
function dhdt = three_reservoir_ode(t, h, Q_in, A_vec, R_vec, rho, g, h_max)
    % Desempacotar vetores
    h1 = h(1);
    h2 = h(2);
    h3 = h(3);
    
    A1 = A_vec(1);
    A2 = A_vec(2);
    A3 = A_vec(3);
    
    R1 = R_vec(1);
    R2 = R_vec(2);
    R3 = R_vec(3);
    
    % Cálculo das vazões intermediárias (q1, q2)
    q1 = sign(h1 - h2) * sqrt(rho * g * abs(h1 - h2) / R1);
    q2 = sign(h2 - h3) * sqrt(rho * g * abs(h2 - h3) / R2);
    
    % Cálculo da vazão de saída (q3)
    if h3 < 0
        q3 = 0; % Impede altura negativa
    else
        q3 = sqrt(rho * g * h3 / R3);
    end
    % --- NOVO: Lógica de Transbordamento (Overflow) ---
    % Calcula o fluxo líquido para cada tanque
    net_in_1 = Q_in - q1;
    net_in_2 = q1 - q2;
    net_in_3 = q2 - q3;
    
    % Inicializa vazões de transbordamento como zero
    q_ovf_1 = 0;
    q_ovf_2 = 0;
    q_ovf_3 = 0;
    % Se o tanque está no limite (ou acima) E o fluxo líquido é positivo (iria enchê-lo mais),
    % a vazão de transbordamento é igual ao fluxo líquido para forçar dh/dt = 0.
    if (h1 >= h_max) && (net_in_1 > 0)
        q_ovf_1 = net_in_1;
    end
    if (h2 >= h_max) && (net_in_2 > 0)
        q_ovf_2 = net_in_2;
    end
    if (h3 >= h_max) && (net_in_3 > 0)
        q_ovf_3 = net_in_3;
    end
    
    % Equações diferenciais (com transbordamento)
    dh1dt = (net_in_1 - q_ovf_1) / A1;
    dh2dt = (net_in_2 - q_ovf_2) / A2;
    dh3dt = (net_in_3 - q_ovf_3) / A3;
    
    % Retorno do vetor de derivadas
    dhdt = [dh1dt; dh2dt; dh3dt];
end
% =========================================================================
% FUNÇÃO DA EDO (MALHA FECHADA - CONTROLADOR PI)
% Esta função define o sistema aumentado com o controlador
% =========================================================================
function dh_aug_dt = controlled_reservoir_ode(t, h_aug, A_vec, R_vec, rho, g, ...
                                            Kp, Ki, Q_min, Q_max, h_max, ...
                                            t_step, h3_ref_inicial, h3_ref_final)
    
    % Desempacotar o vetor de estados aumentado
    h1 = h_aug(1);
    h2 = h_aug(2);
    h3 = h_aug(3);
    e_int = h_aug(4); % Estado do integrador
    % --- Lógica do Controlador PI ---
    
    % 1. Determinar o setpoint (h3_ref) com base no tempo
    if t < t_step
        h3_ref = h3_ref_inicial;
    else
        h3_ref = h3_ref_final;
    end
    
    % 2. Calcular o erro
    e = h3_ref - h3;
    
    % 3. Calcular a saída do controlador (Proporcional + Integral)
    Q_calc = (Kp * e) + (Ki * e_int);
    
    % 4. Aplicar Saturação (limites da bomba)
    if Q_calc > Q_max
        Q_in = Q_max;
    elseif Q_calc < Q_min
        Q_in = Q_min;
    else
        Q_in = Q_calc;
    end
    
    % --- Dinâmica do Sistema (calculada ANTES do anti-windup) ---
    
    % Desempacotar parâmetros
    A1 = A_vec(1); A2 = A_vec(2); A3 = A_vec(3);
    R1 = R_vec(1); R2 = R_vec(2); R3 = R_vec(3);
    
    % Calcular vazões (q1, q2)
    q1 = sign(h1 - h2) * sqrt(rho * g * abs(h1 - h2) / R1);
    q2 = sign(h2 - h3) * sqrt(rho * g * abs(h2 - h3) / R2);
    
    % Calcular vazão de saída (q3)
    if h3 < 0
        q3 = 0;
    else
        q3 = sqrt(rho * g * h3 / R3);
    end
    
    % --- NOVO: Lógica de Transbordamento (Overflow) ---
    net_in_1 = Q_in - q1;
    net_in_2 = q1 - q2;
    net_in_3 = q2 - q3;
    
    q_ovf_1 = 0; q_ovf_2 = 0; q_ovf_3 = 0;
    if (h1 >= h_max) && (net_in_1 > 0)
        q_ovf_1 = net_in_1;
    end
    if (h2 >= h_max) && (net_in_2 > 0)
        q_ovf_2 = net_in_2;
    end
    if (h3 >= h_max) && (net_in_3 > 0)
        q_ovf_3 = net_in_3;
    end
    
    % --- NOVO: Lógica Anti-Windup (Clamping) Atualizada ---
    
    de_int_dt = e; % Derivada do estado do integrador (assume que integra)
    
    % Condição 1: Saturação do atuador (bomba)
    bool_actuator_sat_pos = (Q_calc > Q_max) && (e > 0); % Saturado em cima E erro positivo
    bool_actuator_sat_neg = (Q_calc < Q_min) && (e < 0); % Saturado em baixo E erro negativo
    
    % Condição 2: Saturação do sistema (transbordamento)
    % Se qualquer tanque transbordar (q_ovf > 0) E o controlador ainda
    % estiver tentando subir o nível (e > 0), paramos de integrar.
    bool_overflow_sat = (q_ovf_1 > 0 || q_ovf_2 > 0 || q_ovf_3 > 0) && (e > 0);
    if bool_actuator_sat_pos || bool_actuator_sat_neg || bool_overflow_sat
        de_int_dt = 0; % Para de integrar se qualquer saturação ocorrer
    end
    
    % Equações diferenciais dos reservatórios
    dh1dt = (net_in_1 - q_ovf_1) / A1; 
    dh2dt = (net_in_2 - q_ovf_2) / A2;
    dh3dt = (net_in_3 - q_ovf_3) / A3;
    
    % Retorno do vetor de derivadas aumentado (4 estados)
    dh_aug_dt = [dh1dt; dh2dt; dh3dt; de_int_dt];
end