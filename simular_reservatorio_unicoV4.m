function simular_reservatorio_unico()
    % =========================================================================
    % 1. DEFINIÇÃO DOS PARÂMETROS DO SISTEMA
    % =========================================================================
    
    % Constantes físicas
    rho = 1000;  % Densidade da água (kg/m^3)
    g = 9.78;    % Aceleração da gravidade (m/s^2)
    
    % Parâmetros do reservatório 3
    A3 = 0.095 * 0.095; 
    
    % Limite de Transbordamento
    h_max = 0.27; % Limite de altura para transbordamento (m)
    
    % --- MODIFICADO (PREMISSA DO USUÁRIO) ---
    % Cálculo da vazão de equilíbrio (Q0) e Resistência R3
    % Baseado na aproximação da vazão média de esvaziamento de R3.

    
    h3eq = 0.243; % Ponto de equilíbrio de referência (Altura inicial do esvaziamento) 
    h3f = 0.015;  % Altura final do esvaziamento 
    dt = 57.7;    % Tempo de esvaziamento 
    
    % 'q' representa a vazão de equilíbrio Q0 (calculada como vazão média)
    q = ((h3eq - h3f) * A3) / dt;
    
    % Apenas R3 é necessária para esta simulação

    R3 = (rho * g * h3eq) / q^2;
    
    % =========================================================================
    % 2. SIMULAÇÃO 1: RESPOSTA EM MALHA ABERTA A DEGRAU NA VAZÃO
    % =========================================================================
    
    % Vazão de entrada (Equilíbrio)
    Q0 = q;
    
    % Nova vazão (aumento de 20%)
    Q_step = Q0 * 1.20;
    
    % Condições iniciais (altura de equilíbrio de h3)
    h0_1 = h3eq; % ATUALIZADO: 0.243 m
    
    % Período de simulação
    tspan1 = [0 600];
    
    % Resolvendo a EDO (Malha Aberta)
    [t1, h_sol1] = ode45(@(t, h) one_reservoir_ode(t, h, Q_step, A3, R3, rho, g, h_max), tspan1, h0_1);
    
    fprintf('--- Simulação 1 (1 Tanque): Concluída (Resposta a degrau de vazão) ---\n');

    % =========================================================================
    % 3. SIMULAÇÃO 2: RASTREAMENTO DE SETPOINT (DEGRAU)
    % =========================================================================
    
    % Parâmetros do Controlador PI (PLACEHOLDERS - Requerem sintonia!)
    Kp_1 = 0.05;  %  (Ganho Proporcional)
    Ki_1 = 0.005; %  (Ganho Integral)
    
    % Limites da vazão de entrada (Saturação da bomba)
    Q_max = Q0 * 3;  % este valor é um placeholder (Vazão máx. 3x a de equilíbrio)
    Q_min = 0;       % Vazão não pode ser negativa
    
    % Definição do Setpoint (Degrau no setpoint de h3)
    h3_ref_inicial = h3eq;                 % ATUALIZADO: 0.243 m
    h3_ref_final = h3_ref_inicial * 0.80;  % Degrau no setpoint
    t_step = 50; % Tempo (s) em que o degrau é aplicado
    
    % Condições iniciais para o sistema controlado
    % h_aug = [h3, e_int]
    e_int_0 = Q0 / Ki_1; % Estado inicial do integrador para Q_out = Q0
    h0_2 = [h3eq; e_int_0]; % ATUALIZADO: 0.243 m
    
    % Período de simulação
    tspan2 = [0 600];
    
    % Resolvendo a EDO do sistema controlado
    [t2, h_sol2] = ode45(@(t, h_aug) controlled_one_reservoir_ode(t, h_aug, A3, R3, rho, g, ...
                                    Kp_1, Ki_1, Q_min, Q_max, h_max, ...
                                    t_step, h3_ref_inicial, h3_ref_final), ...
                                    tspan2, h0_2);
    
    fprintf('\n--- Simulação 2 (1 Tanque): Concluída (Rastreamento de Setpoint) ---\n');

    % =========================================================================
    % 4. SIMULAÇÃO 3: REJEIÇÃO A DISTÚRBIO (AUMENTO SÚBITO DE NÍVEL)
    % =========================================================================
    
    % Define o distúrbio (2 cm de altura adicionados subitamente)
    h_disturb = 0.02; % este valor é um placeholder (ATUALIZADO)
    
    % O Setpoint permanece CONSTANTE no nível de equilíbrio
    h3_ref_disturb = h3eq; % ATUALIZADO: 0.243 m
    t_step_disturb = 0; % Sem degrau no setpoint
    
    % Condições iniciais:
    e_int_0_disturb = Q0 / Ki_1; 
    h0_3 = [h3eq + h_disturb; e_int_0_disturb]; % NOVO: Começa em (0.243 + 0.02) m
    
    % Período de simulação
    tspan3 = [0 600];

    % Resolvendo a EDO (mesma função de controle)
    [t3, h_sol3] = ode45(@(t, h_aug) controlled_one_reservoir_ode(t, h_aug, A3, R3, rho, g, ...
                                    Kp_1, Ki_1, Q_min, Q_max, h_max, ...
                                    t_step_disturb, h3_ref_disturb, h3_ref_disturb), ... 
                                    tspan3, h0_3);
    
    fprintf('\n--- Simulação 3 (1 Tanque): Concluída (Rejeição a Distúrbio) ---\n');
    
    % =========================================================================
    % 5. PLOTAGEM DOS RESULTADOS
    % =========================================================================
    
    % --- FIGURA 1: SISTEMA NÃO-CONTROLADO (1 TANQUE) ---
    figure; 
    %sgtitle('Simulação (1 Tanque) em Malha Aberta (Não Controlado)'); 
    
    plot(t1, h_sol1(:, 1), 'g:', 'LineWidth', 2');
    hold on;
    yline(h_max, 'm--', 'LineWidth', 1); 
    title('Simulação 1: Resposta a Degrau na Vazão (Q_0 * 1.20)');
    xlabel('Tempo (s)');
    ylabel('Altura (m)');
    legend('h_3', 'h_{max}', 'Location', 'best');
    grid on;

    % --- FIGURA 2: SISTEMA CONTROLADO (RASTREAMENTO DE SETPOINT) ---
    figure; 
   % sgtitle('Simulação (1 Tanque) em Malha Fechada (Rastreamento)'); 

    plot(t2, h_sol2(:, 1), 'g:', 'LineWidth', 2');
    hold on;
    
    % Cria o vetor do setpoint para plotar a referência
    h3_ref_vec = (t2 >= t_step) * h3_ref_final + (t2 < t_step) * h3_ref_inicial;
    plot(t2, h3_ref_vec, 'k--', 'LineWidth', 1.5');
    yline(h_max, 'm--', 'LineWidth', 1);
    
    title('Simulação 2: Resposta Controlada (PI) a Degrau no Setpoint de h_3');
    xlabel('Tempo (s)');
    ylabel('Altura (m)');
    legend('h_3 (controlada)', 'Setpoint (h_3)', 'h_{max}', 'Location', 'best');
    grid on;
    
    % --- FIGURA 3: SISTEMA CONTROLADO (REJEIÇÃO A DISTÚRBIO) ---
    figure; 
   % sgtitle('Simulação (1 Tanque) em Malha Fechada (Regulação)'); 

    plot(t3, h_sol3(:, 1), 'g:', 'LineWidth', 2'); % Altura h3
    hold on;
    
    % Cria o vetor do setpoint (constante)
    h3_ref_vec_3 = ones(size(t3)) * h3_ref_disturb;
    plot(t3, h3_ref_vec_3, 'k--', 'LineWidth', 1.5'); % Setpoint
    yline(h_max, 'm--', 'LineWidth', 1);
    
    title('Simulação 3: Resposta Controlada (PI) a Distúrbio de Nível (+2 cm)');
    xlabel('Tempo (s)');
    ylabel('Altura (m)');
    legend('h_3 (controlada)', 'Setpoint (h_3)', 'h_{max}', 'Location', 'best');
    grid on;
    
end

% =========================================================================
% FUNÇÃO DA EDO (MALHA ABERTA - 1 TANQUE)
% =========================================================================
function dhdt = one_reservoir_ode(t, h3, Q_in, A3, R3, rho, g, h_max)

    % Cálculo da vazão de saída (q3)
    if h3 < 0
        q3 = 0; 
    else
        q3 = sqrt(rho * g * h3 / R3);
    end

    % Lógica de Transbordamento (Overflow)
    net_in_3 = Q_in - q3;
    q_ovf_3 = 0;

    if (h3 >= h_max) && (net_in_3 > 0)
        q_ovf_3 = net_in_3;
    end
    
    % Equação diferencial (com transbordamento)
    dh3dt = (net_in_3 - q_ovf_3) / A3;
    
    % Retorno do vetor de derivadas
    dhdt = dh3dt;
end

% =========================================================================
% FUNÇÃO DA EDO (MALHA FECHADA - 1 TANQUE - CONTROLADOR PI)
% =========================================================================
function dh_aug_dt = controlled_one_reservoir_ode(t, h_aug, A3, R3, rho, g, ...
                                            Kp, Ki, Q_min, Q_max, h_max, ...
                                            t_step, h3_ref_inicial, h3_ref_final)
    
    % Desempacotar o vetor de estados aumentado
    h3 = h_aug(1);
    e_int = h_aug(2); % Estado do integrador

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
    
    % Calcular vazão de saída (q3)
    if h3 < 0
        q3 = 0;
    else
        q3 = sqrt(rho * g * h3 / R3);
    end
    
    % Lógica de Transbordamento (Overflow)
    net_in_3 = Q_in - q3;
    q_ovf_3 = 0;

    if (h3 >= h_max) && (net_in_3 > 0)
        q_ovf_3 = net_in_3;
    end
    
    % --- Lógica Anti-Windup (Clamping) Atualizada ---
    
    de_int_dt = e; % Derivada do estado do integrador
    
    bool_actuator_sat_pos = (Q_calc > Q_max) && (e > 0); 
    bool_actuator_sat_neg = (Q_calc < Q_min) && (e < 0); 
    bool_overflow_sat = (q_ovf_3 > 0) && (e > 0);

    if bool_actuator_sat_pos || bool_actuator_sat_neg || bool_overflow_sat
        de_int_dt = 0; % Para de integrar
    end
    
    % Equações diferenciais
    dh3dt = (net_in_3 - q_ovf_3) / A3; 
    
    % Retorno do vetor de derivadas aumentado (2 estados)
    dh_aug_dt = [dh3dt; de_int_dt];
end