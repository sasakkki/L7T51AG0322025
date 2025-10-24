%% Lab 7 - 3 tanques
clc
close all
clear all

A = 0.095 * 0.095;
hMax = 0.26;
rho = 1000;
g = 9.78;
ti = 3;
tempf = 7;
dt = tempf - ti;
hi = 0.17;
hf = 0.0145;

q = (-hf + hi) * A / dt;
R1 = hMax / q * 3 / 4;
R2 = hMax / q * 1 / 2;
R3 = hMax / q * 1 / 4;
C1 = A;
C2 = C1;
C3 = C1;

A = [-1/(R1*C1) 1/(R1*C1) 0; 1/(R1*C2) -(1/(R1*C2) + 1/(R2*C2)) 1/(R2*C2); 0 1/(R2*C3) -(1/(C3*R3) + 1/(R2*C3))];
B = [1/C1; 0; 0];
C = eye(size(A));
D = zeros(size(B));

[Num, Den] = ss2tf(A, B, C, D);
sys = tf(Num(3, :), Den);
N = tf(Num(3, :), 1);
D = tf(Den, 1);
s = tf([1 0], 1);

opt = stepDataOptions('InputOffset', 0, 'StepAmplitude', hMax + 1/4);
[Y, T] = step(sys, 0:0.001:10000, opt);

figure(1)
rlocus(sys)

% Sintonia proporcional
kp = 0.00315; % Utilizar o mais adequado a partir do lugar das raízes anterior
Gkp = pid(kp, 0, 0);
a = series(Gkp, sys);
sysP = feedback(a, 1);
[Yp, Tp] = step(sysP, 0:0.001:4000, opt);

% Sintonia integrativa
sysPI = (N / (s * (D + (kp * N))));
figure(2)
rlocus(sysPI)

% Sintonia integrativa (continuação)
ki = 0.000232; % Utilizar o mais adequado a partir do lugar das raízes anterior
Gki = pid(kp, ki, 0);
a = series(Gki, sys);
sysPI = feedback(a, 1);
[Yi, Ti] = step(sysPI, 0:0.001:4000, opt);
% Sintonia derivativa
sysPID = ((N * (s^2)) / (s * D + (kp * s * N) + (ki * N)));
figure(2)
rlocus(sysPID)

% Sintonia derivativa (continuação)
kd = 0.0129; % Utilizar o mais adequado a partir do lugar das raízes anterior
Gkd = pid(kp, ki, kd);
a = series(Gkd, sys);
sysPID = feedback(a, 1);

[Yd, Td] = step(sysPID, 0:0.001:4000, opt);

figure(4)
plot(T, Y(:, 1, 1), 'k', 'linewidth', 1.25)
hold on
plot(Tp, Yp(:, 1, 1), 'g', 'linewidth', 1.25)
plot(Ti, Yi(:, 1, 1), 'b', 'linewidth', 1.25)
plot(Td, Yd(:, 1, 1), 'r', 'linewidth', 1.25)
grid on
legend('Resposta sem controle', 'Resposta com P', 'Resposta PI', 'Resposta PID', ...
    'location', 'southeast')
ylabel('Altura [m]')
xlabel('Tempo [s]')
axis([0 150 0 max(Yd(:))])