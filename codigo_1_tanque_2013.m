clc
close all
clear all

A = 0.095 * 0.095;
hMax = 0.27;
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

A = [-1 / (R1 * C1)];
B = [1 / C1];
C = [1];
D = 0;

[Num, Den] = ss2tf(A, B, C, D);
sys = tf(Num, Den);
N = tf(Num, 1);
D = tf(Den, 1);
s = tf([1 0], 1);

figure(1)
rlocus(sys)

kp = 0.1;
Gkp = pid(kp, 0, 0);
a = series(Gkp, sys);
sys = feedback(a, 1);

% Sintonia integrativa
sysP = (N / (s * (D + (kp * N))));
figure(2)
rlocus(sysP)

% Sintonia integrativa (continuação)
ki = 0.4; % Utilizar o mais adequado a partir do lugar das raízes anterior
Gki = pid(kp, ki, 0);
a = series(Gki, sys);
sysPI = feedback(a, 1);

opt = stepDataOptions('InputOffset', 0, 'StepAmplitude', hMax);
[Y, T] = step(sys, 0:0.01:2000, opt);
[Yp, Tp] = step(sys, 0:0.01:40, opt);
[Yi, Ti] = step(sysPI, 0:0.01:40, opt);

figure(3)
plot(T, Y(:, 1, 1), 'k', 'linewidth', 1.25)
hold on
plot(Tp, Yp(:, 1, 1), 'g', 'linewidth', 1.25)
plot(Ti, Yi(:, 1, 1), 'b', 'linewidth', 1.25)
grid on
legend('Resposta sem controle', 'Resposta com P', 'Resposta com PI', ...
    'location', 'southeast')
ylabel('Altura [m]')
xlabel('Tempo [s]')
axis([0 40 0 max(Yi(:))])