clear; clc; close all;

%% Parametri del processo
M   = 0.0064;
m   = 0.253;
tau = 0.006;

%% Variabile di Laplace
s = tf('s');

%% Funzione di trasferimento del sistema
G = 1 / ( s * (M + m) * (tau*s + 1) );

%% Controllore C(s)
% ESEMPIO: rete anticipatrice + guadagno
% Modifica questa parte in base al loop shaping che vuoi ottenere
K  = 25.4;
wz = 1/tau;    % zero del controllore
wp = 100;   % polo del controllore

%C = K * (1 + s/wz) / (1 + s/wp);
C = K; 


%% Funzione d'anello
L = C * G;

%% Funzione di trasferimento in anello chiuso 
T = feedback(L, 1);

%% Margini di stabilita'
[Gm, Pm, Wcg, Wcp] = margin(L);
% Wcp = gain crossover frequency = pulsazione critica per il margine di fase

fprintf('Margine di fase PM = %.2f deg\n', Pm);
fprintf('Pulsazione critica wc = %.4f rad/s\n', Wcp);

%% Dati per Bode
w = logspace(-2, 3, 2000);   % intervallo di frequenze
[mag, phase, wout] = bode(L, w);

mag   = squeeze(mag);
phase = squeeze(phase);
mag_dB = 20*log10(mag);

%% Valori in corrispondenza della pulsazione critica
mag_wc   = interp1(wout, mag_dB, Wcp, 'linear');
phase_wc = interp1(wout, phase,  Wcp, 'linear');

%% Figura con modulo e fase
figure('Color','w');

% --- Modulo ---
subplot(2,1,1);
semilogx(wout, mag_dB, 'b', 'LineWidth', 1.5); hold on; grid on;
yline(0, '--k', 'LineWidth', 1);                         % 0 dB
xline(Wcp, '--r', 'LineWidth', 1.2);                    % pulsazione critica
plot(Wcp, mag_wc, 'ro', 'MarkerFaceColor', 'r');

xlabel('\omega [rad/s]');
ylabel('Modulo [dB]');
title('Diagramma di Bode del modulo di L(s) = C(s)G(s)');

text(Wcp, mag_wc, sprintf('  \\omega_c = %.3f rad/s', Wcp), ...
    'VerticalAlignment', 'bottom', 'Color', 'r');

legend('|L(j\omega)|', '0 dB', '\omega_c', 'Location', 'best');

% --- Fase ---
subplot(2,1,2);
semilogx(wout, phase, 'b', 'LineWidth', 1.5); hold on; grid on;
yline(-180, '--k', 'LineWidth', 1);                     % -180 deg
xline(Wcp, '--r', 'LineWidth', 1.2);                    % pulsazione critica
plot(Wcp, phase_wc, 'ro', 'MarkerFaceColor', 'r');

xlabel('\omega [rad/s]');
ylabel('Fase [deg]');
title('Diagramma di Bode della fase di L(s) = C(s)G(s)');

text(Wcp, phase_wc, sprintf('  PM = %.2f^\\circ', Pm), ...
    'VerticalAlignment', 'bottom', 'Color', 'r');

legend('\angle L(j\omega)', '-180^\circ', '\omega_c', 'Location', 'best');

%% Risposta allo scalino del sistema in anello chiuso
figure('Color','w');
step(T);
grid on;
title('Risposta allo scalino del sistema in anello chiuso');
xlabel('Tempo [s]');
ylabel('Uscita');

%% Informazioni sulla risposta allo scalino
info = stepinfo(T);

fprintf('\n--- Step response info ---\n');
disp(info);


%% Mostra anche le funzioni di trasferimento a video
disp('Funzione di trasferimento del processo G(s):');
G

disp('Funzione di trasferimento del controllore C(s):');
C

disp('Funzione di trasferimento d''anello L(s)=C(s)G(s):');
L

disp('Funzione di trasferimento in anello chiuso T(s):');
T