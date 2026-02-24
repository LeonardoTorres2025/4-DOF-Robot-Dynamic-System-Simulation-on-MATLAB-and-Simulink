clear all;
close all;
clc;

%% -------------------- PARAMETROS DH --------------------
L1 = Revolute('a', 0,       'alpha', pi/2, 'd', 0.061,   'offset', 0);
L2 = Revolute('a', 0.14814, 'alpha', 0,    'd', 0,       'offset', 0);
L3 = Revolute('a', 0,       'alpha', pi/2, 'd', 0,       'offset', pi/2);
L4 = Revolute('a', 0,       'alpha', 0,    'd', 0.1165);
%% -------------------- MASAS --------------------
L1.m = 0.28106;
L2.m = 0.33488;
L3.m = 0;
L4.m = 0.15786;  
%% -------------------- CENTROS DE MASA --------------------
L1.r = [0, -0.0305, 0];
L2.r = [-0.07407, 0, 0];
L3.r = [0, 0, 0];
L4.r = [0, 0, -0.05825];
%% -------------------- INERCIAS --------------------
L1.I = [ ...
    429570.87e-9, ...   % Ixx
    541406.19e-9, ...   % Iyy
    396346.61e-9, ...   % Izz
    0, ...              % Ixy
    0, ...              % Ixz
    4013.23e-9 ];       % Iyz

L2.I = [ ...
    991913.02e-9, ...
    1437769.51e-9, ...
    617772.19e-9, ...
    0, ...
    0, ...
    0 ];
L3.I = [0 0 0 0 0 0];

L4.I = [ ...
    263136.28e-9, ...
    175640.61e-9, ...
    124274.92e-9, ...
    -45.60e-9, ...
    84.86e-9, ...
    -5573.97e-9 ];


%% -------------------- CONSTRUCCION ROBOT --------------------
L = [L1, L2, L3, L4];
robot4g = SerialLink(L, 'name', 'robot4GDL');
% robot4g.display();

%% Jacobiano y cinemática
% CD = robot4g.fkine([pi/4, pi/4, pi/4, pi/4]);
% robot4g.jacob0([pi/4, pi/4, pi/4, pi/4]);

%% -------------------- TRAYECTORIA --------------------
q0 = [0 0 0 0];
qf = [pi 0 (5*pi)/6 pi];
T = 5;
t = linspace(0, T, 100);   % 100 muestras en 5 s
[q, qd, qdd] = jtraj(q0, qf, t);
tau_rtb = zeros(T, robot4g.n);
for k=1:length(t)
    tau_rtb(k,:) = robot4g.rne(q(k,:), qd(k,:), qdd(k,:));
end

figure;
subplot(2,2,1);
plot(t, tau_rtb(:,1), 'LineWidth', 1.5);
grid on;ylabel('\tau_1 (Nm)');xlabel('Tiempo (s)');title(['Torque de la junta 1']);

subplot(2,2,2);
plot(t, tau_rtb(:,2), 'LineWidth', 1.5);
grid on;ylabel('\tau_2 (Nm)');xlabel('Tiempo (s)');title(['Torque de la junta 2']);

subplot(2,2,3);
plot(t, tau_rtb(:,3), 'LineWidth', 1.5);
grid on;ylabel('\tau_3 (Nm)');xlabel('Tiempo (s)');title(['Torque de la junta 3']);

subplot(2,2,4);
plot(t, tau_rtb(:,4), 'LineWidth', 1.5);
grid on;ylabel('\tau_4 (Nm)');xlabel('Tiempo (s)');title(['Torque de la junta 4']);

% disp('Máximos de cada columna:')
% disp([ max(tau_rtb(:,1)), max(tau_rtb(:,2)), max(tau_rtb(:,3)), max(tau_rtb(:,4)) ])
% 
% disp('Mínimos de cada columna:')
% disp([ min(tau_rtb(:,1)), min(tau_rtb(:,2)), min(tau_rtb(:,3)), min(tau_rtb(:,4)) ])

% Calcular máximos y mínimos
Maximos = max(tau_rtb).*(100/9.81);
Minimos = min(tau_rtb).*(100/9.81);

% Preparar datos para la tabla
Data = [Maximos.' Minimos.'];
ColumnNames = {'Máximo', 'Mínimo'};
RowNames = {'tau1', 'tau2', 'tau3', 'tau4'};

% Crear la figura con tabla
f = figure;
t = uitable(f,...
    'Data', Data,...
    'ColumnName', ColumnNames,...
    'RowName', RowNames,...
    'Position', [20 20 300 130]);
