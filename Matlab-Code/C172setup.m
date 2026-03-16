function aircraft = C172setup()
% C172setup: Define aircraft parameters for C172

%% === Basic Parameters ===
aircraft.m = 1000;    % mass (kg)
aircraft.g = 9.81;        % gravity (m/s^2)
aircraft.S = 16.7544;      % wing area (m^2)
aircraft.b = 11;      % wing span (m)
aircraft.c = 1.54557;      % mean aerodynamic chord (m)
aircraft.rho = 1.202;     % air density (kg/m^3) at 200m
aircraft.T = 2200;        % max thrust (N)
aircraft.W = aircraft.m * 9.81;
%% === Inertia Matrix (kg*m^2) ===
aircraft.Ixx = 1286; aircraft.Iyy = 1825; aircraft.Izz = 2666; aircraft.Ixz = 0;
aircraft.I = [aircraft.Ixx 0 -aircraft.Ixz; 0 aircraft.Iyy 0; -aircraft.Ixz 0 aircraft.Izz];


end
