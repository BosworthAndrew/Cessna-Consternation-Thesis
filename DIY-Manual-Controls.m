
clc; close all; clear;

%% ---------- Load Aero Data ----------
T_aero = readtable('case12.csv');

alphas = unique(T_aero.Alpha);

Aero = struct();

for i = 1:length(alphas)
    alphaVal = alphas(i);
    alphaStr = makeAlphaField(alphaVal);

    Ta = T_aero(T_aero.Alpha == alphaVal, :);
    cases = unique(Ta.Case);

    for j = 1:length(cases)
        cname = matlab.lang.makeValidName(cases{j});
        row = Ta(strcmp(Ta.Case, cases{j}), :);

        Aero.(alphaStr).(cname).CFx = row.CFx;
        Aero.(alphaStr).(cname).CFy = row.CFy;
        Aero.(alphaStr).(cname).CFz = row.CFz;
        Aero.(alphaStr).(cname).CMx = row.CMx;
        Aero.(alphaStr).(cname).CMy = row.CMy;
        Aero.(alphaStr).(cname).CMz = row.CMz;
        Aero.(alphaStr).(cname).CL  = row.CL;
        Aero.(alphaStr).(cname).CD  = row.CD;
        Aero.(alphaStr).(cname).CS  = row.CS;
        Aero.(alphaStr).(cname).CMl = row.CMl;
        Aero.(alphaStr).(cname).CMm = row.CMm;
        Aero.(alphaStr).(cname).CMn = row.CMn;
    end
end

%% ---------- Load Derivative Data ----------
T_deriv = readtable('smoothD14.csv');

alphas_deriv = unique(T_deriv.Alpha);

Deriv = struct();

for i = 1:length(alphas_deriv)
    alphaVal = alphas_deriv(i);
    alphaStr = makeAlphaField(alphaVal);

    Ta = T_deriv(T_deriv.Alpha == alphaVal, :);
    coefs = unique(Ta.Coef);

    for j = 1:length(coefs)
        cname = matlab.lang.makeValidName(coefs{j});
        row = Ta(strcmp(Ta.Coef, coefs{j}), :);

        % Store derivative values wrt each variable
        Deriv.(alphaStr).(cname).dAlpha = row.Alpha;  % derivative wrt Alpha
        Deriv.(alphaStr).(cname).dBeta  = row.Beta;
        Deriv.(alphaStr).(cname).dp     = row.p;
        Deriv.(alphaStr).(cname).dq     = row.q;
        Deriv.(alphaStr).(cname).dr     = row.r;
        Deriv.(alphaStr).(cname).dMach  = row.Mach;
        Deriv.(alphaStr).(cname).dU     = row.U;
        Deriv.(alphaStr).(cname).dConGrp1 = row.ConGrp_1;
        Deriv.(alphaStr).(cname).dConGrp2 = row.ConGrp_2;
        Deriv.(alphaStr).(cname).dConGrp3 = row.ConGrp_3;
        Deriv.(alphaStr).(cname).dConGrp4 = row.ConGrp_4;
    end
end

aircraft = C172setup();

% Trim condition
V_des = 38;

x_runway = 1000 / 3.281;
y_runway = 0;

x_touchdown = 1000 / 3.281;

 x_init = -30380.6 / 3.281;
 x_TOD =  -30380.6 / 3.281;

  x_init = -6500;
  x_TOD =  -6500;

  altitudeDes = abs(x_TOD - x_touchdown) * tan(deg2rad(3));

altitudeIntegratorIC = altitudeDes;

 dz_des = 0;


winds = [0; 0; 0];
tf = 250;

[x_trim, controls_trim] = trimC172N18( ...
    aircraft, Aero, Deriv, V_des, dz_des, winds);

% Initial state
x0 = x_trim;
x0(10) = x_init;
x0(11) = 0;
x0(12) = -altitudeDes;

K_lqr = zeros(4, 12);  % 4 controls x X states
K_lqr2 = zeros(4, 12);  % 4 controls x X states

x_ref = x_trim;
x_ref(11) = 0;
x_ref(12) = -altitudeDes;

% Display trim
fprintf('\nTrimmed controls:\n');
fprintf(' Elevator = %.3f deg\n', rad2deg(controls_trim(1)));
fprintf(' Aileron  = %.3f deg\n', rad2deg(controls_trim(2)));
fprintf(' Rudder   = %.3f deg\n', rad2deg(controls_trim(3)));
fprintf(' Throttle = %.3f\n', controls_trim(4));


model = 'C172AutopilotWithFG';

load_system(model);
set_param(model,'FixedStep','0.01');

x_trim_lin = x_trim;

% Zero inertial positions
x_trim_lin(10) = 0;
x_trim_lin(11) = 0;
x_trim_lin(12) = 0;   % Ze

op = operpoint(model);

op.States(1).x = x_trim_lin;

op = update(op);

opInputVec = [controls_trim(:); x_ref(:)];  % 4 + 12 = 16
op.Inputs(1).u = opInputVec;

op = update(op);

sys = linearize(model, op);

A = sys.A;
B = sys.B;

idx = [1 2 3 4 5 6 7 8 9 10 11 12];

A_lqr = A(idx, idx);
control_idx = 1:4;
B_lqr = B(idx, control_idx); 



Q_micro = diag([
    1/1^2;             % u
    1/1.5^2;           % v
    1/2.5^2;             % w
    1/deg2rad(6)^2;   % p 
    1/deg2rad(4)^2;    % q 
    1/deg2rad(8)^2;    % r  
    1/deg2rad(8)^2;    % phi
    1/deg2rad(10)^2;   % theta 
    1/deg2rad(8)^2;    % psi
    1e-4;             % x
    1e-6;              % y
    1/1^2;             % z
]);

Q_norm = diag([
    1/1^2;             % u
    1/1.5^2;           % v
    1/2.5^2;           % w
    1/deg2rad(6)^2;    % p 
    1/deg2rad(4)^2;    % q 
    1/deg2rad(8)^2;    % r 
    1/deg2rad(8)^2;    % phi
    1/deg2rad(10)^2;   % theta 
    1/deg2rad(8)^2;    % psi
    1e-8;             % x
    4e-4;              % y
    1/1^2;             % z
]);

R = 2 * diag([
    1/deg2rad(1)^2;   % elevator
    1/deg2rad(6)^2;   % aileron
    1/deg2rad(6)^2;   % rudder
    1/0.05^2;          % throttle
]);

K_lqr  =  lqr(A_lqr, B_lqr, Q_micro, R);
K_lqr2  =  lqr(A_lqr, B_lqr, Q_norm, R);

eig(A_lqr -  B_lqr * K_lqr)
eig(A_lqr - B_lqr *K_lqr2)

s = svd(K_lqr);
normK = max(s);

assignin('base','K_lqr', K_lqr);
assignin('base','K_lqr2', K_lqr2);


% Set integrator IC
set_param([model '/Integrator'], ...
    'InitialCondition', mat2str(x0));

dt = 0.01;   % time step
t = 0:dt:tf;
u = repmat(controls_trim', length(t), 1);
control_ts = timeseries(u, t);


combinedData = [repmat(controls_trim', length(t), 1), repmat(x_ref', length(t), 1)];

% Create single timeseries
combined_ts = timeseries(combinedData, t);

% Feed to Simulink
simIn = Simulink.SimulationInput(model);
simIn = simIn.setExternalInput(combined_ts);  % single Inport

simIn = simIn.setModelParameter('StopTime',num2str(tf));


% x_ref to direct to airport
x_ref = x_trim;
x_ref(11) = 0;
x_ref(12) = -altitudeDes;


% Solver
simIn = simIn.setModelParameter( ...
    'SolverType','Fixed-step', ...
    'Solver','ode4', ...       
    'FixedStep','0.01');   


% Run simulations
out = sim(simIn);



function alphaStr = makeAlphaField(alphaVal)
    degAlpha = round(alphaVal);

    degAlphaInt = int32(degAlpha);

    if degAlphaInt < 0
        alphaStr = sprintf('a_minus%d', int32(abs(degAlphaInt)));
    else
        alphaStr = sprintf('a%d', degAlphaInt);
    end
end





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



function xdot = nonLinearWindN28(x, controls, wind, Aero, Deriv, aircraft)
%% STATES
u = x(1); v = x(2); w = x(3);
p = x(4); q = x(5); r = x(6);
phi = x(7); theta = x(8); psi = x(9);
xe = x(10); ye = x(11); ze = x(12);

%% AIRCRAFT PROPERTIES
m = aircraft.m; g = 9.81;
b = aircraft.b; c = aircraft.c; S = aircraft.S; rho = aircraft.rho;
Tmax = aircraft.T;
Ixx = aircraft.Ixx; Iyy = aircraft.Iyy; Izz = aircraft.Izz; Ixz = aircraft.Ixz;

%% ROTATION MATRIX & GRAVITY
R = [
    cos(theta)*cos(psi), cos(theta)*sin(psi), sin(theta);
    sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), -sin(phi)*cos(theta);
    -cos(phi)*sin(theta)*cos(psi)-sin(phi)*sin(psi), -cos(phi)*sin(theta)*sin(psi)+sin(phi)*cos(psi), cos(phi)*cos(theta)
];
g_e = [0;0;-g]; g_b = R'*g_e;
gxb = g_b(1); gyb = g_b(2); gzb = g_b(3);


%% WIND (earth frame to body frame)
W_e = wind(:);          % [Wx; Wy; Wz] in earth frame
W_b = R' * W_e;         % transform to body frame

Wxb = W_b(1);
Wyb = W_b(2);
Wzb = W_b(3);


%% RELATIVE WIND
u_rel = u - Wxb;
v_rel = v - Wyb;
w_rel = w - Wzb;

%% CONTROLS
delta_e = controls(1);
delta_a = controls(2);
delta_r = controls(3);
delta_t = controls(4);
delta_f = deg2rad(-10); % fixed flaps



%% PROPULSION / THRUST
SP_SL    = 180 * 745.7; % Shaft power sea level (Watt)
rho_SL   = 1.225;       % kg/m^3
A_p      = 1.132;       % prop constant
B_p      = A_p - 1;     % prop constant
V_design = 55;           % Design cruise speed (m/s)
prop_eff_max = 0.8;      % Maximum prop efficiency
Vinf = max(sqrt(u^2 + v^2 + w^2),0.1); % true airspeed

% Efficiency curve
prop_eff = prop_eff_max * (1 - ((Vinf - V_design)/V_design).^2);
prop_eff = max(0.3, min(prop_eff_max, prop_eff));  % clamp

Thrust = SP_SL * (A_p * (rho/rho_SL) - B_p) * (prop_eff / Vinf);
Fx_thrust = delta_t*Thrust;

% %% --- PROPWASH EFFECT ON WING ROOT ---
 D_prop = 1.905;  % prop diameter [m]
 A_prop = pi*(D_prop/2)^2;  % prop disk area

v_induced = sqrt(max(Thrust,0)/(2*rho*A_prop));

K = 0.5;
u_eff = u_rel + K * v_induced;

V = max(sqrt(u_eff^2 + v_rel^2 + w_rel^2), 0.1);
alpha = atan2(w_rel, u_eff);
beta  = asin(max(min(v_rel / V, 1), -1));


%% --- AERODYNAMIC COEFFICIENT INTERPOLATION ---
alphaDeg = rad2deg(alpha);
alphaDegClamped = max(min(alphaDeg, 25), -12);
alpha_low_int = floor(alphaDegClamped);
alpha_high_int = ceil(alphaDegClamped);
if alpha_low_int == alpha_high_int && alpha_low_int < 25
    alpha_high_int = alpha_low_int + 1;
elseif alpha_low_int == alpha_high_int && alpha_low_int == 25
    alpha_low_int = alpha_low_int - 1;
end


w_alpha = (alphaDegClamped - alpha_low_int)/(alpha_high_int - alpha_low_int);

% Retrieve Aero/Deriv structs (low)
S1 = getAeroDerivStruct(alpha_low_int, Aero, Deriv);
S2 = getAeroDerivStruct(alpha_high_int, Aero, Deriv);

% Linear interpolation
Aero_alpha  = structInterp(S1.Aero, S2.Aero, w_alpha);
Deriv_alpha = structInterp(S1.Deriv, S2.Deriv, w_alpha);

%% DYNAMIC PRESSURE
qbar = 0.5*rho*V^2;

%% AERODYNAMIC FORCES
CL = Aero_alpha.Base_Aero.CL + delta_e*Deriv_alpha.CL.dConGrp2 ...
     + delta_a*Deriv_alpha.CL.dConGrp3 + delta_r*Deriv_alpha.CL.dConGrp4 ...
     + delta_f*Deriv_alpha.CL.dConGrp1;

CD0_parasitic = 0.0156;
CD_correction = 0.09825; % 0.12 def good - 0.09825 for 0.555 % throttle 500 ft/min descent (1500/2700 rpm)
CD = CD0_parasitic + CD_correction + Aero_alpha.Base_Aero.CD ...
     + delta_e*Deriv_alpha.CD.dConGrp2 + delta_a*Deriv_alpha.CD.dConGrp3 ...
     + delta_r*Deriv_alpha.CD.dConGrp4 + delta_f*Deriv_alpha.CD.dConGrp1;

CFy = Aero_alpha.Base_Aero.CFy + Deriv_alpha.CFy.dBeta*beta ...
     + Deriv_alpha.CFy.dp*(p*b/(2*V)) + Deriv_alpha.CFy.dr*(r*b/(2*V)) ...
     + delta_a*Deriv_alpha.CFy.dConGrp3 + delta_r*Deriv_alpha.CFy.dConGrp4;

Lift = CL*qbar*S;
Drag = CD*qbar*S;
Fy_aero = CFy*qbar*S;
Fx_aero = -Drag*cos(alpha) + Lift*sin(alpha);
Fz_aero = -Drag*sin(alpha) - Lift*cos(alpha);

Fx = Fx_aero + Fx_thrust;
Fy = Fy_aero;
Fz = Fz_aero;

%% MOMENTS
L = qbar*S*b*(Aero_alpha.Base_Aero.CMl + Deriv_alpha.CMl.dBeta*beta ...
     + Deriv_alpha.CMl.dp*(p*b/(2*V)) + Deriv_alpha.CMl.dr*(r*b/(2*V)) ...
     + delta_a*Deriv_alpha.CMl.dConGrp3 + delta_e*Deriv_alpha.CMl.dConGrp2 ...
     + delta_f*Deriv_alpha.CMl.dConGrp1 + delta_r*Deriv_alpha.CMl.dConGrp4);

M = qbar*S*c*(Aero_alpha.Base_Aero.CMm + Deriv_alpha.CMm.dq*(q*c/(2*V)) ...
     + delta_e*Deriv_alpha.CMm.dConGrp2 + delta_a*Deriv_alpha.CMn.dConGrp3 ...
     + delta_r*Deriv_alpha.CMm.dConGrp4 + delta_f*Deriv_alpha.CMm.dConGrp1);

N = qbar*S*b*(Aero_alpha.Base_Aero.CMn + Deriv_alpha.CMn.dBeta*beta ...
     + Deriv_alpha.CMn.dp*(p*b/(2*V)) + Deriv_alpha.CMn.dr*(r*b/(2*V)) ...
     + delta_r*Deriv_alpha.CMn.dConGrp4 + delta_e*Deriv_alpha.CMn.dConGrp2 ...
     + delta_f*Deriv_alpha.CMn.dConGrp1 + delta_a*Deriv_alpha.CMn.dConGrp3);

%% ROTATIONAL ACCELERATIONS
p_dot = (L - (Izz-Iyy)*q*r - Ixz*(q^2-r^2))/Ixx;
q_dot = (M - (Ixx-Izz)*p*r + Ixz*(p*r))/Iyy;
r_dot = (N - (Iyy-Ixx)*p*q - Ixz*(p^2-q^2))/Izz;


%% TRANSLATIONAL ACCELERATIONS
u_dot = r*v - q*w + Fx/m + gxb;
v_dot = p*w - r*u + Fy/m + gyb;
w_dot = q*u - p*v + Fz/m - gzb;

%% EULER RATES
phi_dot   = p + q*sin(phi)*tan(theta) + r*cos(phi)*tan(theta);
theta_dot = q*cos(phi) - r*sin(phi);
psi_dot   = q*sin(phi)/cos(theta) + r*cos(phi)/cos(theta);

%% EARTH-FRAME VELOCITIES
vel_earth = R * ([u; v; w]) + W_e;
% xe_dot = vel_earth(1); ye_dot = vel_earth(2); ze_dot = -vel_earth(3);
xe_dot = vel_earth(1); ye_dot = vel_earth(2); ze_dot = vel_earth(3);


%% RETURN STATE DERIVATIVES
xdot = [u_dot; v_dot; w_dot; p_dot; q_dot; r_dot; phi_dot; theta_dot; psi_dot; xe_dot; ye_dot; ze_dot];

end

%% --- Helper Functions ---
function S = structInterp(S1, S2, w)
    S = S1;
    f = fieldnames(S1);
    for i = 1:length(f)
        if isstruct(S1.(f{i}))
            S.(f{i}) = structInterp(S1.(f{i}), S2.(f{i}), w);
        elseif isnumeric(S1.(f{i})) && isscalar(S1.(f{i}))
            S.(f{i}) = (1-w)*S1.(f{i}) + w*S2.(f{i});
        else
            S.(f{i}) = S1.(f{i});
        end
    end
end

function S = getAeroDerivStruct(alphaInt, Aero, Deriv)
    % Returns struct for given integer alpha (deg)
    switch alphaInt
        case -12, S.Aero=Aero.a_minus12; S.Deriv=Deriv.a_minus12;
        case -11, S.Aero=Aero.a_minus11; S.Deriv=Deriv.a_minus11;
        case -10, S.Aero=Aero.a_minus10; S.Deriv=Deriv.a_minus10;
        case -9,  S.Aero=Aero.a_minus9;  S.Deriv=Deriv.a_minus9;
        case -8,  S.Aero=Aero.a_minus8;  S.Deriv=Deriv.a_minus8;
        case -7,  S.Aero=Aero.a_minus7;  S.Deriv=Deriv.a_minus7;
        case -6,  S.Aero=Aero.a_minus6;  S.Deriv=Deriv.a_minus6;
        case -5,  S.Aero=Aero.a_minus5;  S.Deriv=Deriv.a_minus5;
        case -4,  S.Aero=Aero.a_minus4;  S.Deriv=Deriv.a_minus4;
        case -3,  S.Aero=Aero.a_minus3;  S.Deriv=Deriv.a_minus3;
        case -2,  S.Aero=Aero.a_minus2;  S.Deriv=Deriv.a_minus2;
        case -1,  S.Aero=Aero.a_minus1;  S.Deriv=Deriv.a_minus1;
        case 0,   S.Aero=Aero.a0;       S.Deriv=Deriv.a0;
        case 1,   S.Aero=Aero.a1;       S.Deriv=Deriv.a1;
        case 2,   S.Aero=Aero.a2;       S.Deriv=Deriv.a2;
        case 3,   S.Aero=Aero.a3;       S.Deriv=Deriv.a3;
        case 4,   S.Aero=Aero.a4;       S.Deriv=Deriv.a4;
        case 5,   S.Aero=Aero.a5;       S.Deriv=Deriv.a5;
        case 6,   S.Aero=Aero.a6;       S.Deriv=Deriv.a6;
        case 7,   S.Aero=Aero.a7;       S.Deriv=Deriv.a7;
        case 8,   S.Aero=Aero.a8;       S.Deriv=Deriv.a8;
        case 9,   S.Aero=Aero.a9;       S.Deriv=Deriv.a9;
        case 10,  S.Aero=Aero.a10;      S.Deriv=Deriv.a10;
        case 11,  S.Aero=Aero.a11;      S.Deriv=Deriv.a11;
        case 12,  S.Aero=Aero.a12;      S.Deriv=Deriv.a12;
        case 13,  S.Aero=Aero.a13;      S.Deriv=Deriv.a13;
        case 14,  S.Aero=Aero.a14;      S.Deriv=Deriv.a14;
        case 15,  S.Aero=Aero.a15;      S.Deriv=Deriv.a15;
        case 16,  S.Aero=Aero.a16;      S.Deriv=Deriv.a16;
        case 17,  S.Aero=Aero.a17;      S.Deriv=Deriv.a17;
        case 18,  S.Aero=Aero.a18;      S.Deriv=Deriv.a18;
        case 19,  S.Aero=Aero.a19;      S.Deriv=Deriv.a19;
        case 20,  S.Aero=Aero.a20;      S.Deriv=Deriv.a20;
        case 21,  S.Aero=Aero.a21;      S.Deriv=Deriv.a21;
        case 22,  S.Aero=Aero.a22;      S.Deriv=Deriv.a22;
        case 23,  S.Aero=Aero.a23;      S.Deriv=Deriv.a23;
        case 24,  S.Aero=Aero.a24;      S.Deriv=Deriv.a24;
        case 25,  S.Aero=Aero.a25;      S.Deriv=Deriv.a25;
        otherwise, S.Aero=Aero.a0; S.Deriv=Deriv.a0;
    end
end


function [x_trim, u_trim] = trimC172N18(aircraft, Aero, Deriv, V_des, dz_des, wind)

%% Initial guess
alpha0   = deg2rad(6);
de0      = deg2rad(-5);
da0      = 0;
dr0      = 0;
dt0      = 0.6;
v0       = 0;
theta0   = deg2rad(5);

%% TRIM VARIABLES (phi and psi fixed to 0 for wings-level straight flight)
z0 = [alpha0; de0; da0; dr0; dt0; v0; theta0];

%% Bounds
LB = [deg2rad(-12); deg2rad(-25); deg2rad(-20); deg2rad(-20); 0; -2; deg2rad(-5)];
UB = [deg2rad(12); deg2rad( 25); deg2rad( 20); deg2rad( 20); 1;  2; deg2rad(15)];

%% Solve using lsqnonlin
options = optimoptions("lsqnonlin", "Display", "iter", ...
    "TolFun", 1e-10, "TolX", 1e-10, ...
    "MaxFunctionEvaluations", 2000, ...
    "MaxIterations", 400);

z_sol = lsqnonlin(@(z) trimErr(z, wind, aircraft, Aero, Deriv, V_des, dz_des), ...
                  z0, LB, UB, options);

%% Extract solution
alpha = z_sol(1);
de    = z_sol(2);
da    = z_sol(3);
dr    = z_sol(4);
dt    = z_sol(5);
v     = z_sol(6);
theta = z_sol(7);
phi   = 0;  % Fixed for wings-level flight
psi   = 0;  % Fixed for straight flight

% Body velocities (AIR-RELATIVE)
u_air = V_des*cos(alpha);
w_air = V_des*sin(alpha);
v_air = v;

% Body velocities (GROUND-RELATIVE) = air-relative + wind
u = u_air + wind(1);
v = v_air + wind(2);
w = w_air + wind(3);

% Angular rates and positions
p=0; q=0; r=0;
xe=0; ye=0; ze=0;

x_trim = [u; v; w; p; q; r; phi; theta; psi; xe; ye; ze];
u_trim = [de; da; dr; dt];

%% Verify trim
xdot = nonLinearWindN28(x_trim, u_trim, wind, Aero, Deriv, aircraft);

% Compute actual airspeed components
u_rel = u - wind(1);
v_rel = v - wind(2);
w_rel = w - wind(3);
V_air = sqrt(u_rel^2 + v_rel^2 + w_rel^2);
beta_actual = atan2(v_rel, sqrt(u_rel^2 + w_rel^2));

% Compute ground speed and track
V_ground = sqrt(u^2 + v^2 + w^2);

end 

%% TRIM OBJECTIVE FUNCTION
function F = trimErr(z, wind, aircraft, Aero, Deriv, V_des, dz_des)
    alpha = z(1);
    de    = z(2);
    da    = z(3);
    dr    = z(4);
    dt    = z(5);
    v     = z(6);
    theta = z(7);
    phi   = 0;  % Fixed for wings-level flight
    psi   = 0;  % Fixed for straight flight
    
    % Build air-relative velocity components
    u_air = V_des*cos(alpha);
    w_air = V_des*sin(alpha);
    v_air = v;
    
    % Convert to ground-relative velocities
    u = u_air + wind(1);
    v = v_air + wind(2);
    w = w_air + wind(3);
    
    % Build state vector
    p=0; q=0; r=0;
    xe=0; ye=0; ze=0;
    x = [u; v; w; p; q; r; phi; theta; psi; xe; ye; ze];
    u_ctrl = [de; da; dr; dt];
    
    % Get state derivatives
    xdot = nonLinearWindN28(x, u_ctrl, wind, Aero, Deriv, aircraft);
    
    % Extract state derivatives
    du = xdot(1);
    dv = xdot(2);
    dw = xdot(3);
    dp = xdot(4);
    dq = xdot(5);
    dr_dot = xdot(6);
    dz = xdot(12);
    
    % TRIM CONDITIONS for wings-level, straight flight
    F = [
        du;          % Zero longitudinal acceleration
        dv;          % Zero lateral acceleration
        dw;          % Zero vertical acceleration (body frame)
        dp;          % Zero roll acceleration
        dq;          % Zero pitch acceleration
        dr_dot;      % Zero yaw acceleration
        dz - dz_des; % Altitude rate
    ];
end

