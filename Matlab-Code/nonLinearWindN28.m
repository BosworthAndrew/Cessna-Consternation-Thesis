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

% alphaDeg = rad2deg(alpha);
% alphaDegClamped = max(min(alphaDeg, 12), -12);
% alpha_low_int = floor(alphaDegClamped);
% alpha_high_int = ceil(alphaDegClamped);
% if alpha_low_int == alpha_high_int && alpha_low_int < 12
%     alpha_high_int = alpha_low_int + 1;
% elseif alpha_low_int == alpha_high_int && alpha_low_int == 12
%     alpha_low_int = alpha_low_int - 1;
% end

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
