
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

fprintf("\n--- TRIM RESULT ---\n");
fprintf("Desired Airspeed = %.2f m/s\n", V_des);
fprintf("Actual Airspeed  = %.2f m/s\n", V_air);
fprintf("Ground Speed     = %.2f m/s\n", V_ground);
fprintf("\nWind (body frame):\n");
fprintf("  Wx = %.2f m/s\n", wind(1));
fprintf("  Wy = %.2f m/s\n", wind(2));
fprintf("  Wz = %.2f m/s\n", wind(3));
fprintf("\nBody velocities (ground frame):\n");
fprintf("  u = %.2f m/s\n", u);
fprintf("  v = %.2f m/s\n", v);
fprintf("  w = %.2f m/s\n", w);
fprintf("\nBody velocities (air-relative):\n");
fprintf("  u_rel = %.2f m/s\n", u_rel);
fprintf("  v_rel = %.2f m/s\n", v_rel);
fprintf("  w_rel = %.2f m/s\n", w_rel);
fprintf("\nAttitude:\n");
fprintf("  alpha = %.2f deg\n", rad2deg(alpha));
fprintf("  theta = %.2f deg\n", rad2deg(theta));
fprintf("  phi   = %.2f deg (fixed)\n", rad2deg(phi));
fprintf("  psi   = %.2f deg (fixed)\n", rad2deg(psi));
fprintf("  beta  = %.2f deg (sideslip - natural from asymmetries)\n", rad2deg(beta_actual));
fprintf("\nTrimmed controls:\n");
fprintf("  Elevator (de) = %.2f deg\n", rad2deg(de));
fprintf("  Aileron  (da) = %.2f deg\n", rad2deg(da));
fprintf("  Rudder   (dr) = %.2f deg\n", rad2deg(dr));
fprintf("  Throttle (dt) = %.4f\n", dt);
fprintf("\nResiduals (should be ~0):\n");
fprintf("  du     = %e\n", xdot(1));
fprintf("  dv     = %e\n", xdot(2));
fprintf("  dw     = %e\n", xdot(3));
fprintf("  dp     = %e\n", xdot(4));
fprintf("  dq     = %e\n", xdot(5));
fprintf("  dr     = %e\n", xdot(6));
fprintf("  dphi   = %e\n", xdot(7));
fprintf("  dtheta = %e\n", xdot(8));
fprintf("  dpsi   = %e\n", xdot(9));
fprintf("  dxe    = %e\n", xdot(10));
fprintf("  dye    = %e\n", xdot(11));
fprintf("  dze    = %e (target: %.2f)\n", xdot(12), dz_des);
fprintf("\nResidual norm = %e\n", norm(xdot(1:9)));

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
