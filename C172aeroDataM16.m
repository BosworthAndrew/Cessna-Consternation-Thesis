clc;

function alphaStr = makeAlphaField(alphaVal)
    degAlpha = round(alphaVal);

    degAlphaInt = int32(degAlpha);

    if degAlphaInt < 0
        alphaStr = sprintf('a_minus%d', int32(abs(degAlphaInt)));
    else
        alphaStr = sprintf('a%d', degAlphaInt);
    end
end



%% ---------- Load Aero Data ----------
%T_aero = readtable('smoothCase72.csv');
%T_aero = readtable('smoothCase73.csv');
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
%T_deriv = readtable('smoothDeriv731.csv');
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

%% Example Access
% Aero data: CFx for Case "Base_Aero" at alpha = X
alphaStr = makeAlphaField(-8);
disp(Aero.(alphaStr).Base_Aero.CMm);

% Derivative data: CD derivative wrt Beta at alpha = X
alphaStr = makeAlphaField(0);
disp(Deriv.(alphaStr).CFy.dBeta);

