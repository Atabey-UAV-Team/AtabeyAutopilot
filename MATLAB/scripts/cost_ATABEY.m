function [F0] = cost_ATABEY(Z, V_target, trim_type)
    %% Setup
    % Durumları ve Girdileri Ayır
    X = Z(1:9);
    U = Z(10:12);

    % ATABEY modelinden türevleri çek
    xdot = ATABEY_dynamics(X, U);

    % Hız (Va) ve Uçuş Yolu Açısı (gamma)
    Va    = sqrt(X(1)^2 + X(2)^2 + X(3)^2);
    alpha = atan2(X(3), X(1));
    beta  = asin(X(2) / Va);          % yan kayma açısı
    gam   = X(8) - alpha;             % theta - alpha

    % Ağırlık Matrisi (varsayılan – turn case override eder)
    H = diag(ones(1, 14));

    %% Trim Seçimi
    % trim_type:
    %   'level'    – Düz ve yatay uçuş  (varsayılan)
    %   'climb'    – Sabit tırmanış
    %   'descent'  – Sabit alçalış
    %   'turn'     – Koordineli dönüş (sabit irtifa)
    %   'sideslip' – Sabit yan kaymalı uçuş

    switch trim_type

        %------------------------------------------------------------------
        case 'level'
        %   Steady, wings-level, no-sideslip flight
        %    1-9.  xdot = 0       (steady flight)
        %    10.   Va = V_target  (airspeed)
        %    11.   gam = 0        (level flightpath)
        %    12.   X(2) = 0       (no sideslip)
        %    13.   X(7) = 0       (no bank angle)
        %    14.   X(9) = 0       (heading north)
            Q = [xdot;
                 V_target - Va;
                 gam;
                 X(2);
                 X(7);
                 X(9)];
        
            w     = ones(1, 14);
            w(10) = 100;    % Va_err baskın, ağırlığını çek
            w(8)  = 5;      % xdot8 de yüksekti, biraz bastır
            H     = diag(w);

        %------------------------------------------------------------------
        case 'climb'
        %   Steady climbing flight at fixed airspeed and climb angle
        %   Extra parameter convention: V_target = [Va_des, gam_des]
        %    1-9.  xdot = 0
        %    10.   Va = Va_des
        %    11.   gam = gam_des   (positive climb angle, rad)
        %    12.   X(2) = 0        (no sideslip)
        %    13.   X(7) = 0        (wings level)
        %    14.   X(9) = 0        (heading north)
            Va_des  = V_target(1);
            gam_des = V_target(2);          % e.g. deg2rad(5)

            Q = [xdot;
                 Va_des - Va;
                 gam_des - gam;
                 X(2);
                 X(7);
                 X(9)];

        %------------------------------------------------------------------
        case 'descent'
        %   Steady descending flight at fixed airspeed and descent angle
        %   Extra parameter convention: V_target = [Va_des, gam_des]
        %    1-9.  xdot = 0
        %    10.   Va = Va_des
        %    11.   gam = gam_des   (negative descent angle, rad)
        %    12.   X(2) = 0        (no sideslip)
        %    13.   X(7) = 0        (wings level)
        %    14.   X(9) = 0        (heading north)
            Va_des  = V_target(1);
            gam_des = V_target(2);          % e.g. deg2rad(-3)

            Q = [xdot;
                 Va_des - Va;
                 gam_des - gam;
                 X(2);
                 X(7);
                 X(9)];

        %------------------------------------------------------------------
        case 'turn'
        %   Coordinated steady turn at constant altitude
        %   Extra parameter convention: V_target = [Va_des, phi_des, psi_dot_des]
        %    1-9.  xdot = 0        (steady flight)
        %    10.   Va = Va_des     (airspeed)
        %    11.   gam = 0         (level turn – altitude holds)
        %    12.   X(2) = 0        (coordinated – no sideslip)
        %    13.   X(7) = phi_des  (desired bank angle, rad)
        %    14.   X(9) = 0        (relax heading constraint during turn)
        %   Note: psi_dot is implicitly set by phi and Va; supply psi_dot_des
        %         as V_target(3) if you want to penalise turn-rate error too.
            Va_des      = V_target(1);
            phi_des     = V_target(2);      % e.g. deg2rad(30)
            psi_dot_des = V_target(3);      % e.g. 9.81*tan(phi_des)/Va_des

            % Turn-rate from states: psi_dot ≈ X(6)/cos(X(8))  (yaw rate / cos theta)
            psi_dot = X(6) / cos(X(8));

            Q = [xdot;
                 Va_des - Va;
                 gam;
                 X(2);
                 phi_des - X(7);
                 psi_dot_des - psi_dot];

            % Up-weight turn-rate and bank tracking
            w = ones(1, 14);
            w(13) = 5;   % bank angle error weight
            w(14) = 5;   % turn rate error weight
            H = diag(w);

        %------------------------------------------------------------------
        case 'sideslip'
        %   Steady flight with a fixed sideslip angle (e.g. crosswind trim)
        %   Extra parameter convention: V_target = [Va_des, beta_des]
        %    1-9.  xdot = 0
        %    10.   Va = Va_des
        %    11.   gam = 0          (level flight)
        %    12.   beta = beta_des  (desired sideslip, rad)
        %    13.   X(7) = 0         (wings level)
        %    14.   X(9) = 0         (heading north)
            Va_des   = V_target(1);
            beta_des = V_target(2);         % e.g. deg2rad(5)

            Q = [xdot;
                 Va_des - Va;
                 gam;
                 beta_des - beta;
                 X(7);
                 X(9)];

        %------------------------------------------------------------------
        otherwise
            error('cost_ATABEY: Bilinmeyen trim tipi: "%s"\n', trim_type);
            %  Geçerli tipler: ''level'', ''climb'', ''descent'', ''turn'', ''sideslip''

    end % switch

    %% Sonuç
    F0 = Q' * H * Q;
end