function [Pfun,PEGdata,varargout]=PEG_1thrust(R_,V_,a_thrust,PEGdata,s)

% Use Powered Explicit Guidance (PEG) algorithm to calculate steering commands
%
% INPUTS (for a full list including optional inputs, refer to documentation)
% R_ [m, ECI, 1×3] current position
% V_ [m/s, ECI, 1×3] current velocity
% a_thrust [m/s^2, scalar] sensed acceleration (scalar) due to thrust alone. This is named "a" in <ref1>
% DeltaV_thrust_ [m/s, 1×3] change in velocity due to thrust alone since last guidance update. This is named "DeltaV_S_" in <ref1>
% a_grav_ [m/s^2, ECI, 1×3] sensed acceleration (vector) due to gravity alone. This is named "G_NAV_" in <ref1>
% V_exhaust [m/s, scalar] exhaust velocity
% R_orbit [m] desired orbital radius
% inclination [deg] desired orbital inclination
% PEGdata: a struct containing data calculated at last guidance call. User should not edit.
%
% OUTPUTS
% Pfun(@t) a function handle set with current values of lambda_, lambda_dot_, and K that calculates thrust direction at time t
%   * Input: t [s] time since current guidance commands were issued
%   * Output: P_ [unitless, ECI, 1×3] unit vector in the direction thrust should be applied
% T_GO [s] time-to-go; predicted amount of time until rocket reaches orbit
% lambda_ [unitless] unit vector defining average thrust direction
% lambda_dot_ [s^-1] turning rate vector
% K [s] time about which the guidance solution is expanded
% PEGdata: a struct containing data calculated at this guidance call to be used upon next call. User should not edit.
%
% AUTHOR
% Joel T Harter
% Interstellar Technologies
% 2024/5/14
%
% REFERENCES
% 1. An Explicit Solution to the Exoatmospheric Powered Flight Guidance and Trajectory Optimization Problem for Rocket Propelled Vehicles
%       AIAA
%       R. F. Jaggers
%       1977
%       arc.aiaa.org/doi/10.2514/6.1977-1051

%% Parse Input
arguments

    % REQUIRED
    % Reset on every call
    R_                      (1,3)   double
    V_                      (1,3)   double
    a_thrust                (1,1)   double
    % Reset on every call after first call
    PEGdata                         struct  = struct('initialize',true)
    s.DeltaV_thrust_        (1,3)   double  = Inf(1,3)
    % Set on first call
    s.V_exhaust             (1,1)   double  = Inf
    s.orbitRadius           (1,1)   double  = Inf
    s.orbitInclination      (1,1)   double  = Inf                       % [deg] desired orbital inclination
    
    % OPTIONAL
    % Reset on every call
    s.a_grav_               (1,3)   double  = Inf(1,3)                  % [m/s^2, ECI] acceleration due to gravity sensed by navigation system; defaults to "truth" value at position R_
    % Set on first call
    s.orbitSpeed            (1,1)   double  = Inf                       % [m/s] desired speed at orbit insertion point; defaults to value for circular orbit
    s.orbitFlightPathAngle  (1,1)   double  = 0                         % [deg] desired flight path angle at orbit insertion point; defaults to 0 (circular orbit)
    s.epsilon               (1,1)   double  = 5e-2                      % convergence criterion for F_V in the corrector
    s.epsilon_R             (1,1)   double  = 30                        % [m] convergence criterion for stabilization of R_p_, approximately the same as suggested value of 100 ft <ref1 p577>
    s.epsilon_V             (1,1)   double  = 1e-2                      % [unitless] convergence criterion for D_V <ref1 p576>
    s.D                     (1,1)   double  = 0.5                       % [unitless] damping factor for convergence of U_y_ <ref1 p577>
    s.i_corr_min            (1,1)   double  = 3                         % minimum number of iterations within corrector.
    s.i_corr_max_initial    (1,1)   double  = 50                        % maximum number of iterations within corrector
    s.i_corr_max_update     (1,1)   double  = 1                         % maximum number of iterations within corrector for all calls after first call (ref1 suggests only 1 iteration is necessary)
    s.I_max                 (1,1)   double  = 10                        % maximum number of iterations for R_p_ to converge. Source recommends I_max=5 but I have found I_max=10 to be better <ref1 p577>
    s.T_GO_min              (1,1)   double  = 4                         % [s] minimum time to go; warn user if T_GO drops below this value
end

% unpack variables for all calls
initialize=PEGdata.initialize;
R = norm(R_);
G_NAV_=s.a_grav_;

if ~initialize
    % unpack variables for subsequent calls after first call

    DeltaV_S_=s.DeltaV_thrust_;
    if all(isinf(DeltaV_S_))
        error('must specify DeltaV_S_ ([m/s] change in sensed velocity since last call) on all calls after first call. (cannot be NaN)')
    end

    % Unpack stored data from previous function call
    % Design Parameters
    V_ex=PEGdata.V_ex;
    % Orbit Parameters (set by user on first call only)
    R_D=PEGdata.R_D;
    V_D=PEGdata.V_D;
    i_D=PEGdata.i_D;
    gamma_D=PEGdata.gamma_D;
    % Iteration Controls (optionally set by user on first call only)
    epsilon=PEGdata.epsilon;
    epsilon_R=PEGdata.epsilon_R;
    epsilon_V=PEGdata.epsilon_V;
    D=PEGdata.D;
    i_corr_min=PEGdata.i_corr_min;
    i_corr_max=PEGdata.i_corr_max;
    I_max=PEGdata.I_max;
    T_GO_min=PEGdata.T_GO_min;
    % Current State Variables
    D_V_=PEGdata.D_V_;
    U_y_=PEGdata.U_y_;
    T_GO=PEGdata.T_GO;
    % Steering Variables
    lambda_dot=PEGdata.lambda_dot;
    % Predicted Final State
    V_p_=PEGdata.V_p_;
    R_p_=PEGdata.R_p_;
    R_pm=PEGdata.R_pm;
    % Desired Final State
    R_D_=PEGdata.R_D_;
    V_D_=PEGdata.V_D_;

else
    % unpack variables for first call
    DeltaV_S_=zeros(1,3);
    V_ex=s.V_exhaust;
    R_D=s.orbitRadius;
    V_D=s.orbitSpeed;
    i_D=s.orbitInclination;
    gamma_D=s.orbitFlightPathAngle;
    epsilon_R=s.epsilon_R;
    D=s.D;
    epsilon_V=s.epsilon_V;
    epsilon=s.epsilon;
    i_corr_min=s.i_corr_min;
    i_corr_max=s.i_corr_max_initial;
    i_corr_max_update=s.i_corr_max_update;
    I_max=s.I_max;
    T_GO_min=s.T_GO_min;
    if isinf(V_ex)
        error('must specify V_exhaust ([m/s] exhaust velocity) on first call')
    elseif isinf(R_D)
        error('must specify orbitRadius ([m] desired orbital radius) on first call')
    elseif isinf(i_D)
        error('must specify orbitInclination ([deg] desired orbital inclination) on first call')
    elseif isinf(V_D)
        V_D = sqrt(Constants.mu/R_D);% [m/s] use default orbital speed for a circular orbit by default
    end

    % INITIALIZE <ref1 p576>
    lambda_dot = 1e-5;% [s^-1] <ref1 p576>
    lambda_ = uvec(V_);% [unitless] <ref1 p576>
    D_V_ = zeros(size(lambda_));% [m/s] <ref1 p576>
    V_p_ = 1.001*V_;% [m/s] <ref1 p576>
    R_p_ = R_;% [m] <ref1 p576>
    R_pm = norm(R_p_);% [m] magnitude of R_p_
    U_y_=uvec(cross(V_,R_));% [unitless] unit vector in opposite direction current as angular velocity vector; this should converge to be normal to orbital plane
    
end

%% Perform Calculations

% UPDATE <ref1 p577>
D_V_=D_V_-DeltaV_S_;% [m/s] <ref1 p570>

% Guidance Loop
i_corr=0;% iteration counter
while i_corr<i_corr_min || (initialize && i_corr==1) ||(i_corr<=i_corr_max && norm(V_miss_)>epsilon_V*norm(D_V_))
    if ~(initialize && i_corr == 0)% skip to corrector before performing full first full predictor-corrector loop on first call


        % TIME-T0-GO <ref1 p577>

        D_V = norm(D_V_);% <ref1 p577>
        L = D_V;% [m/s] <ref1 p571>
        tau = V_ex/a_thrust;% [s] ratio of current mass to mass flow rate. function of time. <ref1 p570>
        T_B = tau*(1-exp(-L/V_ex));% [s] burn time <ref1 p571>
        T_GO = T_B;% [s] time-to-go <ref1 p571>

        % INTEGRALS <ref1 p577>

        S = -L*(tau-T_B)+V_ex*T_B;% [m] <ref1 p570 eq32>
        J = L*T_B-S;% [m] <ref1 p571 eq33>
        Q = S*tau-V_ex*T_B^2/2;% [m*s] <ref1 p571 eq34>
        K = T_GO-S/L;% [s] <ref1 p571 eq38>

        K_p = T_GO/2;% [s] <ref1 p572 eq39>
        theta = lambda_dot*K_p;% [unitless] <ref1 p572 eq39>
        f_1 = sin(theta)/theta;% [unitless] <ref1 p572 eq39>
        f_2 = 3*(f_1-cos(theta))/theta^2;% [unitless] <ref1 p572 eq39>
        delta = lambda_dot*(K-T_GO/2);% [unitless] <ref1 p573 eq48>
        F_1 = f_1*cos(delta);% [unitless] <ref1 p573 eq48>
        F_2 = f_2*cos(delta);% [unitless] <ref1 p573 eq48>
        F_3 = F_1*(1-theta*delta/3);% [unitless] <ref1 p573 eq48>
        Q_T = F_2*(Q-S*K);% [m*s] <ref1 p573 eq48>
        S_T = F_3*S;% [m] <ref1 p573 eq48>

        % RANGE-TO-GO <ref1 p577>

        V_GO_ = F_1*D_V_;% [m/s] velocity-to-go <ref1 p577>
        if isinf(G_NAV_)% if G_NAV_ wasn't provided by the user, use true value of mu
            mu_L = Constants.mu;
        else% if G_NAV_ was provided by the user, calculate mu_L (sensed value of mu) from it
            mu_L = -dot(G_NAV_,R_)*R;% [m^3/s^2] sensed earth gravitational constant, assumed not to be constant, provided by navigation <ref1 p575>
        end
        R_F1 = 3*dot(V_,R_)/R^2;% [s^-1] <ref1 p575>
        R_F2 = 3*dot(V_p_,R_p_)/R_pm^2;% [s^-1] <ref1 p575>
        theta_1 = T_GO*sqrt(mu_L/R^3)/2;% [unitless] <ref1 p575>
        theta_2 = T_GO*sqrt(mu_L/R_pm^3)/2;% [unitless] <ref1 p575>
        B_1 = -2*theta_1^2/T_GO;% [s^-1] <ref1 p575>
        B_2 = -2*theta_2^2/T_GO;% [s^-1] <ref1 p575>
        T_1 = tan(theta_1)/theta_1;% [unitless] <ref1 p575>
        T_2 = tan(theta_2)/theta_2;% [unitless] <ref1 p575>
        R_GO_ = R_D_/T_2 - R_/T_1 - T_GO/2*(V_ + V_D_ - V_GO_);% [m] range-to-go vector <ref1 p576 eq69>
        lambda_ = uvec(V_GO_);% [unitless] average thrust direction unit vector <ref1 p576 just below eq69-a>
        R_GO_ = R_GO_ + (S_T - dot(lambda_,R_GO_))*lambda_;% [m] range-to-go vector, enforcing that lambda_ and lambda_dot_ are orthogonal to eachother <ref1 p576 eq69-a>

        % PREDICTOR <ref1 p577>

        I=0;% iteration counter
        while I==0 || (abs(norm(R_p_)-R_pm)>epsilon_R && I<I_max)
            I=I+1;% increment the counter
            R_pm = norm(R_p_);% [m] magnitude of R_p_
            theta_2 = T_GO*sqrt(mu_L/R_pm^3)/2;% [unitless] <ref1 p575>
            T_2 = tan(theta_2)/theta_2;% [unitless] <ref1 p575>
            A_1 = T_2*(B_1+(1-1/T_1)*R_F1);% [s^-1] <ref1 p575>
            A_2 = T_2*B_2-(T_2-1)*R_F2;% [s^-1] <ref1 p575>
            A_3 = 1/T_1+A_1*T_GO/2;% [unitless] <ref1 p575>
            A_4 = 1/T_2-A_2*T_GO/2;% [unitless] <ref1 p575>
            R_p_ = (A_3*R_+T_GO/2*((1+T_2/T_1)*V_ + (T_2-1)*V_GO_)+R_GO_)/A_4;% [m] predicted position <ref1 p575 eq61>
            V_p_ = T_2/T_1*V_+A_1*R_+A_2*R_p_+T_2*V_GO_;% [m] predicted velocity <ref1 p575 eq62>
            if I>=I_max
                warning('max number of predictor loop iterations reached')
                break
            end
        end

    end% program skips to here on initialization
    i_corr=i_corr+1;

    % CORRECTOR <ref1 p577>
    
    U_x_ = uvec(R_p_ - dot(U_y_,R_p_)*U_y_);% [unitless] remove U_y_ component from R_p_ and scale to length 1 <ref1 p576 eq63>
    R_D_ = R_D*U_x_;% [m] <ref1 p576 eq63>
    U_z_ = cross(U_x_,U_y_);% [unitless] unit vector down range <ref1 p576 eq64>

    V_D_ = V_D*(U_x_*sind(gamma_D) + U_z_*cosd(gamma_D));% [m/s] desired velocity vector <ref1 p576 eq64>
    V_miss_ = V_D_ - V_p_;% [m/s] V_D_ error term <ref1 p576 eq65>
    D_V_ = D_V_ + V_miss_;% [m/s] correct D_V_ <ref1 p576 eq65>
    lambda_V_ = uvec(D_V_);% [unitless] <ref1 p576>
    F_V = (dot(lambda_V_,V_D_-V_)/dot(lambda_,V_p_-V_))-1;% [unitless] <ref1 p576 eq66>
    if abs(F_V)<epsilon
        D_V_ = D_V*(1+F_V)*lambda_V_;% [m/s] <ref1 p576 eq 65-a>
    end

    S_epsilon = -D*(U_y_(3)+cosd(i_D))*U_z_(3)/(1-U_x_(3)^2);% [unitless] <ref1 p577>
    U_y_ = U_y_*sqrt(1-S_epsilon^2)+S_epsilon*U_z_;% [unitless] <ref1 p577>

end
if i_corr_min<i_corr && i_corr==i_corr_max
    warning('Maximum number of predictor-corrector loop iterations reached')
end
if initialize
    i_corr_max = i_corr_max_update;% set the correction loop to run fewer times after initial convergence at first timestep <ref1 p572> (reference recomments i_corr_max_update=1)
end
if T_GO<T_GO_min
    warning(['Guidance called within ',num2str(T_GO_min),' s of burnout. Solution may be unstable.'])
end

% Compute steering command variables
K = J/L;% [s] <ref1 p570 eq24>
lambda_ = uvec(V_GO_);% [unitless] <ref1 p570 eq25>
S_T = dot(lambda_,R_GO_);% [m] <ref1 p570>
R_GO_ = R_GO_ + (S_T - dot(lambda_,R_GO_))*lambda_;% [m] <ref1 p570 eq26>
lambda_dot_ = (R_GO_ - S_T*lambda_)/Q_T;% [s^-1] <ref1 p570 eq27>
lambda_dot = norm(lambda_dot_);% [s^-1]  store magnitude of lambda_dot_ vector
Pfun=@(t)lambda_.*cos(lambda_dot.*(t-K))+lambda_dot_/lambda_dot.*sin(lambda_dot.*(t-K));

%% Prepare Output

varargout={T_GO,K,lambda_,lambda_dot_};

PEGdata.V_ex = V_ex;
PEGdata.R_D = R_D;
PEGdata.i_D = i_D;
PEGdata.V_D = V_D;
PEGdata.gamma_D = gamma_D;
PEGdata.epsilon = epsilon;
PEGdata.epsilon_R = epsilon_R;
PEGdata.epsilon_V = epsilon_V;
PEGdata.D = D;
PEGdata.i_corr_min=i_corr_min;
PEGdata.i_corr_max = i_corr_max;
PEGdata.I_max = I_max;
PEGdata.T_GO_min = T_GO_min;
PEGdata.D_V_ = D_V_;
PEGdata.U_y_ = U_y_;
PEGdata.T_GO = T_GO;
PEGdata.lambda_dot = lambda_dot;
PEGdata.R_p_ = R_p_;
PEGdata.R_pm = R_pm;
PEGdata.V_p_ = V_p_;
PEGdata.R_D_ = R_D_;
PEGdata.V_D_ = V_D_;
PEGdata.initialize = false;
