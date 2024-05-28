function [r_,v_,t,stateHistory]=PEG_3DOFsim(r_,v_,m_0,F_T,I_sp,r_D,i_D,s)
% Given a rocket's initial conditions, use Powered Explicit Guidance (PEG) to guide it into orbit in a 3-degree-of-freedom simulation
%
% Joel T Harter
% Interstellar Technologies
% 2024 May 27
arguments (Input)
    r_              (1,3)   double          % [m, ECI] initial position
    v_              (1,3)   double          % [m/s, ECI] initial velocity
    m_0             (1,1)   double          % [kg] initial mass
    F_T             (1,1)   double          % [N] thrust force
    I_sp            (1,1)   double          % [s] specific impulse
    r_D             (1,1)   double          % [m] desired orbital radius
    i_D             (1,1)   double          % [deg] desired orbital inclination
    s.dt_g          (1,1)   double  = 1e0   % [s] guidance timestep size
    s.dt_c          (1,1)   double  = 1e-2  % [s] control timestep size
end
arguments (Output)
    r_              (1,3)   double          % [m, ECI] orbit insertion position
    v_              (1,3)   double          % [m/s, ECI] velocity at orbit insertion
    t               (1,1)   double          % [s] time of orbit insertion
    stateHistory    (:,7)   double          % [s,  m,m,m  m/s,m/s,m/s] a list of historic data, each row containing an entry of [time, ECI position [x,y,z], ECI velocity [x,y,z]]
end

dt_g=s.dt_g;
dt_c=s.dt_c;
mu=Constants.mu;% [m^3/s^2] earth's gravitational constant
g_0=Constants.g_0;
m_dot = -F_T/g_0/I_sp;
tau = -m_0/m_dot;
V_ex = g_0*I_sp;
m=@(t)m_0+t*m_dot;

if nargout<4
    storeHistory = false;
else
    storeHistory = true;
    stateHistory=(0:floor(-m_0/m_dot/dt_c))';
    i_history=1;
    stateHistory(i_history,2:7)=[r_,v_];% record current state in history
end

%% 3DOF sim

r=norm(r_);
a_T = F_T/m_0;% initialize
a_ = a_T*uvec(v_)-mu*r_/r^3;% assumed initial value of acceleration vector
dt_ge = dt_g;
t=0;
finalGuidance=false;
finalControl=false;

for i_g=(0:tau/dt_g)
    t_g=i_g*dt_g;
    if i_g==0
        [P_,PEGdata,T_GO]=PEG_1thrust(r_,v_,a_T,...
            orbitRadius=r_D,...
            orbitInclination=i_D,...
            V_exhaust=V_ex);
    else
        [P_,PEGdata,T_GO]=PEG_1thrust(r_,v_,a_T,...
            PEGdata,...
            DeltaV_thrust_=Deltav_T_);
    end
    if T_GO-dt_g<PEGdata.T_GO_min% if getting too close to T_GO, don't call guidance anymore
        dt_ge = T_GO;
        finalGuidance=true;
    end
    Deltav_T_=zeros(1,3);% [m/s] reset change in velocity due to thrust since last guidance call
    t_c = 0;% [s] reset seconds since last guidance call
    for i_c=(1:round(dt_ge/dt_c)+1)
        if finalGuidance && t_c+dt_c>T_GO
            dt_c = T_GO-t_c;
            finalControl=true;
        end
        t_c=t_c+dt_c;
        t=t_g+t_c;% current timestamp
        a_old_=a_;% store old acceleration vector
        a_G_=-mu*r_/r^3;% acceleration due to gravity
        a_T_=F_T*P_(t_c)/m(t);% acceleration due to thrust
        a_=a_G_+a_T_;% total acceleration
        v_old_=v_;% store old velocity
        v_=v_+(a_+a_old_)/2*dt_c;% new velocity
        r_=r_+(v_+v_old_)/2*dt_c;% new position
        if storeHistory
            i_history=i_history+1;
            stateHistory(i_history,2:7)=[r_,v_];% record current state in history
        end
        if finalControl
            if storeHistory
                stateHistory(i_history,1)=t;% record final time
            end
            break
        end
        r=norm(r_);% distance from center of earth
        Deltav_T_=Deltav_T_+a_T_*dt_c;% accumulate change in speed due to thrust
    end
    if finalGuidance
        break
    end
    a_T=F_T/m(t);% [m/s^2] magnitude of sensed acceleration due to thrust
end

t=i_g*dt_g+i_c*dt_c;% [s] final time
if storeHistory
    stateHistory(i_history+1:end,:)=[];% cut off unused allocated space
end


