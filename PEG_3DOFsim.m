function [r_,v_,t,History]=PEG_3DOFsim(r_,v_,m_0,F_T,I_sp,r_D,i_D,s)
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
    History         (:,10)  double          % [s,  m,m,m,  m/s,m/s,m/s,  N,N,N] a list of historic data, each row containing an entry of [time, position, velocity, thrust] in ECI coordinates
end

dt_g=s.dt_g;
dt_c=s.dt_c;
k_gc=round(dt_g/dt_c);
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
    History=(0:floor(-m_0/m_dot/dt_c))'*dt_c;
    i_history=1;
    History(i_history,2:7)=[r_,v_];% record current state in history
end

%% 3DOF sim

r=norm(r_);
a_T = F_T/m_0;% initialize
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
        k_gc=ceil(T_GO/dt_c);
        finalGuidance=true;
    end
    Deltav_T_=zeros(1,3);% [m/s] reset change in velocity due to thrust since last guidance call
    t_c = 0;% [s] reset seconds since last guidance call
    for i_c=(1:k_gc)
        if finalGuidance && t_c+dt_c>T_GO
            dt_c = T_GO-t_c;
            finalControl=true;
        end
        a_G_=-mu*r_/r^3;% [m/s^2] acceleration due to gravity
        if i_g==0&&i_c==1
            F_T_=F_T*P_(0);
            History(i_history,8:10)=F_T_;% record initial thrust in history
            a_ = F_T_/m_0-mu*r_/r^3;% initial value of acceleration vector
        end
        t_c=t_c+dt_c;% [s] increment control loop time
        t=t_g+t_c;% [s] current timestamp
        a_old_=a_;% [m/s^2] store old acceleration vector
        F_T_=F_T*P_(t_c);% [N] thrust vector
        a_T_=F_T_/m(t);% [m/s^2] acceleration due to thrust
        a_=a_G_+a_T_;% [m/s^2] total acceleration
        v_old_=v_;% [m/s] store old velocity
        v_=v_+(a_+a_old_)/2*dt_c;% [m/s] new velocity
        r_=r_+(v_+v_old_)/2*dt_c;% [m] new position
        if storeHistory
            i_history=i_history+1;
            History(i_history,2:10)=[r_,v_,F_T_];% record current state in history
        end
        if finalControl
            if storeHistory
                History(i_history,1)=t;% record final time
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
    History(i_history+1:end,:)=[];% cut off unused allocated space
end



