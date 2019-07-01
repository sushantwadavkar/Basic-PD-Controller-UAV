function c = ProportionalDerivative(Kp_phi, Kd_phi, Kp_theta, Kd_theta, Kp_psi, Kd_psi)
    
    if nargin == 0
        Kp_phi = 5;
        Kd_phi = 0.4;
        Kp_theta = 5;
        Kd_theta = 0.4;
        Kp_psi = 5;
        Kd_psi = 0.4;

    elseif nargin == 1 || nargin > 2
        error('Incorrect number of parameters.');
    end

    
    c = @(state, thetadot) pd_controller(state, thetadot, Kp_phi, Kd_phi, Kp_theta, Kd_theta, Kp_psi, Kd_psi);

end



function [inputs,state] = pd_controller(state, thetadot, Kp_phi, Kd_phi, Kp_theta, Kd_theta, Kp_psi, Kd_psi)
    
    if ~isfield(state,'orientation')
        state.orientation = zeros (3,1);
    end

    
    inputs = zeros(4,1);

    
    vels=state.xdot;
    
    phi_des=0.0;
    theta_des=0.0;
    psi_des=0.0;
    
    phi=state.orientation(1);
    theta=state.orientation(2);
    psi=state.orientation(3);
    
    phidot=thetadot(1);
    thetadot2=thetadot(2);
    psidot=thetadot(3);
    
    error_phi=phi_des-phi;
    error_theta=theta_des-theta;
    error_psi=psi_des-psi;
    
    e = zeros(3,1);
    e(1)=Kp_phi*-error_phi+Kd_phi*phidot;
    e(2)=Kp_theta*-error_theta+Kd_theta*thetadot2;
    e(3)=Kp_psi*-error_psi+Kd_psi*psidot;

    e_phi=e(1);
    e_theta=e(2);
    e_psi=e(3);

    Ixx=state.I(1,1);
    Iyy=state.I(2,2);
    Izz=state.I(3,3);

    b=state.b;
    k=state.k;
    L=state.L;
    m=state.m;
    g=state.g;



    %Now compute total thrust (to simplify input equations)
    thrust = m*g/(k*cos(phi)*cos(theta));

    %Now compute the inputs
    inputs(1)= thrust/4 - (2*b*e_phi*Ixx + k*L*e_psi*Izz)/(4*b*L*k);
    inputs(2)= thrust/4 - (2*b*e_theta*Iyy - k*L*e_psi*Izz)/(4*b*L*k);
    inputs(3)= thrust/4 - (-2*b*e_phi*Ixx + k*L*e_psi*Izz)/(4*b*L*k);
    inputs(4)= thrust/4 + (2*b*e_theta*Iyy + k*L*e_psi*Izz)/(4*b*L*k);

    
    state.orientation = state.orientation + state.dt*thetadot;
end