function [t,q,qd,qdd,xe] = moctrl_PDGrav(dh,xdes,q0,qd0,T,Kp,Kd,g0,mL,mm,Il,Im,kr,Fv,Fs)
%
% Motion control in the operational space using a PD controller with
% gravity conpensation for a end effector postion in desired equilibrium. 
% 
% Inputs
%
% dh is a nx5 table formatted [a α d θ type]. Type = 0 for revolute, 1 for
% prismatic. If revolute, θ will be ignored, if pristmatic, d will be
% ignored
%
% xdes (nx1) is the desired joint position, CONSTANT VALUES ONLY
%
% q0 and qd0 are nx1 vectors for the inital joint positions and velocities
%
% T is the simulation time
%
% Kp, Kd are scalar values for proportional and derivative gains
%
% g0 is a 3x1 vector of gravity acceleration in base frame
%
% mL, mm are nx1 vectors for the masses of links and motors
%
% Il is a 3x3xn matrix for the inertia tensor for each link
%
% Im is an nx1 vector for the motor inertias
%
% kr is an nx1 vector for the gear ratios
%
% Fv and Fs are nx1 vector for the viscous and coulomb friction
%
%
% Outputs
%
% t is a time vector 
%
% q is a nxlength(t) matrix of the joint varibales as f(t)
%
% qd is a nxlength(t) matrix of the joint rates as f(t)
%
% qdd is a nxlength(t) matrix of the joint accelerations as f(t)
%
% xe is a 3xlength(t) matrix of the end effector position as f(t)


dt = 0.01; % Step interval is always the same
N = T/dt; 
t = 0; q = q0; qd = qd0;
qdd = zeros([size(dh,1),1]);
 
for i = 1:size(dh,1)
    if dh(i,5) == 0
        L(i) = Link('revolute','d',dh(i,3),'alpha',dh(i,2),'a', ...
            dh(i,1),'m',mL(i),'I',Il(:,:,i),'Jm',Im(i),'G',kr(i), ...
            'r',[0.5*dh(i,1);0;0.5*dh(i,3)],'B',Fv(i),'Tc',Fs(i));
    else
        L(i) = Link('prismatic','theta',dh(i,4),'alpha',dh(i,2),'a', ...
            dh(i,1),'m',mL(i),'I',Il(:,:,i),'Jm',Im(i),'G',kr(i), ...
            'r',[0.5*dh(i,1);0;0.5*dh(i,3)],'B',Fv(i),'Tc',Fs(i));
    end
end
rbt = SerialLink(L);
rbt.gravity = g0;
Ti = rbt.fkine(transpose(q(:,end)));
xe = Ti(1:3,4);

for i = 1:N
    
    Ja = rbt.jacob0(transpose(q(:,end)),'trans');
    y = transpose(Ja)*(Kp*(xdes-xe(:,end)) - Kd*Ja*qd(:,end));
    u = y + transpose(rbt.gravload(transpose(q(:,end))));

    tau = @(robot,t,q,qd) transpose(u); 

    [ti, qi, qdi] = rbt.fdyn(dt,tau,q(:,end),qd(:,end));
    qddi = ( (qdi(end,:) - qdi(end-1,:)) / (ti(end) - ti(end-1)) );

    t = [t transpose(ti(end,:))+t(end)];
    q = [q transpose(qi(end,:))];
    qd = [qd transpose(qdi(end,:))];
    qdd = [qdd transpose(qddi)];
    Ti = rbt.fkine(transpose(q(:,end)));
    xe = [xe Ti(1:3,4)];

end