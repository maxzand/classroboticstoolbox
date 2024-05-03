% Inverse dynamics example if user inputs intial end effector position
% instead of inital joint variables. 

dh = [1 0 0 0 0 ; 1 0 0 0 0 ; 1 0 0 0 0];
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

% Say for example user input euler zyz coordinates (they coiuld also input 
% R directly, ZYX angles, etc.) and end effector position, then
T = eul2r(30,45,60,'deg'); 
T(1:3,4) = [1,0.6,0]; T(4,:) = [0 0 0 1];

q0 = rbt.ikunc(T);