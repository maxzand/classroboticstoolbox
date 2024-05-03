function T = dkin_sym(DH_sym)
    %
    % DH_sym is a nx4 symbolic matrix oriented [a α d θ]
    %
    % T is the symbolic homogenous transform for the end effector pose (or
    % the pose at which the DH table stops). 
    %
    a = DH_sym(:,1); al = DH_sym(:,2); d = DH_sym(:,3); t = DH_sym(:,4); 
    T = eye(4);
    for j = 1:size(DH_sym,1)
        Tn = [cos(t(j)) , -sin(t(j))*cos(al(j)) ,  sin(t(j))*sin(al(j)) , a(j)*cos(t(j));
             sin(t(j)) ,  cos(t(j))*cos(al(j)) , -cos(t(j))*sin(al(j)) , a(j)*sin(t(j));
             0         ,  sin(al(j))           ,  cos(al(j))           , d(j)          ;
             0         ,  0                    ,  0                    , 1             ];
        T = T*Tn;
    end
    T = simplify(T);
end