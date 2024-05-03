function eom = EOM(DH)
    % 
    % DH is a nx5 table oriented [a α d θ type] where type is 0 for
    % revolute and 1 for primsatic. All other variables may be symbolic
    % variables, especially the joint variables themselves. 
    %
    % This function will create all symbolic variables including mL, mm, 
    % IL, Im, θdot, θddot, d_dot, d_ddot, kr, and lets g0 = [gx; gy; gz].
    %
    % eom is a nx1 symbolic vector containing all equation of motions in
    % each entry
    %
    n = size(DH,1);
    % Create symbolic variables
    str = ['syms mL [1 ' num2str(n) ']']; eval(str);
    str = ['syms mm [1 ' num2str(n) ']']; eval(str);
    % str = ['syms IL [1 ' num2str(n) ']']; eval(str);
    % str = ['syms Im [1 ' num2str(n) ']']; eval(str);
    str = ['syms t_dot [1 ' num2str(n) ']']; eval(str); 
    str = ['syms t_ddot [1 ' num2str(n) ']']; eval(str);
    str = ['syms d_dot [1 ' num2str(n) ']']; eval(str); 
    str = ['syms d_ddot [1 ' num2str(n) ']']; eval(str);
    str = ['syms tau [1 ' num2str(n) ']']; eval(str);

    str = ['syms ILxx [1 ' num2str(n) ']']; eval(str);
    str = ['syms ILyy [1 ' num2str(n) ']']; eval(str);
    str = ['syms ILzz [1 ' num2str(n) ']']; eval(str);
    str = ['syms Imxx [1 ' num2str(n) ']']; eval(str);
    str = ['syms Imyy [1 ' num2str(n) ']']; eval(str);
    str = ['syms Imzz [1 ' num2str(n) ']']; eval(str);
    IL = sym(zeros([3,3,n])); Im = sym(zeros([3, 3, n]));
    for i = 1:n
        IL(:,:,i) = [ILxx(i) 0 0 ; 0 ILyy(i) 0 ; 0 0 ILzz(i)];
        Im(:,:,i) = [Imxx(i) 0 0 ; 0 Imyy(i) 0 ; 0 0 Imzz(i)];
    end

    syms gx gy gz;
    g0 = [gx; gy; gz];
    q = sym(zeros([n,1])); qdot = sym(zeros([n,1])); 
    qddot = sym(zeros([n,1]));
    for i = 1:n
        if DH(i,5) == 0
            q(i) = DH(i,4);
            qdot(i) = t_dot(i);
            qddot(i) = t_ddot(i);
        else
            q(i) = DH(i,3);
            qdot(i) = d_dot(i);
            qddot(i) = d_ddot(i);
        end
    end

    % Initialize Matricies
    B = sym(zeros(n)); c = sym(zeros(n)); G = sym(zeros([n,1]));
    JL = JL_sym(DH); Jm = Jm_sym(DH);

    % Create B matrix
    for i = 1:n
        % Ri = eye(3); 
        % Rmi = eye(3); 
        T = dkin_sym(DH(1:i,1:4));
        Ri = (T(1:3,1:3)); 
        Rmi = (T(1:3,1:3)); 
        % Using these Ri matricies and constant I values, the B matrix 
        % matches waht the book has for a two link planar arm
        B = B + ( mL(i)*transpose(JL(1:3,:,i))*JL(1:3,:,i) + ...
            transpose(JL(4:6,:,i))*Ri*IL(:,:,i)*transpose(Ri)*JL(4:6,:,i) + ...
            mm(i)*transpose(Jm(1:3,:,i))*Jm(1:3,:,i) + ...
            transpose(Jm(4:6,:,i))*Rmi*Im(:,:,i)*transpose(Rmi)*Jm(4:6,:,i) );
    end
    B = simplify(B);

    % Create c and G matrix
    for i = 1:n
        for j = 1:n
            for k = 1:n
                cijk = 0.5*(diff(B(i,j),q(k)) + diff(B(i,k),q(j)) - ...
                    diff(B(j,k),q(i)));
                c(i,j) = c(i,j) + cijk*qdot(k);
            end
            G(i) = G(i) - (mL(j)*transpose(g0)*JL(1:3,i,j) + ...
                mm(j)*transpose(g0)*Jm(1:3,i,j));
        end
    end
    c = simplify(c);
    G = simplify(G);

    % Create eom symbolic vector
    eom = B*qddot + c*qdot + G;
    eom = simplify(eom);
    eom = eom == transpose(tau);

end
