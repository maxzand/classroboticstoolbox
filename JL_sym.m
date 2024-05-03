function J = JL_sym(DH_sym)
    %
    % DH_sym needs to oreinted [a α d θ type], where type is 0 for revolute or
    % 1 for prismatic. DH_sym is comprised of symbolic variables and/or
    % zeros (suppose you could plug in actual numbers too), and is nx5. 
    %
    % This assumes z0 = [0 0 1] and p0 = [0 0 0].
    %
    % Will create a 6xnxn matrix. Jp(mi) is represented by J(1:3,:,i),
    % Jo(mi) is represented by J(4:6,i).
    %
    n = size(DH_sym,1);
    J = sym(zeros(6,n,n));
    for i = 1:n
        for j = 1:n
            if j>i
                % Do Nothing, keep as zeros
            elseif j == 1
                if DH_sym(j,end) == 0
                    z0 = [0; 0; 1]; p0 = [0; 0; 0];
                    mod_DH_sym = DH_sym;
                    mod_DH_sym(i,1) = 0.5*DH_sym(i,1); % half of a
                    mod_DH_sym(i,3) = 0.5*DH_sym(i,3); % half of d
                    TL = dkin_sym(mod_DH_sym(1:i,1:end-1));
                    J(1:3,j,i) = cross(z0,TL(1:3,4)-p0);
                    J(4:6,j,i) = z0;
                else
                    J(:,j,i) = [0; 0; 1; 0 ; 0; 0];                    
                end                
            else
                if DH_sym(j,end) == 0
                    T = dkin_sym(DH_sym(1:j-1,1:end-1));
                    mod_DH_sym = DH_sym;
                    mod_DH_sym(i,1) = 0.5*DH_sym(i,1); % half of a
                    mod_DH_sym(i,3) = 0.5*DH_sym(i,3); % half of d
                    TL = dkin_sym(mod_DH_sym(1:i,1:end-1));
                    J(1:3,j,i) = cross(T(1:3,3),TL(1:3,4)-T(1:3,4));
                    J(4:6,j,i) = T(1:3,3);
                else
                    T = dkin_sym(DH_sym(1:j-1,1:end-1));
                    J(:,j,i) = [T(1:3,3) ; 0 ; 0; 0];                    
                end
            end
        end
    end
end