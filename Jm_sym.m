function J = Jm_sym(DH_sym)
    %
    % DH_sym needs to oreinted [a α d θ type], where type is 0 for revolute or
    % 1 for prismatic. DH_sym is comprised of symbolic variables and/or
    % zeros (suppose you could plug in actual numbers too), and is nx5. 
    %
    % kr, a vector made in this function that makes symbolic variables for
    % each "motor" for each joint (kr1, kr2, etc.).
    %
    % This assumes z0 = [0 0 1] and p0 = [0 0 0].
    %
    % Will create a 6xnxn matrix. Jp(mi) is represented by J(1:3,:,i),
    % Jo(mi) is represented by J(4:6,i).
    %
    n = size(DH_sym,1);
    J = sym(zeros(6,n,n));
    z0 = [0; 0; 1]; p0 = [0; 0; 0];
    str = ['syms kr [1 ' num2str(n) ']']; eval(str);
    for i = 1:n
        for j = 1:n
            if j>i-1
                % Do Nothing, keep as zeros
            elseif j == 1
                if DH_sym(j,end) == 0
                    if i == 1
                        % Do nothing, keep as zeros sinze z0 x (p0-p0) = 0
                    else
                        TL = dkin_sym(DH_sym(1:i-1,1:end-1));
                        J(1:3,j,i) = cross(z0,TL(1:3,4)-p0); 
                        
                    end
                else
                    J(1:3,j,i) = z0;                    
                end    
            else
                if DH_sym(j,end) == 0
                    T = dkin_sym(DH_sym(1:j-1,1:end-1));
                    TL = dkin_sym(DH_sym(1:i-1,1:end-1));
                    J(1:3,j,i) = cross(T(1:3,3),TL(1:3,4)-T(1:3,4));
                else
                    T = dkin_sym(DH_sym(1:j-1,1:end-1));
                    J(1:3,j,i) = T(1:3,3);  
                end
            end
            if j == i
                if i == 1
                    J(4:6,j,i) = kr(i)*z0;
                else
                    TL = dkin_sym(DH_sym(1:i-1,1:end-1));
                    J(4:6,j,i) = kr(i)*TL(1:3,3);
                end
            elseif j<i
                if DH_sym(j,end) == 0
                    if j == 1
                        J(4:6,j,i) = z0;
                    else
                        T = dkin_sym(DH_sym(1:j-1,1:end-1));
                        J(4:6,j,i) = T(1:3,3);
                    end
                else
                    % Do nothing and keep as zeros for prismatic Joj
                end
            end
        end
    end
end