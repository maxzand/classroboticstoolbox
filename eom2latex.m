function eom_latex = eom2latex(eom)
    eom = string(eom);
    for i = 1:length(eom)
        r = ['t_ddo' num2str(i)]; f = ['t_ddot' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['t_do' num2str(i)]; f = ['t_dot' num2str(i)];
        eom = strrep(eom,f,r);       
        r = ['d_ddo' num2str(i)]; f = ['d_ddot' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['d_do' num2str(i)]; f = ['d_dot' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['\theta_{' num2str(i) '}']; f = ['t' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['\dot{\theta_{' num2str(i) '}}'];  f = ['t_do' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['\ddot{\theta_{' num2str(i) '}}']; f = ['t_ddo' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['d_{' num2str(i) '}']; f = ['d' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['\dot{d_{' num2str(i) '}}']; f = ['d_do' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['\ddot{d_{' num2str(i) '}}']; f = ['d_ddo' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['m_{L_{' num2str(i) '}}']; f = ['mL' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['m_{m_{' num2str(i) '}}']; f = ['mm' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['k_{r_{' num2str(i) '}}']; f = ['kr' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['a_{' num2str(i) '}']; f = ['a' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['I_{L_{xx_{' num2str(i) '}}}']; f = ['ILxx' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['I_{L_{yy_{' num2str(i) '}}}']; f = ['ILyy' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['I_{L_{zz_{' num2str(i) '}}}']; f = ['ILzz' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['I_{m_{xx_{' num2str(i) '}}}']; f = ['Imxx' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['I_{m_{yy_{' num2str(i) '}}}']; f = ['Imyy' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['I_{m_{zz_{' num2str(i) '}}}']; f = ['Imzz' num2str(i)];
        eom = strrep(eom,f,r);
        r = ['\tau_{' num2str(i) '}']; f = ['tau' num2str(i)];
        eom = strrep(eom,f,r);
    end
    r = 'g_{x}'; f = 'gx'; eom = strrep(eom,f,r);
    r = 'g_{y}'; f = 'gy'; eom = strrep(eom,f,r);
    r = 'g_{z}'; f = 'gz'; eom = strrep(eom,f,r);
    r = '='; f = '=='; eom = strrep(eom,f,r);
    r = ''; f = '*'; eom = strrep(eom,f,r);

    eom_latex = eom;
end
