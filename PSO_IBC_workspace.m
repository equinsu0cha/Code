%persistent x vn K1 K2 K3 a1 a2 a3 pbest gbest bestG bestP
%First run through
% x = [k0(1) k0(2) k0(3) a0(1) a0(2) a0(3)];
%Starting X                
%x = [-1 -1 -1 -0.1 -0.1 -0.1;
%    1 1 1 0.1 0.1 0.1];
%x =[-0.8218   -0.0926   -2.2716    0.0301    0.1357    0.0397
%    2.7715    1.2143    1.1914    0.1129    0.1370    0.0749];
%x = [10.6842   10.4955    9.9283    1.0446    1.1652    1.0278;
%   -4.0171   -2.7912   -2.7347   -0.5938   -0.6080   -0.6131];
%x = [11.5083   11.0280   10.4920    1.1172    1.2341    1.0985;
%   -4.9061   -3.6386   -3.4985   -0.6822   -0.6658   -0.7032];
x = [10.3277   10.3035   10.1684    1.1351    1.1882    1.0954;
   -3.8727   -3.0692   -2.7675   -0.5731   -0.5913   -0.5675];
vn = [1 1 1 0.2 0.2 0.2;
    -1 -1 -1 -0.2 -0.2 -0.2].';
open_system('IBC');
set_param('IBC','FastRestart','on');
count = 0;
i=1;
pGe=0;
pQe=0;
pWe=0;
Rb_0 = strcat('[',num2str([0 0 0]),']');
set_param('IBC/Starting Quaternion/Rb_0','Value',Rb_0);
G_1 = [x(1,1) x(1,4) x(1,5); x(1,4) x(1,2) x(1,6); x(1,5) x(1,6) x(1,3)];
set_param('IBC/Gamma_1','Value',strcat('[',num2str(G_1(1:3)),';',num2str(G_1(4:6)),';',num2str(G_1(7:9)),']'));
G_2 = [x(2,1) x(2,4) x(2,5); x(2,4) x(2,2) x(2,6); x(2,5) x(2,6) x(2,3)];
set_param('IBC/Gamma_2','Value',strcat('[',num2str(G_2(1:3)),';',num2str(G_2(4:6)),';',num2str(G_2(7:9)),']'));
[out] = sim('IBC');
Q_out = out.get('Q_out');
tic
for psi = 0:30:180
    for theta = 0:30:180
        for rho = 0:30:180
            Rb0 = [psi theta rho]; %creates starting euler angles (rad)
            set_param('IBC/Starting Quaternion/Rb_0','Value',strcat('[',num2str(Rb_0),']'));
            try
                t1 = cputime;
                [out]=sim('IBC');
                disp(cputime-t1);
                i = i + 1;
                Q_out = out.get('Q_out');
                pGe=pGe+Q_out(4:6);
                pQe=pQe+Q_out(1:3);
                pWe=pWe+Q_out(7:9);
            catch ME
                if (strcmp(ME.identifier,'Simulink:Engine:DerivNotFinite'))
                    disp('Null');
                    count=count+1;
                    i = i + 1;
                    pGe=pGe+10;
                    pQe=pQe+10;
                    pWe=pWe+10;
                    break;
                end
                if (strcmp(ME.identifier,'SL_SERVICES:utils:UNDEFINED_ID'))
                    disp('Err');
                    break;
                end
            end
        end
    end
end
toc
%k=waitforbuttonpress;
disp(x);
pbest=x;
gbest=x;
Qe=pQe/i;
We=pWe/i;
Ge=pGe/i;
bestP1=[Qe(1:3) We(1:3)];    
bestP2=[Qe(1) Qe(2); Qe(1) Qe(3); Qe(2) Qe(3)];
bestP3=[We(1) We(2); We(1) We(3); We(2) We(3)];
bestG=Ge;
disp(bestG);
tx = 0;
%k = waitforbuttonpress;
while(true)
    vn = vn + (tx/1000)*(rand(6,2)).*(pbest-x).' + (1-tx/1000)*(rand(6,2)).*(gbest-x).';
    x = x + 1*vn.';
    G_1 = [x(1,1) x(1,4) x(1,5); x(1,4) x(1,2) x(1,6); x(1,5) x(1,6) x(1,3)];
    set_param('IBC/Gamma_1','Value',strcat('[',num2str(G_1(1:3)),';',num2str(G_1(4:6)),';',num2str(G_1(7:9)),']'));
    G_2 = [x(2,1) x(2,4) x(2,5); x(2,4) x(2,2) x(2,6); x(2,5) x(2,6) x(2,3)];
    set_param('IBC/Gamma_2','Value',strcat('[',num2str(G_2(1:3)),';',num2str(G_2(4:6)),';',num2str(G_2(7:9)),']'));
    pGe=0;
    pQe=0;
    pWe=0;
    i=1;
    tic
    for psi = 0:30:180
        for theta = 0:30:180
            for rho = 0:30:180
                Rb0 = [psi theta rho]; %creates starting euler angles (rad)
                set_param('IBC/Starting Quaternion/Rb_0','Value',strcat('[',num2str(Rb_0),']'));
                try
                    %t1 = cputime;
                    [out]=sim('IBC');
                    %disp(cputime-t1);
                    i = i + 1;
                    Q_out = out.get('Q_out');
                    pGe=pGe+Q_out(4:6);
                    pQe=pQe+Q_out(1:3);
                    pWe=pWe+Q_out(7:9);
                catch ME
                    if (strcmp(ME.identifier,'Simulink:Engine:DerivNotFinite'))
                        disp('Null');
                        count=count+1;
                        i = i + 1;
                        pGe=pGe+10;
                        pQe=pQe+10;
                        pWe=pWe+10;
                        break;
                    end
                    if (strcmp(ME.identifier,'SL_SERVICES:utils:UNDEFINED_ID'))
                        disp('Err');
                        break;
                    end
                end
            end
        end
    end
    toc
    Qe=pQe/i;
    We=pWe/i;
    Ge=pGe/i;
    disp(['Ge: ',num2str(Ge)]);
    disp(['X1: ',num2str(x(1,1:6))]);
    disp(['X2: ',num2str(x(2,1:6))]);
    disp(['tx: ', num2str(tx)]);
    vnp = vn.';
    disp(['vn1: ', num2str(vnp(1,1:6))]);
    disp(['vn2: ', num2str(vnp(2,1:6))]);
    if(count>=20)
        disp(['count: ',num2str(count)]);
        x = gbest;
        vn = [0.5 0.5 0.5 0.05 0.05 0.05;
            -0.5 -0.5 -0.5 -0.05 -0.05 -0.05].';
    else
        if((Qe&Ge&We)~=0)
        [bestP1,bestP2,bestP3,pbest,gbest,bestG]=testE_IBC(bestP1,bestP2,bestP3,pbest,bestG,gbest,x,Qe,We,Ge,vn,tx);
        end
    end
    count = 0;
    if(tx<=1000)
        tx = tx + 1;
    else
        break
    end
end
