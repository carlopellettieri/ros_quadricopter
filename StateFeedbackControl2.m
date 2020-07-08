function C= sfb ;

% caratteristiche fisiche del drone
%consideriamo un modello semplificato del drone ovvero dove gli ingressi
%sono le coppie tau_x/tau_y/tau_z e il trust T per poter applicare il
%PolePlacement

m=0.5; %[kg] massa del drone

jx=0.0196; %[Kg*m^2] momento d'inerzia rispetto all'asse x
jy=0.0196; %[Kg*m^2] momento d'inerzia rispetto all'asse y
jz=0.0264; %[Kg*m^2] momento d'inerzia rispetto all'asse z

g=9.81; % accelerazione di gravità


% modello lineare

% A=[ 0 0 0 1 0 0 0 0 0 0 0 0;
%     0 0 0 0 1 0 0 0 0 0 0 0;
%     0 0 0 0 0 1 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 -g 0 0 0 0;
%     0 0 0 0 0 0 +g 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 1 0 0;
%     0 0 0 0 0 0 0 0 0 0 1 0;
%     0 0 0 0 0 0 0 0 0 0 0 1;
%     0 0 0 0 0 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 0 0;
%     0 0 0 0 0 0 0 0 0 0 0 0];
% 
% B=[0 0 0 0;
%    0 0 0 0;
%    0 0 0 0;
%    0 0 0 0;
%    0 0 0 0;
%    -1/m 0 0 0;
%    0 0 0 0;
%    0 0 0 0;
%    0 0 0 0;
%    0 1/jx 0 0;
%    0 0 1/jy 0;
%    0 0 0 1/jz];
% 
% C=[1 0 0 0 0 0 0 0 0 0 0 0;
%     0 1 0 0 0 0 0 0 0 0 0 0;
%     0 0 1 0 0 0 0 0 0 0 0 0]
% 
% D=0;

%ideal initial condition for position
P_star=[ 10 ;-5 ; 20]

%initial condition
% xo=[P_star*1.1 ;0.1*ones(9,1)];
xo=[P_star; zeros(9,1)];

%x_star=x-x_tilde
x_star= [P_star; zeros(9,1)]
% State feedback control
% verifica sulla controllabilità del sistema

% rank(ctrb(A,B)) % [B AB A^2B ...]
% 
% %% Autovalori
% 
% eig(A) 

%il sistema è instabile e quindi attraverso un loop posizioneremo i poli
%del sistema in modo da renderlo stabile

% Pole placement
%La scelta dei poli viene fatta in base alle dinamiche interne del drone.
%Per posizione e velocità infatti vengono scelti dei poli più lenti
%rispetto a quelli che fanno riferimento alle rotazioni
% P= [ -1 -1.5 -2 -2.5 -3 -3.5 -4 -4.5 -5 -5.5 -5 -6.5 ];
% 
% %gain to stabilize the model
% K=place(A,B,P)

% integral action

A_mu=[0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 ;
     0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 -g 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 g 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
    1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 ;
    0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 ]

% A_mu2=[A, zeros(12,3);
%     eye(3), zeros(3,12)];

B_mu= [0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   -1/m 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 0 0 0;
   0 1/jx 0 0;
   0 0 1/jy 0;
   0 0 0 1/jz;
   0 0 0 0 ;
   0 0 0 0 ;
    0 0 0 0 ];

% feedback gain
% P_mu= [ -1 -1.5 -2 -2.5 -3 -3.5 -4 -4.5 -5 -5.5 -5 -6.5 -1.2 -1.8 -2.3];
% 
% K_c=place(A_mu, B_mu, P_mu)


% LQR

% Q_m=diag([10,10,10,0.1,0.1,0.1,10,10,10,1,1,1])
% 
% R=diag([1,0.1,0.1,0.1])
% 
% N=zeros(12,4);
% 
% [K_lqr, S , eig ]= lqr(A,B,Q_m,R,N);

% LQR with integral action

Q_me=diag([20,20,20,0.1,0.1,0.1,20,20,20,1,1,1,10,10,10])

Re=diag([5,1,1,1])

Ne=zeros(15,4);

[K_lqre, S, eigs]= lqr(A_mu,-B_mu,Q_me,Re,Ne,0.01);


% sigma_star 

K1=K_lqre(:,1:12);
K2=K_lqre(:,13:15);

u_star= [m*g;0;0;0]



