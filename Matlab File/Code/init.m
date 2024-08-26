clc
clear all
X0=[1.0000e-02
1.4261e-02
1.9724e-02
2.6693e-02
3.5531e-02
4.6651e-02
6.0506e-02
7.7558e-02
9.8234e-02
1.2285e-01
1.5154e-01
1.8415e-01
2.2016e-01
2.5871e-01
2.9861e-01
3.3850e-01
3.7702e-01
4.1298e-01
4.4553e-01
4.7416e-01
4.9872e-01
5.2649e-01
5.5776e-01
5.9216e-01
6.2904e-01
6.6751e-01
7.0650e-01
7.4489e-01
7.8163e-01
8.1583e-01
8.4687e-01
8.7439e-01
8.9830e-01
9.1870e-01
9.3585e-01
9.5007e-01
9.6174e-01
9.7124e-01
9.7891e-01
9.8507e-01
9.9000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01
5.0000e-01];
CC=zeros(4,82);
CC(1,1) = 1;
CC(2,41)=1;
CC(3,42)=1;
CC(4,82)=1;

Ls=2.7063; Vs=3.2063; Ds=0.5; Bs=0.5; Fs=1.0; zFs=0.5; qF = 1.0;
U0 = [Ls Vs Ds Bs Fs zFs qF]';
[A,B,C,D]=cola_linearize('cola4_lin',X0,U0);
X_prime0 = colamod(0,X0,U0);
G4u =  pck(A,B,C,D);
Y0 = CC*X0

dt = 0.01; 

B_n = B(:,1:4);
D_n = D(:,1:4);
sys = ss(A, B_n, C, D_n);

sys_d = c2d(sys,dt);

Ts = dt; % sample time
predictionHorizon = 300; % prediction horizon 
controlHorizon = 50; % control horizon 

mpc_controller = mpc(sys_d, Ts, predictionHorizon, controlHorizon);
mpc_controller = mpc(sys_d, Ts, predictionHorizon, controlHorizon);

% weight
mpc_controller.Weights.OutputVariables = 10*[1 50 1 5];
mpc_controller.Weights.ManipulatedVariables = 0.1*[0.01 0.01 1 0.1];
mpc_controller.Weights.ManipulatedVariablesRate = 0.001*[0.1,0.1,1,0.1];

% constraint
mpc_controller.MV(1).Min = -10;
mpc_controller.MV(1).Max = 10;

mpc_controller.MV(2).Min = -10;
mpc_controller.MV(2).Max = 10;

mpc_controller.MV(3).Min = -10;
mpc_controller.MV(3).Max = 10;

mpc_controller.MV(4).Min = -10;
mpc_controller.MV(4).Max = 10;


x0 = mpcstate(mpc_controller); 
% initial state vector
x0.Plant = 0.0*X0;
