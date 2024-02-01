clc
clear all 

syms T1 T2 T3 Vx Vy W r L
alph = 30*(pi/180);
sin(alph)
cos(alph)

%% Relaciones Velocidades Robot con Velocidades de llantas
A = [T1; T2; T3];

B = [-sin(alph)/r, cos(alph)/r, L/r;
    -sin(alph)/r, -cos(alph)/r, L/r;
     1/r, 0, L/r];

C = [Vx; Vy; W];

% Obtener velocidades angulares desde globales
T = B*C;
simplify(T)
% Obtener velocidades globales desde angulares
V = inv(B)*A;

%% Modelo cinematico: Velocidades del Robot 