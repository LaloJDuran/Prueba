clc
clear all 
close all

%% Soluciones simbolicas
% Radio del circulo
syms r;

% Centro del circulo 
syms h k;

% Posicion del robot
syms rx ry;

% Punto tangente
syms x y;

% Recta tagente
eq1 = r^2 == (x-h)*(rx-h) + (y-k)*(ry-k)

% Circunferencia
eq2 = r^2 == (x-h)^2+(y-k)^2

[solx,soly] = solve(eq1,eq2)

simplify(solx(1))
simplify(soly(1))
% solucion
% solx =
% 
% -(k^2*(h - k^2*(1/((k + r - y)*(r - k + y)))^(1/2) + r^2*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) - y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)) - r^2*(h - k^2*(1/((k + r - y)*(r - k + y)))^(1/2) + r^2*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) - y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)) + y^2*(h - k^2*(1/((k + r - y)*(r - k + y)))^(1/2) + r^2*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) - y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)) - h*y^2 - 2*k*y*(h - k^2*(1/((k + r - y)*(r - k + y)))^(1/2) + r^2*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) - y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)) + h*k*y - h*k*y0 + h*y*y0)/(k*y + k*y0 - y*y0 - k^2 + r^2)
% -(k^2*(h + k^2*(1/((k + r - y)*(r - k + y)))^(1/2) - r^2*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) + y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)) - r^2*(h + k^2*(1/((k + r - y)*(r - k + y)))^(1/2) - r^2*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) + y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)) + y^2*(h + k^2*(1/((k + r - y)*(r - k + y)))^(1/2) - r^2*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) + y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)) - h*y^2 - 2*k*y*(h + k^2*(1/((k + r - y)*(r - k + y)))^(1/2) - r^2*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) + y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)) + h*k*y - h*k*y0 + h*y*y0)/(k*y + k*y0 - y*y0 - k^2 + r^2)
% 
% 
% soly =
% 
% h - k^2*(1/((k + r - y)*(r - k + y)))^(1/2) + r^2*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y*(1/((k + r - y)*(r - k + y)))^(1/2) + k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) - y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)
% h + k^2*(1/((k + r - y)*(r - k + y)))^(1/2) - r^2*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y*(1/((k + r - y)*(r - k + y)))^(1/2) - k*y0*(1/((k + r - y)*(r - k + y)))^(1/2) + y*y0*(1/((k + r - y)*(r - k + y)))^(1/2)
 