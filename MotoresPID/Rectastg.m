clc
clear all 
close all

%% Soluciones simbolicas
% Radio del circulo
% syms r;
% 
% % Centro del circulo 
% syms h k;
% 
% % Posicion del robot
% syms x0 y0;
% 
% % Punto tangente
% syms x y;
% 
% % Recta tagente
% eq1 = r^2 == (x-h)*(x0-h) + (y-k)*(y0-k)
% 
% % Circunferencia
% eq2 = r^2 == (x-h)^2+(y-k)^2
% 
% [solx,soly] = solve(eq1,eq2)

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
 
X = 20:5:120;
Y = 0:5:100;
U = zeros(length(Y),length(X));
V = zeros(length(Y),length(X));
% 
% for i = 1:1:length(X)
%     for j = 1:1:length(Y)
%% Localizacion del centro del circulo a partir de la pelota
% Radio del circulo
r = 25;

% Direccion interseccion 
dx = 0;
dy = 10;

% Posicion de Pelota
px = 40;
py = 50;

% Punto interseccion 
ix = px-dx;
iy = py-dy;

% % Posicion del robot
% rx = X(i);
% ry = Y(j);
rx = 70;
ry = 100;

% Definir vectores dependiendo de si esta a la derecha o izquierda el robot
if (rx > ix) %Derecha
    ang = 30*pi/180;
    incremento = -20*pi/180;  % 10 grados sentido horario para cuando este dentro del circulo
    alpha = -45*pi/180;
elseif (rx < ix) %Izquierda
    ang = 150*pi/180;   
    incremento = 20*pi/180;  % 10 grados sentido antihorario
    alpha = 45*pi/180;
else
    ang = 0;
end

% Vector hacia el centro del circulo  con mag = radio y ang =30°
ux = r * cos(ang);
uy = r * sin(ang);

% Centro del circulo = P_intersect + Vect
h = ix + ux;
k = iy + uy;

%% Donde esta el robot dentro del circulo?
% Punto tangente
syms x y;

%Calcular distancia del centro del circulo al robot
dist = sqrt((rx-h)^2 + (ry-k)^2);

if dist <= r
    % disp("is in");
    % Direccion a la que esta el robot respecto al centro del circulo
    dir = atan2((ry-k),(rx-h)) + incremento;
    
    % Punto mas cercano es el radio pero en esa direccion 
    x1 = h + r*cos(dir);
    y1 = k + r*sin(dir);
    % Entonces el robot se debe de dirigir a este punto

else
    % disp("is out")
    %% Solucion numerica Fuera del circulo
    %%%%%%%REVISAR SI ESTA BAJO LAS RECTAS ALPHA: ly = m(ly-x1)+y1
    m = tan(alpha);
    yl = m*(rx - ix) + iy;
    
    % Si el robot esta sobre la linea ir hacia el punto tangente 

    if (ry > yl)
        % disp("is over")
        % % Recta tagente entre robot y P_tangente en circunferencia
        % eq1 = r^2 == (x-h)*(rx-h) + (y-k)*(ry-k);
        % 
        % % Circunferencia
        % eq2 = r^2 == (x-h)^2 + (y-k)^2;
        % 
        % % Puntos tangentes, los dos que hay 
        % [solx,soly] = solve(eq1,eq2);
        % 
        % % Punto Tangente 1, Es el unico que nos interesa
        % x1 = double(solx(1));
        % y1 = double(soly(1));

        x1 = (h*k^2 - h*r^2 + h*rx^2 - 2*h^2*rx + h*ry^2 + r^2*rx + h^3 - 2*h*k*ry - k*r*(h^2 - 2*h*rx + k^2 - 2*k*ry - r^2 + rx^2 + ry^2)^(1/2) + r*ry*(h^2 - 2*h*rx + k^2 - 2*k*ry - r^2 + rx^2 + ry^2)^(1/2))/(h^2 - 2*h*rx + k^2 - 2*k*ry + rx^2 + ry^2)
        y1 = (h^2*k - k*r^2 + k*rx^2 + k*ry^2 - 2*k^2*ry + r^2*ry + k^3 - 2*h*k*rx + h*r*(h^2 - 2*h*rx + k^2 - 2*k*ry - r^2 + rx^2 + ry^2)^(1/2) - r*rx*(h^2 - 2*h*rx + k^2 - 2*k*ry - r^2 + rx^2 + ry^2)^(1/2))/(h^2 - 2*h*rx + k^2 - 2*k*ry + rx^2 + ry^2)
 

        % Graficar
        t = -100:1:100;
        % Formula de recta tangente
        eqtTan1 = (x1-h)*(x-h) + (y1-k)*(y-k) == r^2;
        % Despejo 'y' y sustituyo x por mi vector t, para obtener cada punto
        yt1 = subs(solve(eqtTan1,y),x,t);

        % yx = m*(t - ix) + iy;
        % plot(t,yx)
        % hold on

        % Dibujar tangente1
        plot(t,yt1);
        hold on;

    else
        % disp("is under")
        % Ir directo hacia el punto
        x1 = ix;
        y1 = iy;
    end
end

%% Calculo de la velocidad
% Obtener direccion del vector direccion xD
dir = atan2((y1-ry),(x1-rx));

dirx = cos(dir);
diry = sin(dir);

% U(j,i) = dirx;
% V(j,i) = diry;

% Velocidades maximas
Vmax_x = 25;
Vmax_y = 25;

% Calcular velocidad de control (a la que debe de moverse el robot)
Vx = Vmax_x * dirx;
Vy = Vmax_y * diry;
%     end
% end
% 
% quiver(X,Y,U,V,0.5)
hold on
% disp(Vx)
% disp(Vy)

%% Graficar

% Punto Tangente 2
% x2 = double(solx(2))
% y2 = double(soly(2))
% 
% eqtTan2 = (x2-h)*(x-h) + (y2-k)*(y-k) == r^2;
% yt2 = subs(solve(eqtTan2,y),x,t);
% 
% % Dibujar tangente2
% plot(t,yt2);
% hold on;

% Circulo
theta = linspace(0, 2*pi, 100);
xc = h + r * cos(theta);
yc = k + r * sin(theta);

% Dibujar circulo
plot(xc, yc);
hold on;

% Dibujar el centro de la circunferencia
plot(h, k, 'b*');

% Dibujar Pelota
plot(px, py, 'ro');

% Dibujar Interseccion
plot(ix, iy, 'rx');


axis equal;
grid on;
xlabel('X');
ylabel('Y');

axis([20 120 0 100]);
