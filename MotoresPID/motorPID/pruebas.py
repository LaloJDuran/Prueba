import numpy as np
import matplotlib.pyplot as plt


# Definición de rangos y matrices
X = np.arange(-120, 125, 5)
Y = np.arange(0, 120, 5)
U = np.zeros((len(Y), len(X)))
V = np.zeros((len(Y), len(X)))

# Definición de parámetros del círculo y posición inicial
r = 15
dx = 0
dy = 10
px = 0
py = 100
ix = px - dx
iy = py - dy

# Calculo del centro de ambos circulos
ux1 = r * np.cos(30 * np.pi / 180)
uy1 = r * np.sin(30 * np.pi / 180)
h1 = ix + ux1
k1 = iy + uy1

ux2 = r * np.cos(150 * np.pi / 180)
uy2 = r * np.sin(150 * np.pi / 180)
h2 = ix + ux2
k2 = iy + uy2

# Cálculos principales
for i in range(len(X)):
    for j in range(len(Y)):
        rx = X[i]
        ry = Y[j]

        if (rx > (ix - r)) and (rx < (ix + r)) and (ry >= iy):
            dirx = 0
            diry = 0
        else:
            if rx > ix:
                h, k = h1, k1
                incremento = -20 * np.pi / 180
                alpha = -45 * np.pi / 180
            elif rx < ix:
                h, k = h2, k2
                incremento = 20 * np.pi / 180
                alpha = 45 * np.pi / 180
            else:
                h, k = px+5, py+5
                alpha = 0
                incremento = 0

            dist = np.sqrt((rx - h) ** 2 + (ry - k) ** 2)

            if dist <= r:
                direction = np.arctan2((ry - k), (rx - h)) + incremento
                x1 = h + r * np.cos(direction)
                y1 = k + r * np.sin(direction)
                rotacion = 0
            else:
                m = np.tan(alpha)
                yl = m * (rx - ix) + iy

                if ry > yl:
                    sqrt_term = np.sqrt(h ** 2 - 2 * h * rx + k ** 2 - 2 * k * ry + rx ** 2 + ry ** 2 - r ** 2)

                    if ix < rx:
                        # Solución 1 para x e y
                        x1 = (h * k ** 2 + h * rx ** 2 - 2 * h ** 2 * rx + h * ry ** 2 - h * r ** 2 + rx * r ** 2 + h ** 3 - 2 * h * k * ry - k * sqrt_term + ry * sqrt_term) / (h ** 2 - 2 * h * rx + k ** 2 - 2 * k * ry + rx ** 2 + ry ** 2)
                        y1 = (h ** 2 * k + k * rx ** 2 + k * ry ** 2 - 2 * k ** 2 * ry - k * r ** 2 + ry * r ** 2 + k ** 3 - 2 * h * k * rx + h * sqrt_term - rx * sqrt_term) / (h ** 2 - 2 * h * rx + k ** 2 - 2 * k * ry + rx ** 2 + ry ** 2)

                        if ry > (iy - 10):
                            rotacion = 50 * np.pi / 180
                        elif ry > (iy - 15):
                            rotacion = 30 * np.pi / 180
                        elif ry > (iy - 20):
                            rotacion = 20 * np.pi / 180
                        elif ry > (iy - 25):
                            rotacion = 10 * np.pi / 180
                        else:
                            rotacion = 0
                    else:
                        x1 = (
                                         h * k ** 2 + h * rx ** 2 - 2 * h ** 2 * rx + h * ry ** 2 - h * r ** 2 + rx * r ** 2 + h ** 3 - 2 * h * k * ry + k * sqrt_term - ry * sqrt_term) / (
                                         h ** 2 - 2 * h * rx + k ** 2 - 2 * k * ry + rx ** 2 + ry ** 2)
                        y1 = (
                                         h ** 2 * k + k * rx ** 2 + k * ry ** 2 - 2 * k ** 2 * ry - k * r ** 2 + ry * r ** 2 + k ** 3 - 2 * h * k * rx - h * sqrt_term + rx * sqrt_term) / (
                                         h ** 2 - 2 * h * rx + k ** 2 - 2 * k * ry + rx ** 2 + ry ** 2)

                        if ry > (iy - 10):
                            rotacion = -50 * np.pi / 180
                        elif ry > (iy - 15):
                            rotacion = -30 * np.pi / 180
                        elif ry > (iy - 20):
                            rotacion = -20 * np.pi / 180
                        elif ry > (iy - 25):
                            rotacion = -10 * np.pi / 180
                        else:
                            rotacion = 0
                else:
                    x1 = ix
                    y1 = iy
                    rotacion = 0

            direction = np.arctan2((y1 - ry), (x1 - rx)) + rotacion
            dirx = round(np.cos(direction), 4)
            diry = round(np.sin(direction), 4)

        U[j, i] = dirx
        V[j, i] = diry

# Configuración de la gráfica
plt.figure(dpi=300)
plt.grid(True, linewidth=0.5, markeredgewidth=0.2)
plt.rcParams['lines.linewidth'] = 0.5
plt.quiver(X, Y, U, V, color='blue', width=0.0015)

# Código para dibujar los círculos y otros elementos gráficos
theta = np.linspace(0, 2*np.pi, 100)
xc1 = h1 + r * np.cos(theta)
yc1 = k1 + r * np.sin(theta)
plt.plot(xc1, yc1, color='orange')
plt.plot(h1, k1, '*k', markersize=2)
xc2 = h2 + r * np.cos(theta)
yc2 = k2 + r * np.sin(theta)
plt.plot(xc2, yc2, color='orange')
plt.plot(h2, k2, '*k', markersize=2)

# Pelota
plt.plot(px, py, 'or')
plt.plot(ix, iy, '^g')

plt.axis('equal')
plt.tick_params(width='0.1')
plt.xlabel('X')
plt.ylabel('Y')
plt.ylim(0, 120)
plt.xlim(-120, 120)

plt.show()
