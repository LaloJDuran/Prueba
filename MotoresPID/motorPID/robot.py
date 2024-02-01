import numpy as np
import matplotlib.pyplot as plt


def cancha():
    # ________________________GRAFICAR CANCHA____________________________________
    plt.figure()
    # Cancha

    # Crear una nueva figura y eje
    fig, ax = plt.subplots()

    # Configurar el color de fondo y los límites del eje
    ax.set_facecolor('green')
    ax.set_xlim(-180, 180)
    ax.set_ylim(0, 225)

    # Ocultar las etiquetas y las líneas de los ejes
    ax.xaxis.set_visible(False)
    ax.yaxis.set_visible(False)
    for spine in ax.spines.values():
        spine.set_visible(False)

    # Definir las líneas a dibujar
    lineas = [
        [(-162.5, 0), (162.5, 0)],
        [(-162.5, 0), (-162.5, 225)],
        [(162.5, 0), (162.5, 225)],
        [(-112.5, 0), (-112.5, 100)],
        [(-112.5, 100), (112.5, 100)],
        [(112.5, 100), (112.5, 0)],
        [(-55, 0), (-55, 50)],
        [(-55, 50), (55, 50)],
        [(55, 50), (55, 0)]
    ]

    # Dibujar cada línea
    for linea in lineas:
        ax.plot([linea[0][0], linea[1][0]], [linea[0][1], linea[1][1]], color='white', linewidth=2)


def calculo_velocidad(x1, y1, robot_x, robot_y, robot_w, rotacion):
    # Calculo de la velocidad
    # Obtener direccion del vector direccion xD
    direction = np.arctan2(y1 - robot_y, x1 - robot_x) + rotacion
    dirx = round(np.cos(direction), 4)
    diry = round(np.sin(direction), 4)

    # robot_w = math.degrees(robot_w)

    # Calculo orientacion
    if robot_w >= 0.15:  # Esta mirando a la izquierda entonces debe girar en sentido horario (+)
        w1 = -1
    elif robot_w <= -0.15:  # Esta mirando a la derecha
        w1 = 1
    else:
        w1 = 0

    # Velocidades maximas
    Vmax_x = 35
    Vmax_y = 35

    # Calculo de la velocidad de control
    Vx = Vmax_x * dirx
    Vy = Vmax_y * diry
    W = w1

    # print(Vx,Vy)

    return Vx, Vy, W


def orientar_robot(robot_w):
    # Calculo orientacion
    if robot_w >= 0.15:  # Esta mirando a la izquierda entonces debe girar en sentido horario (+)
        w1 = -1
        is_oriented = False
    elif robot_w <= -0.15:  # Esta mirando a la derecha
        w1 = 1
        is_oriented = False
    else:
        w1 = 0
        is_oriented = True

    return w1, is_oriented


class Robot:
    def __init__(self, pos_x, pos_y, pos_w):
        # Inicializa la posición
        self.pos_x = pos_x
        self.pos_y = pos_y
        self.pos_w = pos_w

        # Inicializa la y velocidad
        self.vel_x = 0.0
        self.vel_y = 0.0
        self.vel_w = 0.0

        # Tiempo de muestreo
        self.ts = 0.1

        self.moverse = False

    def set_pos(self, rx, ry, rw):
        self.pos_x = rx
        self.pos_y = ry
        self.pos_w = rw

    def odometria(self, vel_x, vel_y, vel_w, orient):
        # Integral numerica orientacion
        self.pos_w = orient
        #self.pos_w = self.pos_w + self.ts * vel_w

        # Modelo cinematico
        xp = vel_x * np.cos(self.pos_w) - vel_y * np.sin(self.pos_w)
        yp = vel_x * np.sin(self.pos_w) + vel_y * np.cos(self.pos_w)

        # Integral numerica
        self.pos_x = self.pos_x + self.ts * xp
        self.pos_y = self.pos_y + self.ts * yp

    def velocidades_defensa(self, pelota_x, pelota_y):
        # Definición de parámetros del círculo y posición inicial
        r = 15  # Radio
        dx = 0  # Interseccion
        dy = 20
        px = pelota_x   # Pelota
        py = pelota_y
        ix = px - dx
        iy = py - dy

        rx = self.pos_x
        ry = self.pos_y
        rw = self.pos_w

        ux1 = r * np.cos(30 * np.pi / 180)
        uy1 = r * np.sin(30 * np.pi / 180)
        h1 = ix + ux1
        k1 = iy + uy1

        ux2 = r * np.cos(150 * np.pi / 180)
        uy2 = r * np.sin(150 * np.pi / 180)
        h2 = ix + ux2
        k2 = iy + uy2

        # Hacer que no se mueva si la pelota esta detras del robot
        if (rx > (ix - r)) and (rx < (ix + r)) and (ry >= iy):
            self.vel_x, self.vel_y = 0, 0

            vw, is_oriented = orientar_robot(rw)
            self.vel_w = vw
            if is_oriented:
                self.moverse = False

            return self.vel_x, self.vel_y, self.vel_w

        if rx > ix:         # Robot a la derecha de la interseccion
            h, k = h1, k1
            incremento = -20 * np.pi / 180
            alpha = -45 * np.pi / 180
        elif rx < ix:       # Robot a la izquerda de la interseccion
            h, k = h2, k2
            incremento = 20 * np.pi / 180
            alpha = 45 * np.pi / 180
        else:               # Robot justo detras de la interseccion
            h, k = px + 5, py + 5
            alpha = 0
            incremento = 0

        dist = np.sqrt((rx - h) ** 2 + (ry - k) ** 2)

        in_circle = dist <= r
        if in_circle:
            direction = np.arctan2((ry - k), (rx - h)) + incremento
            x1 = h + r * np.cos(direction)
            y1 = k + r * np.sin(direction)
            rotacion = 0
        else:
            m = np.tan(alpha)
            yl = m * (rx - ix) + iy

            over_tengent = ry > yl
            if over_tengent:
                sqrt_term = np.sqrt(h ** 2 - 2 * h * rx + k ** 2 - 2 * k * ry + rx ** 2 + ry ** 2 - r ** 2)

                if ix < rx:    # Si esta a la izquierda
                    # Solución 1 para x e y
                    x1 = (
                                     h * k ** 2 + h * rx ** 2 - 2 * h ** 2 * rx + h * ry ** 2 - h * r ** 2 + rx * r ** 2 + h ** 3 - 2 * h * k * ry - k * sqrt_term + ry * sqrt_term) / (
                                     h ** 2 - 2 * h * rx + k ** 2 - 2 * k * ry + rx ** 2 + ry ** 2)
                    y1 = (
                                     h ** 2 * k + k * rx ** 2 + k * ry ** 2 - 2 * k ** 2 * ry - k * r ** 2 + ry * r ** 2 + k ** 3 - 2 * h * k * rx + h * sqrt_term - rx * sqrt_term) / (
                                     h ** 2 - 2 * h * rx + k ** 2 - 2 * k * ry + rx ** 2 + ry ** 2)

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

        # Calculo de la velocidad de control
        self.vel_x, self.vel_y, self.vel_w = calculo_velocidad(x1, y1, rx, ry, rw, rotacion)

        return self.vel_x, self.vel_y, self.vel_w

    def velocidades_portero(self, pelota_x, pelota_y):
        # Siempre va a esta a la misma distancia de la porteria
        y1 = 20

        # Checar posicion de pelota y mantenerla en los limites
        # Si la pelota esta a la izquierda
        if pelota_x < -55:
            pelota_x = -53
        # Si la pelota esta a la derecha
        elif pelota_x > 55:
            pelota_x = 53

        # Si ya esta en posicion debe de detenerse
        if pelota_x - 2 <= self.pos_x <= pelota_x + 2 and self.pos_y <= 20:
            self.moverse = False
            self.vel_x, self.vel_y, self.vel_w = 0, 0, 0

        # Si todavia no esta en posicion debe de moverse
        else:
            self.moverse = True
            x1 = pelota_x
            rotacion = 0
            self.vel_x, self.vel_y, self.vel_w = calculo_velocidad(x1, y1, self.pos_x, self.pos_y, self.pos_w, rotacion)

        return self.vel_x, self.vel_y, self.vel_w

    def graficar(self, pelota_x, pelota_y, pos_x, pos_y):
        plt.close()
        cancha()
        # Robot
        plt.plot(self.pos_x, self.pos_y, "b^", markersize=10)
        plt.plot(pos_x, pos_y, label='Posicion', linestyle=':', color='black')
        plt.plot(pelota_x, pelota_y, 'ro', markersize=5)
        plt.show()


'''
        # Checar si el robot esta dentro del area de portero
        if -55 <= self.pos_x <= 55:  # si esta dentro:

            # Si ya esta en posicion debe de detenerse
            if pelota_x - 2 <= self.pos_x <= pelota_x + 2 and self.pos_y <= 20:
                self.moverse = False
                self.vel_x, self.vel_y, self.vel_w = 0, 0, 0

            # Si todavia no esta en posicion debe de moverse
            else:
                self.moverse = True
                x1 = pelota_x
                self.vel_x, self.vel_y, self.vel_w = calculo_velocidad(x1, y1, self.pos_x, self.pos_y, self.pos_w)

        # Si esta afura ir al x mas cercano dentro del area del portero
        else:
            # Si el robot esta a la izquierda
            if self.pos_x < -55:
                x1 = -50
            # Si el robot esta a la derecha
            elif self.pos_x > 55:
                x1 = 50
            else:
                x1 = self.pos_x

            self.vel_x, self.vel_y, self.vel_w = calculo_velocidad(x1, y1, self.pos_x, self.pos_y, self.pos_w)

        return self.vel_x, self.vel_y, self.vel_w
'''