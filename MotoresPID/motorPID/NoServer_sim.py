import math
from robot import Robot
from Camara import *
import time
import sys


# _________________________AQUI INICIARIA COMO TAL EL ALGORITMO _________________________________________
# Inicio el robot con una posicion inicial
defensa = Robot(pos_x=50, pos_y=50, pos_w=0)
pelota_x, pelota_y = 112.5, 110
zed = iniciar_camara()
runtime_parameters = sl.RuntimeParameters()


if __name__ == '__main__':
    while True:
        # ______________________________POSICION ROBOT______________________________________
        # Actualizar Posicion robot si es que recive del defensa, de momento simulo con entrada de teclado
        # Solicitar la posición y orientación del robot
        robot_input = input("Posición del robot (x,y,w) o 'e': ")
        if robot_input.lower() == 'e':
            break
        elif robot_input.lower() == '':
            pass
        else:
            # Convertir la entrada del robot en coordenadas x, y y orientación w
            robot_x, robot_y, robot_w = map(float, robot_input.split(','))
            defensa.set_pos(robot_x, robot_y, robot_w)

        # ______________________________POSICION PELOTA______________________________________
        ball_input = input("Posición de pelota (x,y) o 'e': ")
        if ball_input.lower() == 'e':
            break
        elif ball_input.lower() == '':
            pass
        else:
            # Convertir la entrada del robot en coordenadas x, y y orientación w
            pelota_x, pelota_y = map(float, ball_input.split(','))

        # ______________________________CONTROL ROBOT______________________________________

        # Aqui iria el algoritmo para decidir quien se mueve, suponiendo que es el defensa
        defensa.moverse = True
        # Arrays para graficar
        pos_x = []
        pos_y = []
        timeline = []
        ang1 = []
        ang2 = []
        # Array para filtrar
        ry_queue = deque(maxlen=5)
        first_time = time.time()

        defensa.velocidades_defensa(pelota_x, pelota_y)
        while defensa.moverse:
            start_time = time.time()  # Tiempo actual

            # Get orientation
            if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
                ry = get_orientacion(zed)
            else:
                ry = 3

            # Filtrar señal para quitar ruido
            ry_filtered = filtro(ry, ry_queue)

            # Llenar arrays para graficar
            pos_x.append(defensa.pos_x)
            pos_y.append(defensa.pos_y)
            ang1.append(math.degrees(ry))
            ang2.append(math.degrees(ry_filtered))

            # Calcular velocidades
            vx, vy, vw = defensa.velocidades_defensa(pelota_x, pelota_y)

            # Actualizar posicion del robot
            defensa.odometria(vx, vy, vw, ry_filtered)

            elapsed_time = time.time() - start_time  # Tiempo transcurrido
            timeline.append(round(time.time() - first_time, 4))
            print(f'robot: {round(defensa.pos_x, 2)}, {round(defensa.pos_y, 2)}, {round(math.degrees(defensa.pos_w), 2)}\nbalon: {pelota_x}, {pelota_y}')
            print(f'veloc: {round(vx, 2)}, {round(vy, 2)}, {round(vw, 2)}')
            #time.sleep(0.1 - elapsed_time)  # Para que se cumpla/ejecute cada 100 ms

        defensa.graficar(pelota_x, pelota_y, pos_x, pos_y)

    zed.close()
    plt.figure()
    plt.plot(timeline, ang1, "b")
    plt.plot(timeline, ang2, "k")
    plt.show()

