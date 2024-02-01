from robot import Robot
import time
import sys

from socket import AF_INET, socket, SOCK_STREAM
from threading import Thread
import tkinter
import time
#python SERVEREJ1.py 

    
def receive():
    client_socket.send(bytes("DEFENSA", "utf8"))
    """Handles receiving of messages."""
    while True:
        try:
            msg = client_socket.recv(BUFSIZ).decode("utf8")
            msg = msg.strip()
            if msg == "INTERFAZ:COMENZAR":
                hacer()
            elif msg == "INTERFAZ:FIN":  
                sys.exit()
        except OSError:  
            break

    #msg = client_socket.recv(BUFSIZ).decode("utf8")
    
def hacer():
    # _________________________AQUI INICIARIA COMO TAL EL ALGORITMO _________________________________________
    # Inicio el robot con una posicion inicial
    portero = Robot(pos_x=80, pos_y=80, pos_w=0)  # W debe de estar en rad
    defensa = Robot(pos_x=50, pos_y=20, pos_w=0)
        
        
    robot_input = input("Posición del robot (x,y,w) o 'e': ")
    if robot_input.lower() == 'e':
        sys.exit()
    elif robot_input.lower() == '':
        pass
    else:
        # Convertir la entrada del robot en coordenadas x, y y orientación w
        robot_x, robot_y, robot_w = map(float, robot_input.split(','))
        defensa.set_pos(robot_x, robot_y, robot_w)


        ball_input = input("Posición de pelota (x,y) o 'e': ")
        if ball_input.lower() == 'e':
            sys.exit()
        elif ball_input.lower() == '':
            pass
        else:
            # Convertir la entrada del robot en coordenadas x, y y orientación w
            pelota_x, pelota_y = map(float, ball_input.split(','))

        # Aqui iria el algoritmo para decidir quien se mueve, suponiendo que es el defensa
        defensa.moverse = True
        # Arrays para graficar
        pos_x = []
        pos_y = []

        while defensa.moverse:
            start_time = time.time()  # Tiempo actual
            pos_x.append(defensa.pos_x)
            pos_y.append(defensa.pos_y)

            vx, vy, vw = defensa.velocidades_defensa(pelota_x, pelota_y)

            defensa.odometria(vx, vy, vw)
            elapsed_time = time.time() - start_time  # Tiempo transcurrido
            #print(elapsed_time)
            time.sleep(0.1 - elapsed_time)  # Para que se cumpla/ejecute cada 100 ms

        defensa.graficar(pelota_x, pelota_y, pos_x, pos_y)
        a = str(pelota_x)
        client_socket.send(bytes(a, "utf8"))


HOST = '172.20.10.6' # Enter host of the server without inverted commas
PORT = 33000
BUFSIZ = 1024
ADDR = (HOST, PORT)
client_socket = socket(AF_INET, SOCK_STREAM)
client_socket.connect(ADDR)
msg = client_socket.recv(BUFSIZ).decode("utf8")
receive_thread = Thread(target=receive)

receive_thread.start()

