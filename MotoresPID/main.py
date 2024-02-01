from motorPID.robot import Robot
from motorPID.pyArduino import *
import time

import pyzed.sl as sl
import torch
import numpy as np 
import matplotlib.pyplot as plt 



# # ________________________COMUNICACION SERIAL_____________________
ts = 0.1  # Tiempo de muestreo
port = '/dev/ttyUSB0'  # COM8 bluethooth
baudRate = 115200
Esp32 = serialArduino(port, baudRate, 3)
Esp32.readSerialStart()  # Inicia lectura de datos


# __________________________CAMARA___________________________________

zed = sl.Camera()
# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.coordinate_units = sl.UNIT.METER
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
init_params.depth_maximum_distance = 50
# Configurar parámetros de YOLOv5
model = torch.hub.load('ultralytics/yolov5', 'custom','/home/agente2/yolov5/runs/train/exp3/exp3/weights/best.pt',force_reload=True)
# Bucle de captura
runtime_parameters = sl.RuntimeParameters()
status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()
mat = sl.Mat()
depth_map = sl.Mat()
point_cloud = sl.Mat()



# _________________________AQUI INICIARIA COMO TAL EL ALGORITMO _________________________________________
# Inicio el robot con una posicion inicial
portero = Robot(pos_x=80.0, pos_y=80.0, pos_w=0.0)  # W debe de estar en rad

defensa = Robot(pos_x=112.5, pos_y=100.0, pos_w=0.0)


if __name__ == '__main__':

    

    while True:

        #____________OBTENER POSICION DEL ROBOT_____________________

        # Actualizar Posicion robot si es que recive del defensa, de momento simulo con entrada de teclado
        # Solicitar la posición y orientación del robot
        robot_input = input("Posición del robot (x,y,w) o 'e': ")
        if robot_input.lower() == 'e':
            sys.exit()
        elif robot_input.lower() == '':
            pass
        else:
            # Convertir la entrada del robot en coordenadas x, y y orientación w
            robot_x, robot_y, robot_w = map(float, robot_input.split(','))
            defensa.set_pos(robot_x, robot_y, robot_w)


        # __________________OBTENER POSICION PELOTA_ YOLO________________________________
            
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
            frame = mat.get_data()

            # Detectar objetos con YOLOv5
            results = model(frame)

            umbral_confianza=0.55
            deteccion_balon = False
            # Procesar resultados
            for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
                if conf < umbral_confianza:
                    continue
                class_id = int (cls)
                object_name = model.names[class_id]

                x_center = int((xyxy[0] + xyxy[2]) / 2)
                y_center = int((xyxy[1] + xyxy[3]) / 2)

                #print("Coordenadas DE YOLO")
                # Obtener las coordenadas 3D en el centro del objeto detectado
                err, point_cloud_value = point_cloud.get_value(x_center, y_center)
                if err == sl.ERROR_CODE.SUCCESS:
                    x, y, z = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]
                    coordinates_info = f"Coordenadas: X={x:.2f}, Y={y:.2f}, Z={z:.2f}"
                    print(f"{object_name}: {coordinates_info}")
                    if object_name == "balon":
                        xb, yb, zb  = x, y, z
                        deteccion_balon = True
                else:
                    print("Coordenadas no disponibles")
        
            if deteccion_balon:
                pelota_x, pelota_y = xb*100.0 + defensa.pos_x, zb*100.0 + defensa.pos_y
            else:
                ball_input = input("Posición de pelota (x,y) o 'e': ")
                if ball_input.lower() == 'e':
                    sys.exit()
                elif ball_input.lower() == '':
                    pass
                else:
                    # Convertir la entrada del robot en coordenadas x, y y orientación w
                    pelota_x, pelota_y = map(float, ball_input.split(','))


        # ________________________CHECAR VALORES DE POSICIONES______________________
        print(f"robot:{defensa.pos_x}, {defensa.pos_y}\n balon: {pelota_x}, {pelota_y}")
        # Aqui iria el algoritmo para decidir quien se mueve, suponiendo que es el defensa


        #____________________MOVER AL ROBOT___________________________
        defensa.moverse = True
        defensa.velocidades_defensa(pelota_x, pelota_y)
        while defensa.moverse:
            start_time = time.time()  # Tiempo actual

            vx, vy, vw = defensa.velocidades_defensa(pelota_x, pelota_y)

            # Aqui se enviaria Vx, vy y vw a arduino
            Esp32.sendData([vx, vy, vw])

            # Leer las velocidades desde arduino
            vxr = Esp32.rawData[0]
            vyr = Esp32.rawData[1]
            vwr = Esp32.rawData[2]

            # Calclular posicion del robot
            defensa.odometria(vxr, vyr, vwr)

            elapsed_time = time.time() - start_time  # Tiempo transcurrido
            #print(elapsed_time)
            time.sleep(ts - elapsed_time)  # Para que se cumpla/ejecute cada 100 ms

    #Esp32.close()
    zed.close()

