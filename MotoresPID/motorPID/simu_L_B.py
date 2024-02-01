from robot import Robot
import cv2
import pyzed.sl as sl
import torch
import numpy as np 
import matplotlib.pyplot as plt 


#inicializar camara y cargar pesos YOLO

# Inicializar y configurar la cámara ZED
zed = sl.Camera()

# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.coordinate_units = sl.UNIT.METER
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
init_params.depth_maximum_distance = 50


# Configurar parámetros de YOLOv5
model = torch.hub.load('ultralytics/yolov5', 'custom','/home/agente0/yolov5/runs/train/exp6/weights/best.pt',force_reload=True)

# Bucle de captura
runtime_parameters = sl.RuntimeParameters()

status = zed.open(init_params)

if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()

mat = sl.Mat()
depth_map = sl.Mat()
point_cloud = sl.Mat()


# ________AQUI INICIARIA COMO TAL EL ALGORITMO ______________
# Inicio el robot con una posicion inicial
defensa = Robot(pos_x=80, pos_y=80, pos_w=0)  # W debe de estar en rad
portero = Robot(pos_x=50, pos_y=20, pos_w=0)

if _name_ == '_main_':
    while True:
        # Posicion pelota// Yolo  //de momento simulo con entrada de teclado
        # Solicitar la posición pelota
        # Obtener imagen de la cámara ZED
        print("INICIO DE YOLO")
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
            frame = mat.get_data()

            # Detectar objetos con YOLOv5
            results = model(frame)

            umbral_confianza=0.7
            # Procesar resultados
            for *xyxy, conf, cls in results.xyxy[0].cpu().numpy():
                if conf < umbral_confianza:
                    continue
                class_id = int (cls)
                object_name = model.names[class_id]

                x_center = int((xyxy[0] + xyxy[2]) / 2)
                y_center = int((xyxy[1] + xyxy[3]) / 2)

                # Obtener la profundidad en el centro del objeto detectado
                depth_value = depth_map.get_value(x_center, y_center)[1]

                # Verificar si la profundidad es válida
                if depth_value != sl.ERROR_CODE.SUCCESS:
                    distance_info = f"Distancia: {depth_value:.2f} m"
                    
                    print(f"{object_name}: {distance_info}")
                else:
                    distance_info = "Distancia no disponible"
                print("Coordenadas DE YOLO")
                # Obtener las coordenadas 3D en el centro del objeto detectado
                err, point_cloud_value = point_cloud.get_value(x_center, y_center)
                if err == sl.ERROR_CODE.SUCCESS:
                    x, y, z = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]
                    coordinates_info = f"Coordenadas: X={x:.2f}, Y={y:.2f}, Z={z:.2f}"
                    print(f"{object_name}: {coordinates_info}")
                    if object_name == "balon":
                        xb, yb, zb  = x, y, z
                    
                    #print("Coordenadas",x, y, z )
                    

                else:
                    coordinates_info = "Coordenadas no disponibles"

        #robot_input = input("Posición de pelota (x,y) o 'null': ")
        #if robot_input.lower() != 'null':
            # Convertir la entrada del robot en coordenadas x, y y orientación w
            #pelota_x, pelota_y = map(float, robot_input.split(','))
        #    pelota_x, pelota_y = xb, yb
        #else:
        #    break

        pelota_x, pelota_y = 100.0*xb, 100.0*zb

        # Actualizar Posicion robot si es que recive del portero, de momento simulo con entrada de teclado
        # Solicitar la posición y orientación del robot
        robot_input = input("Posición del robot (x,y,w) o 'null': ")
        if robot_input.lower() != 'null':
            # Convertir la entrada del robot en coordenadas x, y y orientación w
            robot_x, robot_y, robot_w = map(float, robot_input.split(','))
            portero.set_pos(robot_x, robot_y, robot_w)

        # Aqui iria el algoritmo para decidir quien se mueve, suponiendo que es el defensa
        portero.moverse = True
        # Arrays para graficar
        pos_x = []
        pos_y = []

        while portero.moverse:
            pos_x.append(portero.pos_x)
            pos_y.append(portero.pos_y)

            vx, vy, vw = portero.velocidades_portero(pelota_x, pelota_y)

            portero.odometria(vx, vy, vw)
            print(portero.pos_x,portero.pos_y)
            
        #portero.graficar(pelota_x, pelota_y, pos_x, pos_y)