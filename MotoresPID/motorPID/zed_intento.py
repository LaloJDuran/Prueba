import cv2
import pyzed.sl as sl
import torch
import numpy as np 
import matplotlib.pyplot as plt 
import time


# Inicializar y configurar la cámara ZED
zed = sl.Camera()

# Create a InitParameters object and set configuration parameters
init_params = sl.InitParameters()
init_params.coordinate_units = sl.UNIT.METER
init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # QUALITY
init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
init_params.depth_maximum_distance = 50


# Configurar parámetros de YOLOv5
model = torch.hub.load('ultralytics/yolov5', 'custom','/home/agente2/yolov5/runs/train/exp3/weights/best.pt',force_reload=True)

# Bucle de captura
runtime_parameters = sl.RuntimeParameters()

status = zed.open(init_params)

if status != sl.ERROR_CODE.SUCCESS:
    print(repr(status))
    exit()

mat = sl.Mat()
depth_map = sl.Mat()
point_cloud = sl.Mat()


try:
    while True:
        inicio = time.time()
        # Obtener imagen de la cámara ZED
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(mat, sl.VIEW.LEFT)
            zed.retrieve_measure(depth_map, sl.MEASURE.DEPTH)
            zed.retrieve_measure(point_cloud, sl.MEASURE.XYZ)
            frame = mat.get_data()

            # Detectar objetos con YOLOv5
            results = model(frame)

            umbral_confianza=0.5
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

                # Obtener las coordenadas 3D en el centro del objeto detectado
                err, point_cloud_value = point_cloud.get_value(x_center, y_center)
                if err == sl.ERROR_CODE.SUCCESS:
                    x, y, z = point_cloud_value[0], point_cloud_value[1], point_cloud_value[2]
                    coordinates_info = f"Coordenadas: X={x:.2f}, Y={y:.2f}, Z={z:.2f}"
                    print(f"{object_name}: {coordinates_info}")
                    #print("Coordenadas",x, y, z )
                    

                else:
                    coordinates_info = "Coordenadas no disponibles"

                # Dibujar cuadro delimitador, texto de distancia y coordenadas
                cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (255, 0, 0), 2)
                cv2.circle(frame, (x_center, y_center), 5, (0,0,255), 2)
                cv2.putText(frame, distance_info, (x_center, y_center + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                cv2.putText(frame, coordinates_info, (x_center, y_center - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            fin =time.time()
            #print(fin-inicio)
            # Mostrar imagen
            cv2.imshow("Frame", frame)

            key = cv2.waitKey(1)
            if key == 27:  # Salir con ESC
                break
                   
except KeyboardInterrupt:
    pass

# Cerrar recursos

zed.close()
cv2.destroyAllWindows()