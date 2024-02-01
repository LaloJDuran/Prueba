########################################################################
#
# Copyright (c) 2022, STEREOLABS.
#
# All rights reserved.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
########################################################################

import pyzed.sl as sl
import time
import matplotlib.pyplot as plt
import math
from collections import deque


def quaternion_to_euler(w, x, y, z):
    # Calcula el ángulo de roll (rotación alrededor del eje Y)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Calcula el ángulo de pitch (rotación alrededor del eje X)
    sinp = 2 * (w * y - z * x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp) # Usa 90 grados si está fuera del rango
    else:
        pitch = math.asin(sinp)

    # Calcula el ángulo de yaw (rotación alrededor del eje Z)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def iniciar_camara():
    zed = sl.Camera()

    # Create a InitParameters object and set configuration parameters
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.AUTO  # Use HD720 or HD1200 video mode (default fps: 60)
    init_params.coordinate_system = sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP  # Use a right-handed Y-up coordinate system
    init_params.coordinate_units = sl.UNIT.METER  # Set units in meters

    # Open the camera
    err = zed.open(init_params)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Camera Open : " + repr(err) + ". Exit program.")
        exit()

    # Enable positional tracking with default parameters
    py_transform = sl.Transform()  # First create a Transform object for TrackingParameters object
    tracking_parameters = sl.PositionalTrackingParameters(_init_pos=py_transform)
    err = zed.enable_positional_tracking(tracking_parameters)
    if err != sl.ERROR_CODE.SUCCESS:
        print("Enable positional tracking : " + repr(err) + ". Exit program.")
        zed.close()
        exit()

    return zed


def get_orientacion(zed):
    # _____________IMU_____________
    zed_sensors = sl.SensorsData()
    zed.get_sensors_data(zed_sensors, sl.TIME_REFERENCE.IMAGE)
    zed_imu = zed_sensors.get_imu_data()

    # Display the IMU orientation quaternion
    zed_imu_pose = sl.Transform()
    ox = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[0], 3)
    oy = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[1], 3)
    oz = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[2], 3)
    ow = round(zed_imu.get_pose(zed_imu_pose).get_orientation().get()[3], 3)

    roll, pitch, yaw = quaternion_to_euler(ow, ox, oy, oz)

    pitch = round(pitch, 2)

    return pitch


def filtro(value, queue, window_size=5):
    queue.append(value)
    return sum(queue) / len(queue)


def main():
    zed = iniciar_camara()
    runtime_parameters = sl.RuntimeParameters()

    i = 0
    start_time = time.time()
    timeline = []
    ang1 = []
    ang2 = []
    ry_queue = deque(maxlen=5)

    while i < 500:
        last = time.time()
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            ry = get_orientacion(zed)
            ry_filtered = filtro(ry, ry_queue)

            # Calcular el tiempo transcurrido desde el inicio
            elapsed_time = round(time.time() - start_time, 4)

            # Guardar en arrays
            ang1.append(ry)
            ang2.append(ry_filtered)
            timeline.append(elapsed_time)

            i = i + 1
        else:
            ry = 100

        time.sleep(0.1)
        tarda = round(time.time() - last, 4)
        print(f"Orientation: {ry}, time: {tarda}")

    # Close the camera

    zed.close()
    plt.figure()
    plt.plot(timeline, ang1, "b")
    plt.plot(timeline, ang2, "k")
    plt.show()


if __name__ == "__main__":
    main()
