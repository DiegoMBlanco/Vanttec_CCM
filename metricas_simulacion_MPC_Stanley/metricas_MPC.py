import os
import numpy as np
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
import math

# --- CONFIGURACIÓN ---
# Ruta a la CARPETA del bag (donde está el archivo .db3 o .mcap y el metadata.yaml)
BAG_PATH = 'rosbag2_2026_05_06-00_56_55_0.db3' 

def get_rosbag_options(path, storage_id='sqlite3'):
    storage_options = rosbag2_py.StorageOptions(uri=path, storage_id=storage_id)
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')
    return storage_options, converter_options

def get_yaw(q):
    return math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

def run_analysis():
    reader = rosbag2_py.SequentialReader()
    storage_options, converter_options = get_rosbag_options(BAG_PATH)
    
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error abriendo el bag: {e}")
        return

    # Diccionarios para almacenar datos
    data = {
        'time': [], 'x': [], 'y': [], 'v': [], 'w': [],
        'cte': [], 'epsi': [], 'path_x': [], 'path_y': []
    }

    # Mapa de tipos de mensaje
    topic_types = {
        '/r1/odom': 'nav_msgs/msg/Odometry',
        '/drawn_plan': 'nav_msgs/msg/Path',
        '/r1/cmd_vel': 'geometry_msgs/msg/Twist',
        '/CTE': 'std_msgs/msg/Float32',
        '/e_psi': 'std_msgs/msg/Float32'
    }

    print("Leyendo mensajes...")
    while reader.has_next():
        (topic, data_raw, t_nanos) = reader.read_next()
        if topic not in topic_types:
            continue

        msg_type = get_message(topic_types[topic])
        msg = deserialize_message(data_raw, msg_type)
        t_sec = t_nanos / 1e9

        if topic == '/r1/odom':
            data['time'].append(t_sec)
            data['x'].append(msg.pose.pose.position.x)
            data['y'].append(msg.pose.pose.position.y)
        elif topic == '/r1/cmd_vel':
            data['v'].append(msg.linear.x)
            data['w'].append(msg.angular.z)
        elif topic == '/CTE':
            data['cte'].append(msg.data)
        elif topic == '/e_psi':
            data['epsi'].append(msg.data)
        elif topic == '/drawn_plan':
            data['path_x'] = [p.pose.position.x for p in msg.poses]
            data['path_y'] = [p.pose.position.y for p in msg.poses]

    # --- CÁLCULO DE ZERO CROSSINGS ---
    def get_zero_crossings(signal):
        sig = np.array(signal)
        # Encuentra donde el signo cambia (positivo a negativo o viceversa)
        return np.where(np.diff(np.sign(sig)))[0]

    zc_cte = get_zero_crossings(data['cte']) if data['cte'] else []

   # --- GENERACIÓN DE GRÁFICAS (REORGANIZADO) ---
    print("Generando gráficas...")
    plt.style.use('ggplot') # Estilo estándar que siempre funciona
    fig = plt.figure(figsize=(16, 12))
    fig.suptitle(f'Métricas de Desempeño MPC - Bag: {os.path.basename(BAG_PATH)}', fontsize=16)

    # 1. Trayectoria (XY) - Ocupa la columna izquierda completa
    ax1 = plt.subplot(3, 2, (1, 3)) 
    if data['path_x']:
        ax1.plot(data['path_x'], data['path_y'], 'r--', label='Trayectoria Deseada', alpha=0.8)
    ax1.plot(data['x'], data['y'], 'b-', label='Trayectoria Seguida (Odom)', linewidth=1.5)
    ax1.set_title('Comparativa de Trayectoria')
    ax1.set_xlabel('X (m)')
    ax1.set_ylabel('Y (m)')
    ax1.legend()
    ax1.grid(True)

    # 2. CTE + Zero Crossings
    ax2 = plt.subplot(3, 2, 2)
    if len(data['cte']) > 0:
        cte_np = np.array(data['cte'])
        ax2.plot(cte_np, 'g-', label='CTE (m)', alpha=0.6)
        ax2.axhline(y=0, color='black', linestyle='-', linewidth=0.8) # Línea de referencia cero
        if len(zc_cte) > 0:
            ax2.scatter(zc_cte, cte_np[zc_cte], color='red', marker='x', s=40, label=f'Zero Crossings ({len(zc_cte)})')
        ax2.set_ylabel('Metros')
    ax2.set_title('Error Lateral (CTE)')
    ax2.legend()
    ax2.grid(True)

    # 3. Heading Error (e_psi) - Centro Derecha
    ax3 = plt.subplot(3, 2, 4)
    if len(data['epsi']) > 0:
        ax3.plot(data['epsi'], 'orange', label='e_psi (rad)')
        ax3.set_ylabel('Radianes')
    ax3.set_title('Error de Orientación (e_psi)')
    ax3.grid(True)

    # 4. Velocidad Lineal - Abajo Izquierda
    ax4 = plt.subplot(3, 2, 5)
    if data['v']:
        ax4.plot(data['v'], 'k-', label='v (m/s)')
    ax4.set_title('Velocidad Lineal (v)')
    ax4.set_xlabel('Muestras')
    ax4.set_ylabel('m/s')
    ax4.grid(True)

    # 5. Control de Dirección (Angular) - Abajo Derecha
    ax5 = plt.subplot(3, 2, 6)
    if data['w']:
        ax5.plot(data['w'], 'purple', label='w (rad/s)')
    ax5.set_title('Control Angular (w)')
    ax5.set_xlabel('Muestras')
    ax5.set_ylabel('rad/s')
    ax5.grid(True)

    plt.tight_layout(rect=[0, 0.03, 1, 0.95])
    plt.show()
if __name__ == '__main__':
    run_analysis()
