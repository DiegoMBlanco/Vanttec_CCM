import glob
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

def analizar_y_graficar():
    # 1. Encontrar todos los CSVs en la carpeta actual
    archivos = glob.glob('/home/adrian/Vanttec_CCM/mpc_turtlebot/analysis/data/*.csv')
    if not archivos:
        print("No se encontraron archivos CSV en la carpeta.")
        return

    print(f"Analizando {len(archivos)} archivos...")

    tiempos_comunes = np.linspace(0, 100, 1000) # Vector temporal muy largo temporalmente
    max_tiempo_minimo = float('inf') # Para cortar la gráfica donde termina el recorrido más corto

    lista_errores_pos = []
    lista_errores_yaw = []

    # 2. Leer e interpolar cada archivo
    for archivo in archivos:
        df = pd.read_csv(archivo)
        t = df['tiempo'].values
        e_pos = df['error_posicion'].values
        e_yaw = df['error_yaw'].values

        # Encontrar el tiempo máximo más corto entre todos los recorridos
        if t[-1] < max_tiempo_minimo:
            max_tiempo_minimo = t[-1]

        # Crear funciones de interpolación
        f_pos = interp1d(t, e_pos, kind='linear', bounds_error=False, fill_value="extrapolate")
        f_yaw = interp1d(t, e_yaw, kind='linear', bounds_error=False, fill_value="extrapolate")

        lista_errores_pos.append(f_pos)
        lista_errores_yaw.append(f_yaw)

    # 3. Crear el vector de tiempo final basado en el recorrido más corto
    t_comun = np.linspace(0, max_tiempo_minimo, 500)

    # Evaluar todas las funciones interpoladas en el nuevo tiempo común
    matriz_pos = np.array([f(t_comun) for f in lista_errores_pos])
    matriz_yaw = np.array([f(t_comun) for f in lista_errores_yaw])

    # 4. Calcular Promedio, Desviación Estándar y RMSE Global
    mean_pos = np.mean(matriz_pos, axis=0)
    std_pos = np.std(matriz_pos, axis=0)
    
    mean_yaw = np.mean(matriz_yaw, axis=0)
    std_yaw = np.std(matriz_yaw, axis=0)

    # RMSE Global (Promedio del error cuadrático de todas las muestras)
    rmse_pos = np.sqrt(np.mean(matriz_pos**2))
    rmse_yaw = np.sqrt(np.mean(matriz_yaw**2))

    print("-" * 30)
    print("MÉTRICAS GLOBALES DEL MPC:")
    print(f"RMSE Posición: {rmse_pos:.4f} m")
    print(f"RMSE Yaw:      {rmse_yaw:.4f} rad")
    print("-" * 30)

    # 5. Graficar Posición
    plt.figure(figsize=(10, 5))
    plt.plot(t_comun, mean_pos, 'b-', label='Error Promedio de Posición')
    plt.fill_between(t_comun, mean_pos - std_pos, mean_pos + std_pos, color='b', alpha=0.2, label='± 1 Desviación Estándar')
    plt.title('Error de Posición del MPC durante la Trayectoria')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Error Euclidiano (m)')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    plt.savefig('/home/adrian/Vanttec_CCM/mpc_turtlebot/analysis/plots/grafica_error_posicion.png', dpi=300)

    # 6. Graficar Yaw
    plt.figure(figsize=(10, 5))
    plt.plot(t_comun, mean_yaw, 'r-', label='Error Promedio de Yaw')
    plt.fill_between(t_comun, mean_yaw - std_yaw, mean_yaw + std_yaw, color='r', alpha=0.2, label='± 1 Desviación Estándar')
    plt.title('Error de Orientación (Yaw) del MPC durante la Trayectoria')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Error (rad)')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend()
    plt.tight_layout()
    plt.savefig('/home/adrian/Vanttec_CCM/mpc_turtlebot/analysis/plots/grafica_error_yaw.png', dpi=300)

    print("✅ Gráficas generadas exitosamente: 'grafica_error_posicion.png' y 'grafica_error_yaw.png'")

if __name__ == '__main__':
    analizar_y_graficar()