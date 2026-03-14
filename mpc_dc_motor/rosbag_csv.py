import numpy as np
import pandas as pd
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore
from pathlib import Path

# ==========================
# CONFIGURACIÓN
# ==========================
bag_path = Path("rosbag2_2026_02_28-21_25_37")  # <-- CAMBIA por tu carpeta
output_csv = "my_validation_STEP.csv"

# ==========================
# LECTURA DEL BAG
# ==========================
typestore = get_typestore(Stores.ROS2_HUMBLE)

pwm_times = []
pwm_vals = []

speed_times = []
speed_vals = []  # ya vienen en rad/s directamente

with AnyReader([bag_path], default_typestore=typestore) as reader:
    
    connections = [
        x for x in reader.connections
        if x.topic in ["/motor_pwm", "/motor_speed"]
    ]

    for connection, timestamp, rawdata in reader.messages(connections=connections):
        msg = reader.deserialize(rawdata, connection.msgtype)

        t = timestamp * 1e-9  # convertir ns → s

        if connection.topic == "/motor_pwm":
            pwm_times.append(t)
            pwm_vals.append(msg.data)

        if connection.topic == "/motor_speed":
            speed_times.append(t)
            speed_vals.append(msg.data)  # rad/s directo, sin conversión

# Convertir a numpy
pwm_times = np.array(pwm_times)
pwm_vals  = np.array(pwm_vals)

speed_times = np.array(speed_times)
speed_vals  = np.array(speed_vals)

# ==========================
# SINCRONIZACIÓN
# ==========================
pwm_interp = np.interp(speed_times, pwm_times, pwm_vals)

t0  = speed_times[0]
t_s = speed_times - t0

# ==========================
# GUARDAR CSV
# ==========================
df = pd.DataFrame({
    "t_s":        t_s,
    "V_V":        pwm_interp,
    "omega_rad_s": speed_vals  # ya en rad/s
})

df.to_csv(output_csv, index=False)

print("====================================")
print("CSV generado:", output_csv)
print("Número de muestras:", len(df))
print("Rango de velocidad: {:.3f} – {:.3f} rad/s".format(speed_vals.min(), speed_vals.max()))
print("Rango de PWM:       {:.2f} – {:.2f}".format(pwm_interp.min(), pwm_interp.max()))
print("====================================")
