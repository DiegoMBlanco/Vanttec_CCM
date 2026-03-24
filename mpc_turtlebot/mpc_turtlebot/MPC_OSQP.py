import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import osqp
from scipy import sparse
from scipy.signal import cont2discrete
from std_srvs.srv import Empty

# Librerías para importar datos de error y métricas de desempeño
import time
import csv


class TurtlebotMPC(Node):

    def __init__(self):
        super().__init__('turtlebot_mpc_node')

        # ==============================
        # 1. Modelo: doble integrador 2D
        # x = [px, vx, py, vy]
        # u = [ax, ay]
        # ==============================
        A1 = np.array([[0., 1.],
                       [0., 0.]])
        B1 = np.array([[0.],
                       [1.]])

        Ac = np.block([
            [A1,              np.zeros((2, 2))],
            [np.zeros((2, 2)), A1             ]
        ])
        Bc = np.block([
            [B1,              np.zeros((2, 1))],
            [np.zeros((2, 1)), B1             ]
        ])
        Cc = np.eye(4)

        self.Ts = 0.05
        Ad, Bd, _, _, _ = cont2discrete((Ac, Bc, Cc, np.zeros((4, 2))), self.Ts)

        self.Ad = sparse.csc_matrix(Ad)
        self.Bd = sparse.csc_matrix(Bd)

        self.nx = 4   # [px, vx, py, vy]
        self.nu = 2   # [ax, ay]
        self.N  = 20

        # Estado de ejecución
        self.executing = False
        self.srv_start = self.create_service(Empty, '/start_execution', self.start_cb)
        self.get_logger().info("Llama /start_execution para comenzar.")

        # ==============================
        # 2. Costos
        # ==============================
        Q  = sparse.diags([200., 1., 200., 1.])
        QN = sparse.diags([200., 1., 200., 1.])
        R  = 1.5 * sparse.eye(self.nu)

        # ==============================
        # 3. Restricciones físicas Turtlebot3
        # ==============================
        self.umin = np.array([-0.4, -0.4])
        self.umax = np.array([ 0.4,  0.4])

        self.xmin = np.array([-np.inf, -0.4, -np.inf, -0.4])
        self.xmax = np.array([ np.inf,  0.4,  np.inf,  0.4])

        # ==============================
        # 4. Estado, path y yaw
        # ==============================
        self.x0         = np.zeros(self.nx)
        self.path        = []
        self.target_idx  = 0
        self.robot_yaw   = 0.0
        self.Q  = Q
        self.QN = QN

        # ==============================
        # 5. Construcción QP
        # ==============================
        P = sparse.block_diag([
            sparse.kron(sparse.eye(self.N), Q),
            QN,
            sparse.kron(sparse.eye(self.N), R)
        ], format='csc')

        q = np.zeros((self.N + 1) * self.nx + self.N * self.nu)

        Ax = sparse.kron(sparse.eye(self.N + 1), -sparse.eye(self.nx)) + \
             sparse.kron(sparse.eye(self.N + 1, k=-1), self.Ad)

        Bu = sparse.kron(
            sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]),
            self.Bd
        )

        Aeq = sparse.hstack([Ax, Bu])

        self.leq = np.hstack([-self.x0, np.zeros(self.N * self.nx)])
        self.ueq = self.leq.copy()

        Aineq = sparse.eye((self.N + 1) * self.nx + self.N * self.nu)

        self.lineq = np.hstack([
            np.kron(np.ones(self.N + 1), self.xmin),
            np.kron(np.ones(self.N),     self.umin)
        ])
        self.uineq = np.hstack([
            np.kron(np.ones(self.N + 1), self.xmax),
            np.kron(np.ones(self.N),     self.umax)
        ])

        A = sparse.vstack([Aeq, Aineq], format='csc')
        l = np.hstack([self.leq, self.lineq])
        u = np.hstack([self.ueq, self.uineq])

        self.prob = osqp.OSQP()
        self.prob.setup(P, q, A, l, u, warm_start=True, verbose=False)

        # ==============================
        # ROS interfaces
        # ==============================
        self.sub_path = self.create_subscription(Path,     "/drawn_plan", self.path_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, "/odom",       self.odom_cb, 20)
        self.pub_cmd  = self.create_publisher(  Twist,     "/cmd_vel",                  10)

        self.timer = self.create_timer(self.Ts, self.control_loop)

        # ==============================
        # Archivo de métricas
        # ==============================
        self.error_file = open('mpc_error_log.csv', mode='w', newline='')
        self.csv_writer = csv.writer(self.error_file)
        self.csv_writer.writerow(['tiempo', 'error_posicion', 'error_yaw'])
        self.start_time = 0.0

        self.get_logger().info("TurtlebotMPC listo, esperando path...")

    # ==================================
    # Callback: path del PathDrawer
    # ==================================
    def path_cb(self, msg: Path):
        self.path = [
            (pose.pose.position.x, pose.pose.position.y)
            for pose in msg.poses
        ]
        self.target_idx = 0
        self.executing = False # nuevo path → esperar /start de nuevo
        self.get_logger().info(f"Path recibido: {len(self.path)} puntos")
        
        
    # ==================================
    # Callback: Inicio del Recorrido
    # ==================================
    def start_cb(self, request, response):

        if not self.path:
            self.get_logger().warn("Sin path. Dibuja uno primero.")
            return response

        self.executing = True
        self.target_idx = 0
        self.start_time = time.time()
        self.get_logger().info("Ejecución iniciada.")
        return response

    # ==================================
    # Callback: odometría
    # Extrae estado [px, vx, py, vy] y yaw
    # ==================================
    def odom_cb(self, msg: Odometry):
        self.x0[0] = msg.pose.pose.position.x
        self.x0[2] = msg.pose.pose.position.y

        # Velocidad del body → world frame
        vx_body = msg.twist.twist.linear.x
        yaw = self.robot_yaw  # ya calculado antes, o calcular aquí primero

        # Extraer yaw primero
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

        # Proyectar al frame del mundo
        self.x0[1] = vx_body * math.cos(self.robot_yaw)  # vx world
        self.x0[3] = vx_body * math.sin(self.robot_yaw)  # vy world

    # ==================================
    # Avanza waypoints mientras el robot
    # está suficientemente cerca (15 cm)
    # ==================================
    def update_target(self):
        if not self.path or self.target_idx >= len(self.path):
            return None

        # Loop while: salta todos los waypoints ya alcanzados
        while self.target_idx < len(self.path) - 1:
            tx, ty = self.path[self.target_idx]
            dist = np.hypot(tx - self.x0[0], ty - self.x0[2])
            if dist < 0.15:
                self.target_idx += 1
                self.get_logger().info(
                    f"Waypoint → {self.target_idx}/{len(self.path)}  ({tx:.2f},{ty:.2f})"
                )
            else:
                break

        tx, ty = self.path[self.target_idx]
        return np.array([tx, 0.0, ty, 0.0])

    # ==================================
    # Loop MPC
    # ==================================
    def control_loop(self):

        if not self.path:
            self.pub_cmd.publish(Twist())
            return
        
        if not self.executing:
            return # espera el /start_execution
        
        xr = self.update_target()
        if xr is None:
            self.pub_cmd.publish(Twist())
            return

        # Actualizar q con referencia actual
        q = np.hstack([
            np.kron(np.ones(self.N), -self.Q @ xr),
            -self.QN @ xr,
            np.zeros(self.N * self.nu)
        ])

        # Actualizar condición inicial
        self.leq[:self.nx] = -self.x0
        self.ueq[:self.nx] = -self.x0

        l = np.hstack([self.leq, self.lineq])
        u = np.hstack([self.ueq, self.uineq])

        self.prob.update(q=q, l=l, u=u)
        res = self.prob.solve()

        if res.info.status != 'solved':
            self.get_logger().warn(f"OSQP: {res.info.status}")
            return

        # Primera acción de control [ax, ay]
        u_opt = res.x[-self.N * self.nu : -(self.N - 1) * self.nu]
        ax, ay = float(u_opt[0]), float(u_opt[1])

        # Velocidades cartesianas predichas (integración 1 paso)
        vx_next = float(np.clip(self.x0[1] + ax * self.Ts, -0.4, 0.4))
        vy_next = float(np.clip(self.x0[3] + ay * self.Ts, -0.4, 0.4))

        # Magnitud de velocidad lineal deseada
        v_mag = math.sqrt(vx_next**2 + vy_next**2)

        # Yaw deseado hacia donde apunta el vector de velocidad
        if v_mag > 0.01:
            desired_yaw = math.atan2(vy_next, vx_next)
        else:
            desired_yaw = self.robot_yaw

        yaw_error = math.atan2(
            math.sin(desired_yaw - self.robot_yaw),
            math.cos(desired_yaw - self.robot_yaw)
        )

        # Reducir velocidad lineal cuando el error de yaw es grande
        YAW_THRESH = 0.3  # ~17°, ajusta a tu gusto
        if abs(yaw_error) > YAW_THRESH:
            v_cmd = 0.0  # girar en sitio primero
        else:
            v_cmd = float(np.clip(v_mag, 0.0, 0.4))

        # Ganancia angular más alta para alinearse rápido
        omega_cmd = float(np.clip(10.0 * yaw_error, -2.5, 2.5))

        twist = Twist()
        twist.linear.x  = v_cmd
        twist.angular.z = omega_cmd
        self.pub_cmd.publish(twist)

        # ==================================
        # Registro de métricas
        # ==================================
        # xr[0] es la X de referencia, xr[2] es la Y de referencia
        error_posicion = math.hypot(xr[0] - self.x0[0], xr[2] - self.x0[2])

        #Guardamos el tiempo transcurrido y ambos errores
        t_actual = time.time() - self.start_time
        self.csv_writer.writerow([t_actual, error_posicion, yaw_error])
        
        self.get_logger().info(
            f"wp={self.target_idx} "
            f"pos=({self.x0[0]:.2f},{self.x0[2]:.2f}) "
            f"ref=({xr[0]:.2f},{xr[2]:.2f}) "
            f"v={twist.linear.x:.3f}  ω={twist.angular.z:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
