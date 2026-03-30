#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
import numpy as np
import osqp
from scipy import sparse
from std_srvs.srv import Empty
import time


class AckermannMPC(Node):

    def __init__(self):
        super().__init__('ackermann_mpc_node')

        # ==============================
        # Parámetros del vehículo
        # ==============================
        self.L  = 1.511421
        self.Ts = 0.05
        self.N  = 15

        self.nx = 4  # [px, py, yaw, v]
        self.nu = 2  # [a, delta]

        # ==============================
        # Costos de estado
        # Q = [px, py, yaw, v]
        # ==============================
        self.Q  = sparse.diags([100., 100., 50., 100.])
        self.QN = self.Q

        # Penalización de delta en tasa de cambio
        self.Rd = sparse.diags([0.5, 50.0])  # aumentar peso Δdelta

        # Penalización absoluta de delta
        self.R  = sparse.diags([0.5, 50.0])  # balancear con Rd

        # ==============================
        # Restricciones
        # ==============================
        max_steer = 0.50   # rad (28.6°), margen bajo límite URDF de 0.6 rad
        self.umin = np.array([-2.0, -max_steer])
        self.umax = np.array([ 2.0,  max_steer])

        self.xmin = np.array([-np.inf, -np.inf, -np.inf, 0.0])
        self.xmax = np.array([ np.inf,  np.inf,  np.inf, 1.0])

        # ==============================
        # Estado
        # ==============================
        self.x0     = np.zeros(self.nx)
        self.u_prev = np.array([0.0, 0.0])

        self.path       = []
        self.target_idx = 0
        self.executing  = False
        self.start_time = 0.0

        # Restricciones de desigualdad sobre x y u (fijas)
        self.lineq = np.hstack([
            np.kron(np.ones(self.N + 1), self.xmin),
            np.kron(np.ones(self.N),     self.umin)
        ])
        self.uineq = np.hstack([
            np.kron(np.ones(self.N + 1), self.xmax),
            np.kron(np.ones(self.N),     self.umax)
        ])

        # ==============================
        # ROS interfaces
        # ==============================
        self.sub_path = self.create_subscription(
            Path, "/drawn_plan", self.path_cb, 10)
        self.sub_odom = self.create_subscription(
            Odometry, "/r1/odom", self.odom_cb, 20)
        self.pub_cmd  = self.create_publisher(Twist, "/r1/cmd_vel", 10)

        self.srv_start = self.create_service(Empty, '/start_execution', self.start_cb)
        self.timer     = self.create_timer(self.Ts, self.control_loop)

        self.get_logger().info("Ackermann MPC listo. Llama /start_execution para comenzar.")

    # ------------------------------------------------------------------
    # Callbacks ROS
    # ------------------------------------------------------------------
    def path_cb(self, msg: Path):
        self.path       = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.target_idx = 0
        self.executing  = False
        self.get_logger().info(f"Path recibido: {len(self.path)} puntos")

    def start_cb(self, req, res):
        if not self.path:
            self.get_logger().warn("Sin path. Dibuja uno primero.")
            return res
        self.executing  = True
        self.target_idx = 0
        self.start_time = time.time()
        self.u_prev     = np.array([0.0, 0.0])
        self.get_logger().info("Ejecución iniciada.")
        return res

    def odom_cb(self, msg: Odometry):
        self.x0[0] = msg.pose.pose.position.x
        self.x0[1] = msg.pose.pose.position.y

        q    = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.x0[2] = math.atan2(siny, cosy)
        self.x0[3] = msg.twist.twist.linear.x

    # ------------------------------------------------------------------
    # Avanza waypoints. Devuelve None cuando se alcanza el punto final.
    # ------------------------------------------------------------------
    def update_target(self):
        if not self.path:
            return None

        # Condición de parada: llegó al último waypoint
        if self.target_idx >= len(self.path) - 1:
            tx, ty = self.path[-1]
            dist_final = np.hypot(tx - self.x0[0], ty - self.x0[1])
            if dist_final < 0.20:
                self.get_logger().info("¡Destino alcanzado! Deteniendo.")
                self.executing = False
                return None

        while self.target_idx < len(self.path) - 1:
            tx, ty = self.path[self.target_idx]
            if np.hypot(tx - self.x0[0], ty - self.x0[1]) < 0.15:
                self.target_idx += 1
                self.get_logger().info(
                    f"Waypoint → {self.target_idx}/{len(self.path)}"
                )
            else:
                break

        tx, ty = self.path[self.target_idx]

        dx   = tx - self.x0[0]
        dy   = ty - self.x0[1]
        dist = math.hypot(dx, dy)

        yaw_ref = math.atan2(dy, dx) if dist > 0.05 else self.x0[2]

        yaw_error = abs(math.atan2(
            math.sin(yaw_ref - self.x0[2]),
            math.cos(yaw_ref - self.x0[2])
        ))
        v_ref = max(0.2, 1.0 - 1.5 * yaw_error)

        return np.array([tx, ty, yaw_ref, v_ref])

    # ------------------------------------------------------------------
    # Linealización Ackermann
    # ------------------------------------------------------------------
    def linearize_model(self):
        _, _, theta, v_real = self.x0
        _, delta = self.u_prev
        v = max(v_real, 0.05)

        Ad       = np.eye(4)
        Ad[0, 2] = -v * math.sin(theta) * self.Ts
        Ad[0, 3] =  math.cos(theta)     * self.Ts
        Ad[1, 2] =  v * math.cos(theta) * self.Ts
        Ad[1, 3] =  math.sin(theta)     * self.Ts
        Ad[2, 3] = (1.0 / self.L) * math.tan(delta) * self.Ts

        Bd       = np.zeros((4, 2))
        Bd[2, 1] = v / (self.L * math.cos(delta) ** 2) * self.Ts
        Bd[3, 0] = self.Ts

        return Ad, Bd

    # ------------------------------------------------------------------
    # QP con penalización en tasa de cambio de u (Δu).
    #
    # Formulación extendida:
    #   min  Σ (x-xr)'Q(x-xr) + u'Ru + Δu'Rd Δu
    #
    # Δu_k = u_k - u_{k-1}, con u_{-1} = u_prev (aplicado en iteración anterior)
    #
    # Esto estabiliza delta evitando saltos bruscos entre iteraciones,
    # eliminando la oscilación creciente del LMPC sin penalización de tasa.
    # ------------------------------------------------------------------
    def solve_mpc(self, xr, Ad, Bd):
        Ad_sp = sparse.csc_matrix(Ad)
        Bd_sp = sparse.csc_matrix(Bd)

        # --- Matrices dinámicas ---
        Ax = (sparse.kron(sparse.eye(self.N + 1), -sparse.eye(self.nx)) +
              sparse.kron(sparse.eye(self.N + 1, k=-1), Ad_sp))
        Bu = sparse.kron(
            sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]),
            Bd_sp
        )
        Aeq   = sparse.hstack([Ax, Bu])
        Aineq = sparse.eye((self.N + 1) * self.nx + self.N * self.nu)

        A = sparse.vstack([Aeq, Aineq], format='csc')

        # --- Hessiano con término de tasa de cambio Rd ---
        # P_u = R*I + Rd * D'D  donde D es la matriz de diferencias hacia adelante
        # D = tridiag(-I, I) de tamaño N×N sobre bloques nu×nu
        e  = np.ones(self.N)
        D  = sparse.diags([e, -e], [0, -1],
                          shape=(self.N, self.N)).toarray()
        D  = sparse.csc_matrix(D)
        Db = sparse.kron(D, sparse.eye(self.nu))          # (N*nu) × (N*nu)

        R_full  = sparse.kron(sparse.eye(self.N), self.R)
        Rd_full = Db.T @ sparse.kron(sparse.eye(self.N), self.Rd) @ Db

        P_x = sparse.block_diag([
            sparse.kron(sparse.eye(self.N), self.Q),
            self.QN
        ])
        P_u = R_full + Rd_full

        P = sparse.block_diag([P_x, P_u], format='csc')

        # --- Vector lineal de costo ---
        # El término Rd genera un offset lineal: -2 * Rd * u_prev en u_0
        # (porque Δu_0 = u_0 - u_prev)
        q_x = np.hstack([
            np.kron(np.ones(self.N), -2.0 * self.Q  @ xr),
            -2.0 * self.QN @ xr,
        ])
        # Offset por u_prev: solo afecta al primer bloque de u
        q_u = np.zeros(self.N * self.nu)
        q_u[:self.nu] -= 2.0 * self.Rd @ self.u_prev

        q_vec = np.hstack([q_x, q_u])

        # --- Bounds ---
        leq = np.hstack([-self.x0, np.zeros(self.N * self.nx)])
        l   = np.hstack([leq,        self.lineq])
        u   = np.hstack([leq.copy(), self.uineq])

        prob = osqp.OSQP()
        prob.setup(P, q_vec, A, l, u,
                   warm_start=True, verbose=False, polish=True,
                   eps_abs=1e-4, eps_rel=1e-4, max_iter=4000)
        return prob.solve()

    # ------------------------------------------------------------------
    # Loop de control MPC
    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.executing or not self.path:
            return

        xr = self.update_target()
        if xr is None:
            # Parada suave
            twist = Twist()
            self.pub_cmd.publish(twist)
            return

        Ad, Bd = self.linearize_model()
        res    = self.solve_mpc(xr, Ad, Bd)

        if res.info.status not in ('solved', 'solved_inaccurate'):
            self.get_logger().warn(f"OSQP: {res.info.status} — manteniendo u_prev")
            a, delta = self.u_prev
        else:
            u_start  = (self.N + 1) * self.nx
            u_opt    = res.x[u_start : u_start + self.nu]
            a, delta = float(u_opt[0]), float(u_opt[1])
            self.u_prev = np.array([a, delta])

        # --- Velocidad lineal ---
        v = float(np.clip(self.x0[3] + a * self.Ts, 0.0, 1.0))

        # --- omega desde delta del MPC (un solo controlador) ---
        omega_raw = v / self.L * math.tan(delta)
        OMEGA_GAIN = 4.0
        omega = float(np.clip(OMEGA_GAIN * omega_raw, -2.0, 2.0))

        twist           = Twist()
        twist.linear.x  = v
        twist.angular.z = omega
        self.pub_cmd.publish(twist)

        yaw_ref   = float(xr[2])
        yaw_error = math.atan2(
            math.sin(yaw_ref - self.x0[2]),
            math.cos(yaw_ref - self.x0[2])
        )

        self.get_logger().info(
            f"wp={self.target_idx}/{len(self.path)} "
            f"pos=({self.x0[0]:.2f},{self.x0[1]:.2f}) "
            f"ref=({xr[0]:.2f},{xr[1]:.2f}) "
            f"v={v:.2f}  ω={omega:.2f}  delta={math.degrees(delta):.2f}°  "
            f"yaw_err={math.degrees(yaw_error):.1f}°"
        )


def main():
    rclpy.init()
    node = AckermannMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()