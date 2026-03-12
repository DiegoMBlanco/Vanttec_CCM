import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np
import osqp
from scipy import sparse
from scipy.signal import cont2discrete

class MotorMPC(Node):

    def __init__(self):
        super().__init__('motor_mpc_node')

        # ==============================
        # 1. Modelo continuo identificado
        # ==============================
        Ac = np.array([
            [-47.1534,  4.9415],
            [  3.7925, -2.0323]
        ])

        Bc = np.array([
            [2.0238],
            [-2.1023]
        ])

        Cc = np.array([[1., 0.]])

        Ts = 0.01  # 10 ms

        # Discretizar
        Ad, Bd, _, _, _ = cont2discrete((Ac, Bc, Cc, 0), Ts)

        self.Ad = sparse.csc_matrix(Ad)
        self.Bd = sparse.csc_matrix(Bd)

        self.nx = 2
        self.nu = 1
        self.N = 15

        # ==============================
        # 2. Costos
        # ==============================
        Q = sparse.diags([50., 0.])
        QN = Q
        R = 2.0 * sparse.eye(1)

        # ==============================
        # 3. Restricciones
        # ==============================
        self.umin = np.array([-5.4])
        self.umax = np.array([5.4])

        self.xmin = np.array([-14.66, -np.inf])
        self.xmax = np.array([14.66,  np.inf])

        # ==============================
        # 4. Estado inicial
        # ==============================
        self.x0 = np.zeros(self.nx)
        self.xr = np.array([4.0, 0.0])  # referencia 100 rad/s

        # ==============================
        # 5. Construcción QP
        # ==============================
        P = sparse.block_diag([
            sparse.kron(sparse.eye(self.N), Q),
            QN,
            sparse.kron(sparse.eye(self.N), R)
        ], format='csc')

        q = np.hstack([
            np.kron(np.ones(self.N), -Q @ self.xr),
            -QN @ self.xr,
            np.zeros(self.N * self.nu)
        ])

        Ax = sparse.kron(sparse.eye(self.N+1), -sparse.eye(self.nx)) + \
             sparse.kron(sparse.eye(self.N+1, k=-1), self.Ad)

        Bu = sparse.kron(
            sparse.vstack([sparse.csc_matrix((1, self.N)), sparse.eye(self.N)]),
            self.Bd
        )

        Aeq = sparse.hstack([Ax, Bu])

        self.leq = np.hstack([-self.x0, np.zeros(self.N*self.nx)])
        self.ueq = self.leq.copy()

        Aineq = sparse.eye((self.N+1)*self.nx + self.N*self.nu)

        self.lineq = np.hstack([
            np.kron(np.ones(self.N+1), self.xmin),
            np.kron(np.ones(self.N), self.umin)
        ])

        self.uineq = np.hstack([
            np.kron(np.ones(self.N+1), self.xmax),
            np.kron(np.ones(self.N), self.umax)
        ])

        A = sparse.vstack([Aeq, Aineq], format='csc')
        l = np.hstack([self.leq, self.lineq])
        u = np.hstack([self.ueq, self.uineq])

        # ==============================
        # 6. OSQP Setup
        # ==============================
        self.prob = osqp.OSQP()
        self.prob.setup(P, q, A, l, u, warm_start=True, verbose=False)

        # ==============================
        # ROS interfaces
        # ==============================
        self.speed_sub = self.create_subscription(
            Float32,
            '/motor_speed',
            self.speed_callback,
            10
        )

        self.duty_pub = self.create_publisher(
            Float32,
            '/motor_duty',
            10
        )

        self.timer = self.create_timer(Ts, self.control_loop)

    # ==================================
    # Recibe velocidad medida
    # ==================================
    def speed_callback(self, msg):
        self.x0[0] = msg.data

    # ==================================
    # Loop MPC
    # ==================================
    def apply_deadzone(self, duty, deadzone=0.3):
        """Mapea [-1,1] saltando la zona muerta del motor."""
        if abs(duty) < 0.01:  # prácticamente cero, apaga el motor
            return 0.0
        elif duty > 0:
            # Mapea (0, 0.9] → [0.3, 0.9]
            return 0.3 + (duty * (0.9 - 0.3) / 0.9)
        else:
            # Mapea [-0.9, 0) → [-0.9, -0.3]
            return -0.3 + (duty * (0.9 - 0.3) / 0.9)
        
    def control_loop(self):

        # Actualizar igualdad dinámica
        self.leq[:self.nx] = -self.x0
        self.ueq[:self.nx] = -self.x0

        # Luego reconstruir vector completo
        l = np.hstack([self.leq, self.lineq])
        u = np.hstack([self.ueq, self.uineq])

        self.prob.update(l=l, u=u)

        res = self.prob.solve()

        if res.info.status != 'solved':
            self.get_logger().warn("OSQP no resolvió")
            return

        ctrl = res.x[-self.N*self.nu:-(self.N-1)*self.nu]

        u_apply = float(ctrl[0])

        # Convertir voltaje a duty cycle
        duty = u_apply / 6.0

        # Blindaje numérico
        duty = float(np.clip(duty, -0.9, 0.9))

        duty = self.apply_deadzone(duty)

        # Ajuste de zona muerta: si duty está cerca de cero, ir al extremo según el signo
        #if -0.3 < duty < 0.3:
            #duty = 0.3 if duty > 0 else -0.3

        msg = Float32()
        msg.data = duty
        self.duty_pub.publish(msg)

      
def main(args=None):
    rclpy.init(args=args)
    node = MotorMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
