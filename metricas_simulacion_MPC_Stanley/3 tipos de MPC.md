# Modelo cinemático de la Bicicleta (Ackermann)

El modelo de la bicicleta tiene 3 grados de libertad $(x, y, \theta)$ y restricciones no holonómicas (no se puede mover lateralmente). Las dos ruedas delanteras y dos ruedas traseras se juntan y consideran como una sola para definir el modelo cinemático. Los controles de entrada corresponden a la aceleración y el ángulo de giro de las ruedas delanteras:

<img width="3987" height="1335" alt="IMG_20260524_052500" src="https://github.com/user-attachments/assets/1a5c92b5-457d-4636-ac3d-da76df9d27d7" />

Los estados son $X, Y, V$. La Velocidad es referente al centro de gravedad. $\Psi$ es el ángulo entre el eje longitudinal del carro y el eje global $\vec{X}$. $\beta$ es el ángulo entre el eje longitudinal del carro y el vector de velocidad. Aparece porque las ruedas giran un ángulo $\delta_f$, pero como el centro de rotación instantáneo no está en $G$, la velocidad en $G$ no apunta exactamente en dirección $\Psi$. Por ello, la dirección real de la velocidad es $\Psi + \beta$. En las ecuaciones se asume que no existe deslizamiento:

1) Proyección sobre el eje global X:

$$\dot{x} = v \cdot \cos(\Psi + \beta)$$

2) Proyección sobre el eje global Y:

$$\dot{y} = v \cdot \sin(\Psi + \beta)$$

3) La aceleración longitudinal controla la velocidad:

$$\dot{v} = u_1$$

La velocidad angular depende de qué tan rápido vaya el carro, de qué tan grande sea $\beta$, y la distancia al eje trasero:

$$\dot{\Psi} = \frac{v}{Lr} \sin(\beta)$$

Por último, tomando en cuenta que las entradas son:
* $u_1 = aceleración$
* $u_2 = \delta_f$

Obtenemos que:

$$\beta = \arctan(\tan(u_2) \cdot \frac{Lr}{Lf + Lr})$$


# MPC Lineal

Es la forma más básica del MPC. Asume que el robot se mueve a una velocidad fija predefinida y que nunca va a cambiar. Aquí el modelo matemático es una línea recta:

$$\begin{bmatrix} \dot{e}_y \\ \dot{e}_\psi \end{bmatrix} = \begin{bmatrix} 0 & V_{cte} \\ 0 & 0 \end{bmatrix} \begin{bmatrix} e_y \\ e_\psi \end{bmatrix} + \begin{bmatrix} 0 \\ \frac{V_{cte}}{L} \end{bmatrix} \delta$$

* Ventajas: Muy rápido y fácild e implementar
* Desventajas: EL modelo deja de ser válido si la velocidad cambia. Además no puede reaccionar a curvas para reducir velocidad


# LTV-MPC

Se trata de un MPC Lineal Variante en el Tiempo. Este modelo reconoce que la velocidad cambia, pero la considera constante durante una delta de tiempo. Este paso lo hace lineal siguiendo el modelo:

$$x_{k+1} = A(v_k)x_k + B(v_k)u_k$$

El modelo busca seguir una velocidad de referencia que es una entrada externa para el optimizador. Para modficiar la velocidad, se utiliza una función que adapta la velocidad dependiendo de la geometría de las curvas que detecta en el path. EL controlador NO modifica esa velocidad. 

* Ventajas: Robusto frente a cambios de velocidad y velocidad media/rápida
* Desventajas: No sabe por qué debe de frenar. No modifica la velocidad por su cuenta.

# NMPC

El optimizador usa ecuaciones trigonométricas (no lineales) considerando a la velocidad como una variable directa. Vamos a tomar solamente el modelo cinemático para la primera versión del NMPC. Para un vehículo de altas velocidades sí es recomendable tomar el modelo dinámico que toma en cosnideración el ángulod e deslizamiento y la fricción. 

$$\dot{x} = v \cdot \cos(\Psi + \beta)$$


$$\dot{y} = v \cdot \sin(\Psi + \beta)$$


$$\dot{v} = u_1$$

La velocidad y el giro están directamente acoplados en el sistema dinámico. La posición cambiará de forma no lineal, y trata de tomar la mejor ruta que tome en consideración las restricciones dinámicas completas del sistema.

* Ventajas: Predice trayectorias feasibles y modifica su velocidad en tiempo real.
* Desventajas: Computacionalmente costoso y sujeto a mínimos locales.

<img width="747" height="312" alt="image" src="https://github.com/user-attachments/assets/f1f88078-199d-463c-b24d-46fe7ea86e0d" />

