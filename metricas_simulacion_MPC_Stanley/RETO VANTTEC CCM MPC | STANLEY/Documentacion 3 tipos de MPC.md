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


# Modelo dinámico de la Bicicleta (Ackermann)

El modelo cinemático solo usa geometría, no considera fuerzas, supone no deslizamiento, y funciona bien a baja velocidad. El modelo dinámico usa leyes de Newton para considerar fuerzas laterales de las llantas, modela el desplazamiento, y sirve para velocidades altas y maniobras agresivas:

<img width="3481" height="1453" alt="IMG_20260524_055530_edit_81371252409844" src="https://github.com/user-attachments/assets/e81d9522-1105-4def-b89d-3f26b72d96d2" />

El modelo dinámico tiene tres grados de libertad principales:
1) $V_x$ Velocidad longitudinal
2) $V_y$ Velocidad lateral
3) $\dot{\Psi}$ Velocidad de yaw

Sus ecuaciones son:

1) Dinámica longitudinal:

$$m(\dot{V_x}-\dot(\Psi)V_y) = F_{xf} + F_{xr}$$

Aquí se aplica la Segunda Ley de Newton $F = ma$ que describe la aceleración longitudinal del sistema de referencia del vehículo. El término $-\dot(\Psi)V_y)$ aparece en la ecuación porque estamos en un sistema no inercial (girando con el vehículo) y se considera el efecto coriolis. $F_{xf}$ y $F_{xr}$ son las fuerzas longitudinales de las ruedas frontal y trasera.

2) Dinámica lateral:

$$m(\dot{V_y}-\dot(\Psi)V_x) = F_{yf} + F_{yr}$$

3) Dinámica rotacional:

$$I_z \ddot(\Psi)V_x) = L_f \cdot F_{yf} - L_r \cdot F_{yr}$$

El momento alrededor del eje Z es causado por las fuerzas laterales: momentos de las ruedas frontal y trasera. Las fuerzas de las llantas se calculan con las ecuaciones de dinámica de llantas:

$$F_{xf} = C_{\tau f} \cdot \tau_{xf} \qquad F_{xr} = C_{\tau r} \cdot \tau_{xr}$$

En donde $C_{\tau f}$ es la rigidez longitudinal de la llanta y $\tau_x$ es el deslizamiento longitudinal. Para el caso de Y:

$$F_{yf} = C_{\alpha f} \cdot \alpha_{f} = C_{\alpha f}(\delta_f - \frac{V_y + L_f \dot{\Psi}}{V_x}) $$

$$F_{yr} = - C_{\alpha r} \alpha_r \cdot \alpha_{r} = - C_{\alpha r}(\frac{V_y + L_r \dot{\Psi}}{V_x}) $$

En donde $\delta f$ es el heading hacia donde apunta la rueda delantera y $\frac{V_y + L_f \dot{\Psi}}{V_x}$ es hacia donde se mueve realmente considerando el slip angle.


# Errores

Dada una trayectoria de referencia ($x_{ref}, y_{ref}, \Psi_{ref}$), definimos dos tipos de errores:

1) $e_y$ (Error lateral): Distancia ortogonal a la ruta. Su cambio es la velocidad proyectada en el eje perpendicular. SImplemente reemplazamos los valores del modelo cinemático de la bicicleta:

$$\dot{e}_y = v \cdot \sin(e_\Psi + \beta)$$

2) $e_\Psi$ (Error de orientación) $= \Psi - \Psi_{ref}$. La taza de cambio del ángulo del error (velocidad de cambio del ángulo del error) corresponde con la del modelo cinemático:

$$\dot{e}_ \Psi = \dot{\Psi} - \dot{\Psi}_{ref} = \frac{v}{L_r}\sin(\beta)$$

¿Por qué así? Imagina que que vas en carro por la carretera y quieres seguir la línea del centro del camino. La proyección sobrel el eje global Y (velocidad en Y) mide la velocidad en la que un objeto se mueve hacia el norte sobre la carretera. En nuestro marco de referencia, el eje X punta hacia el norte. La velocidad $e_y$ que define la velocidad lateral depende del ángulo relativo que tenemos con la línea de referencia $e_\Psi$. Si el ángulo es 0°, significa que nuestro vector de velocidad apunta completamente hacia el eje X y la velocidad lateral sería 0 porque $\sin(0°) = 0$.

<img width="928" height="1134" alt="Gemini_Generated_Image_v85llkv85llkv85l" src="https://github.com/user-attachments/assets/6e2e0b90-2be7-4c52-af4a-08959f3e863c" />


Estos errores se utilizan para describir el modelo cinemático del Ackermann en términos del error; lo que nos permite aplicar aproximaciones de Taylor en ecuaciones cuyos estados no dependen de la pose del robot. Por ejemplo, si tomáramos la pose $X, Y, \Psi$ como entradas, tendríamos que estar recalculando las trayectorias desde el punto 0,0 hasta la distancia en que se encuentran y eso es costoso computacionalmente hablando. Eso solo lo va a hacer el NMPC.


# Series exponenciales

Vamos a representar la función $f(x) = \frac{1}{1-X}$ cómo una suma de términos. A esto se le conoce como serie geométrica. Imagina que divides literalmente esa ecuación con el método de la casita:

<img width="4000" height="3000" alt="IMG_20260525_004559" src="https://github.com/user-attachments/assets/83e5dc6d-05d7-49b3-90f1-4d20fee276cc" />

Mientras más términos tomes, mejor será la aproximación. Solo funciona con números pequeños dentro del valor de convergencia. La serie geométrica tiene la siguiente forma:

$$\frac{1}{1-x} = \sum_{n=0}^{\infty} X^{n}$$

## Serie de Taylor

Sea una función de la forma:

$$f(x) = \sum_{n=0}^{\infty} a_n(X-a)^n$$

Es decir, que se puede reescribir de la siguiente manera:

$$f(x) = a_0 + a_1(X-a) + a_2(X-a)² + \cdot \cdot \cdot + a_n[X-a)^n$$

Vamos a aplicar derivadas con reglas de la cadena:

$$f'(x) = a_1 + 2 \cdot a_2(X-a)(1) + 3 \cdot a_3(X-a)²(1) + 4 \cdot a_4(X-a)³ + \cdot \cdot \cdot$$

$$f''(x) = a_2 + (2 \cdot 3) \cdot a_3(X-a)(1) + (3 \cdot 4) \cdot a_4(X-a)²(1) + \cdot \cdot \cdot$$

$$f'''(x) = (2 \cdot 3) \cdot a_3 + (2 \cdot 3 \cdot 4) \cdot a_4(X-a)(1) + \cdot \cdot \cdot$$

Generalizamos el patrón:

$$f^{n}(X) = n!a_n + \cdot \cdot \cdot$$

Si $X = a$, entonces:

$$f'(a) = a_1 \qquad a_2 = \frac{f''(a)}{2!} \qquad a_3 = \frac{f'''(a)}{3!} \qquad a_n = \frac{f^{n}(a)}{n!}$$

Esto es porque en cada término original de $(x-a)$, si $x = a$, entonces $(a-a) = 0) y solo queda el número factorial multiplicando a la $a$ de grado 1.

Por lo tanto, la Serie de Taylor se ve así:

$$f(x) = f(a) + f'(a)(x-a) + \frac{f''(a)}{2!}(x-a)² + ... + \frac{f^{n}(a)}{n!}(x-a)^n$$

Es decir:

$$f(x) = \sum_{n=0}^{\infty} \frac{f^{n}(a)}{n!}(x-a)^n$$


## Serie de Maclaurin

Es el mismo concepto de la Serie de Taylor pero evaluando $x = a = 0$ y de esta manera igualar funciones trigonométricas con la exponencial base, ya que la derivada de la exponencial es ella misma. SU forma es:

$$f(x) = f(0) + f'(0)(x-0) + \frac{f''(0)}{2!}(x-0)² + ... + \frac{f^{n}(0)}{n!}(x-0)^n$$

* Euler:

$$f(x) = e^x \qquad f(0) = 1$$
$$f'(x) = e^x \qquad f'(0) = 1$$
$$f''(x) = e^x \qquad f''(0) = 1$$
$$f^n(x) = e^x \qquad f^n(0) = 1$$

Su forma queda:

$$e^x = 1 + x + \frac{x²}{2!} + \frac{x³}{3!} + ... + \frac{x^n}{n!}$$

* Seno:

$$f(x) = sen(x) \qquad f(0) = 0$$
$$f'(x) = cos(x) \qquad f'(0) = 1$$
$$f''(x) = -sen(x) \qquad f''(0) = 0$$
$$f'''(x) = -cos(x) \qquad f'''(0) = -1$$

Su forma queda:

$$sen(x) = 0 + 1(x) + \frac{0}{2!}x² - \frac{x³}{3!} + \frac{0}{4!}x⁴ -  \frac{x⁵}{5!} ...$$

* Coseno:

$$f(x) = cos(x) \qquad f(0) = 1$$
$$f'(x) = -sen(x) \qquad f'(0) = 0$$
$$f''(x) = -cos(x) \qquad f''(0) = -1$$
$$f'''(x) = sen(x) \qquad f'''(0) = 0$$

Su forma queda:

$$cos(x) = 1 + 0(x) - \frac{x²}{2!} + \frac{0}{3!}x³ - \frac{x⁴}{4!} + ... $$

* Tangente:

$$f(x) = tan(x) \qquad f(0) = 0$$
$$f'(x) = 1+tan²(x) \qquad f'(0) = 1$$
$$f''(x) = 2tan(x) + 2tan³(x) \qquad f''(0) = 0$$
$$f'''(x) = 2 + 8tan²(x) + 6tan⁴(x) \qquad f'''(0) = 2$$

Su forma queda:

$$tan(x) = 0 + 1x + \frac{0}{2!}x^2 + \frac{2}{3!}x^3 + \frac{0}{4!}x^4 + \frac{16}{5!}x^5 + ...$$


* Arcotangente:

$$f(0) = 0$$
$$f'(0) = 1$$
$$f''(0) = 0$$
$$f'''(0) = -2$$

Su forma queda:

$$\arctan(x) = 0 + 1x + \frac{0}{2!}x^2 + \frac{-2}{3!}x^3 + \frac{0}{4!}x^4 + \frac{24}{5!}x^5 + ...$$


# MPC Lineal

Es la forma más básica del MPC. Asume que el robot se mueve a una velocidad fija predefinida y que nunca va a cambiar. Aquí el modelo matemático es una línea recta que depende de los estados de error.

### Definición del sistema
* Vector de estados:

$$x = \begin{bmatrix} e_y \\ e_\Psi \end{bmatrix}$$

* Vector de entradas:

$$u = \begin{bmatrix} u_2 \end{bmatrix} = \begin{bmatrix} \delta_f \end{bmatrix}$$

Debido a que la velocidad no es variable cambiante, la aceleración no tiene efecto sobre la velocidad. El primer paso es recordar cuales son las ecuaciones que describen nuestros estados:

$$\beta = \arctan\left(\tan(u_2) \cdot \frac{L_r}{L_r + L_f}\right)$$
$$\dot{e}_\Psi = \frac{v}{L_r} \sin(\beta)$$
$$\dot{e}_y = v \cdot \sin(e_\Psi + \beta)$$

Estas ecuaciones son no lineales ya que poseen funciones trigonométricas. Vamos a linearizar aplicando la Serie de Maclaurin a cada función (seno, tangente, y arcotangente) y vamos a tomar solamente la primera derivada que representa nuestro término lineal. Los demás términos ya contienen un exponente mayor a 1:

1) $\sin(\alpha) \approx 0 + 1 \cdot (\alpha - 0) = \alpha$
2) $\tan(\alpha) \approx 0 + 1 \cdot (\alpha - 0) = \alpha$
3) $\arctan(\alpha) \approx 0 + 1 \cdot (\alpha - 0) = \alpha$

Por lo tanto, las ecuaciones linealizadas son:

1) Beta:

$$\beta \approx u_2 \cdot \frac{L_r}{L_r + L_f}$$
   
2) Error de orientación:
   
$$\dot{e}_\Psi \approx \frac{V_{fixed}}{L_r + L_f} u_2$$

3) Error lateral:
   
$$\dot{e}_y \approx V_{fixed} e_\Psi + V_{fixed} \frac{L_r}{L_r + L_f} u_2$$


El siguiente paso es armar la ecuación cinemática linearizada que describa nuestro sistema de error con la forma:

$$\dot{x} = A_c x + B_c u$$

En nuestro caso, buscar qué valores deben de ir en las matrices A y B para que nos den la ecuaciones linearizadas que acabamos de calcular:

$$
\begin{aligned}
\begin{bmatrix}
\dot{e}_y \\
\dot{e}_\Psi
\end{bmatrix}
&=
\begin{bmatrix}
? & ? \\
? & ?
\end{bmatrix}
\begin{bmatrix}
e_y \\
e_\Psi
\end{bmatrix}
+
\begin{bmatrix}
? \\
?
\end{bmatrix}
\begin{bmatrix}
u_2
\end{bmatrix}
\end{aligned}
$$




Las ecuaciones a analizar son:

1)

$$\dot{e}_y = 0 \cdot e_y + V_{fixed} \cdot e_\Psi + V_{fixed} \frac{L_r}{L} \cdot u_2$$

2) 

$$\dot{e}_\Psi = 0 \cdot e_y + 0 \cdot e_\Psi + \frac{V_{fixed}}{L} \cdot u_2$$

Las matrices que cumplen esta ley son:

$$
\begin{aligned}
\begin{bmatrix}
\dot{e}_y \\
\dot{e}_\Psi
\end{bmatrix}
&=
\begin{bmatrix}
0 & V_{fixed} \\
0 & 0
\end{bmatrix}
\begin{bmatrix}
e_y \\
e_\Psi
\end{bmatrix}
+
\begin{bmatrix}
V_{fixed} \frac{L_r}{L_r + L_f} \\
\frac{V_{fixed}}{L_r + L_f}
\end{bmatrix}
\begin{bmatrix}
u_2
\end{bmatrix}
\end{aligned}
$$

* Ventajas: Muy rápido y fácil de implementar
* Desventajas: El modelo deja de ser válido si la velocidad cambia. Además no puede reaccionar a curvas para reducir velocidad


# LTV-MPC

Se trata de un MPC Lineal Variante en el Tiempo. Este modelo reconoce que la velocidad cambia, pero la considera constante durante una delta de tiempo. Este paso lo hace lineal siguiendo el modelo:

$$x_{k+1} = A(v_k)x_k + B(v_k)u_k$$

El modelo busca seguir una velocidad de referencia que es una entrada externa para el optimizador. Para modficiar la velocidad, se utiliza una función que adapta la velocidad dependiendo de la geometría de las curvas que detecta en el path. EL controlador NO modifica esa velocidad. Los estados y entradas son:

$$x = \begin{bmatrix} e_y \\ e_\Psi \\ e_v \end{bmatrix}, \quad u = \begin{bmatrix} u_2 \\ u_1 \end{bmatrix} \to \begin{bmatrix} \delta_f \\ \text{aceleración} \end{bmatrix}$$

Donde el error de velocidad es $e_v = v - v_{ref}$. Las ecuaciones diferenciales continuas de este sistema corresponden con el modelo cinemático de la bicicleta: 

$$\dot{e}_y = v \cdot \sin(e_\Psi + \beta)$$

$$\dot{e}_\Psi = \frac{v}{L_r} \sin(\beta)$$

$$\dot{e}_v = u_1$$

Por lo tanto, las ecuaciones linealizadas son:

1) Beta:

$$\beta \approx u_2 \cdot \frac{L_r}{L_r + L_f}$$
   
2) Error de orientación:
   
$$\dot{e}_\Psi \approx \frac{V_{k}}{L_r + L_f} u_2$$

3) Error lateral:
   
$$\dot{e}_y \approx V_{k} e_\Psi + V_{k} \frac{L_r}{L_r + L_f} u_2$$

4) Error de velocidad:

$$\dot{e}_v = u_1$$


Las matrices que satisfacen estas ecuaciones son:

$$\begin{aligned}
\begin{bmatrix}
\dot{e}_y \\
\dot{e}_\Psi \\
\dot{e}_v
\end{bmatrix}
&=
\begin{bmatrix}
0 & v_k & 0 \\
0 & 0 & 0 \\
0 & 0 & 0
\end{bmatrix}
\begin{bmatrix}
e_y \\
e_\Psi \\
e_v
\end{bmatrix}
+
\begin{bmatrix}
v_k \frac{L_r}{L_r + L_f} & 0 \\
\frac{v_k}{L_r + L_f} & 0 \\
0 & 1
\end{bmatrix}
\begin{bmatrix}
u_2 \\
u_1
\end{bmatrix}
\end{aligned}$$


* Ventajas: Robusto frente a cambios de velocidad y velocidad media/rápida
* Desventajas: No sabe por qué debe de frenar. No modifica la velocidad por su cuenta.

# NMPC

El optimizador usa ecuaciones trigonométricas (no lineales) considerando a la velocidad como una variable directa. Vamos a tomar solamente el modelo cinemático para la primera versión del NMPC. Para un vehículo de altas velocidades sí es recomendable tomar el modelo dinámico que toma en consideración el ángulo de deslizamiento y la fricción. 

A diferencia del MPC lineal o LTV, el NMPC no necesita calcular "errores" relativos para sus ecuaciones dinámicas. Trabaja directamente en las coordenadas globales del mapa $(x, y)$ y la orientación real del coche $(\Psi)$. El vector de estados es:

$$x = \begin{bmatrix}
x \\
y \\
\Psi \\
v
\end{bmatrix}$$

Y los controles son:

$$u = \begin{bmatrix} u_2 \\ u_1 \end{bmatrix} \to \begin{bmatrix} \delta_f \\ \text{aceleración} \end{bmatrix}$$

Se toma directamente el modelo cinemático no lineal de la bicicleta:

$$\begin{aligned}
\begin{bmatrix}
\dot{x} \\
\dot{y} \\
\dot{\Psi} \\
\dot{v}
\end{bmatrix}
&=
\begin{bmatrix}
v \cdot \cos(\Psi + \beta) \\
v \cdot \sin(\Psi + \beta) \\
\frac{v}{L_r} \sin(\beta) \\
u_1
\end{bmatrix}
\end{aligned}$$

En donde Beta mantiene su no linealidad:

$$\beta = \arctan\left(\tan(u_2) \cdot \frac{L_r}{L}\right)$$

La velocidad y el giro están directamente acoplados en el sistema dinámico. La posición cambiará de forma no lineal, y trata de tomar la mejor ruta que tome en consideración las restricciones dinámicas completas del sistema. El error se calcula dentro de la función de costo:

$$J = \sum_{k=0}^{N} \left( Q_x(x_k - x_{ref,k})^2 + Q_y(y_k - y_{ref,k})^2 + Q_\Psi(\Psi_k - \Psi_{ref,k})^2 + Q_v(v_k - v_{ref,k})^2 \right) + costo terminal$$

A diferencia de las dos versiones anteriores, el NMPC no se puede resolver con OSQP. Se puede utilizar Casadi el cual realiza aproximaciones jacobianas dinámicas.

* Ventajas: Predice trayectorias feasibles y modifica su velocidad en tiempo real.
* Desventajas: Computacionalmente costoso y sujeto a mínimos locales.

<img width="747" height="312" alt="image" src="https://github.com/user-attachments/assets/f1f88078-199d-463c-b24d-46fe7ea86e0d" />

