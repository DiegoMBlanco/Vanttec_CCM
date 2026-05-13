# 1. MPC Lineal (Velocidad Constante)

Es la forma más básica. Asume que el robot se mueve a una velocidad fija predefinida y nunca cambia. El modelo matemático es una línea recta.

Ecuación del modelo:

<img width="295" height="74" alt="image" src="https://github.com/user-attachments/assets/1bf89cf7-49cb-4e18-82b6-f78ec30aface" />


* Ventajas: Extremadamente rápido (microsegundos); muy fácil de implementar.
* Desventajas: Si el robot frena o acelera, el modelo deja de ser válido y el control se vuelve inestable o impreciso. No puede reaccionar a curvas cerradas reduciendo velocidad.

# 2. LTV-MPC (Tu código actual)

Es un MPC Lineal Variante en el Tiempo. Reconoce que la velocidad cambia, pero para mantener la computación simple, "congela" la velocidad actual en cada paso de control y asume que el sistema es lineal solo por ese instante.

Ecuación del modelo (Linealizado en $k$):

$$x_{k+1} = A(v_k)x_k + B(v_k)u_k$$

(Aquí $A$ y $B$ dependen de $v$, por eso las recalculas en get_model_matrices).

* Cómo maneja la velocidad: Como la velocidad es una entrada externa para el optimizador, necesitas una función externa (get_adaptive_v_target) que le diga qué velocidad seguir.
* Ventajas: Muy balanceado. Es robusto frente a cambios de velocidad y sigue siendo muy rápido (apto para procesadores modestos).
* Desventajas: El MPC es "ciego" a por qué debe frenar; simplemente obedece a la función externa. No optimiza el perfil de velocidad, solo lo sigue.

# NMPC (MPC No Lineal)

Aquí no hay "mentiras" matemáticas. El optimizador usa las ecuaciones trigonométricas completas del coche. La velocidad ya no es un parámetro fijo, sino una variable que el MPC manipula libremente.

Ecuación del modelo (No Lineal):

$$\dot{x} = v \cdot \cos(\theta)$$

$$\dot{y} = v \cdot \sin(\theta)$$

$$\dot{\theta} = \frac{v}{L} \tan(\delta)$$

* Cómo maneja la velocidad: La velocidad y el giro están "acoplados" en la misma ecuación. El NMPC entiende que si gira mucho el volante ($\delta$), la posición $(x, y)$ cambiará de forma no lineal.

* Ventajas: Comportamiento humano/natural. Puede frenar antes de una curva porque "predice" que se saldrá de la trayectoria si no lo hace. Maximiza el rendimiento del vehículo.

* Desventajas: Computacionalmente muy costoso. Requiere solvers complejos (como CasADi o IPOPT). Puede sufrir de "no convexidad" (encontrar una solución que no es la mejor).


<img width="717" height="339" alt="image" src="https://github.com/user-attachments/assets/1a42f070-0bbe-427c-be44-a277450063da" />













































































