# Dynamic Object Detection documentation

El objetivo es implementar un algortimo de detección de objetos dinámicos en un video en movimeinto utilizando YOLO y que devuelva una imagen segmentada que elimine dichos objetos de la misma.

Se utilizan las librerías:

```python
import cv2     #OpenCV
import numpy as np     #manejo de matrices
import torch     #motor que corre la red neuronal
from ultralitycs import YOLO     #librería que contiene los modelos de YOLO ya entrenados
```

Se optó por usar un modelo de YOLO ya entrenado (YOLO v8) con su versión Nano de segmentación. Mas que nada por el tiempo que se tarda en procesar el modelo de clasificación en cada iteración del algortimo.
De igual manera, la versión Nano de YOLO v8 detecta clases básicas con bastante precisión. La mayoría de objetos dinámicos que se va a encontrar el SDV en el campus serán personas, entonces es una gran opción para empezar a experimentar. la función ".to(device)" solo le indica a mi GPU que se va a encargar de correr el modelo:

```python
model = YOLO('yolov8n-seg.pt').to(device)
```

Se utilizó un dataset llamado Common Objects in COntext (COCO), el cual contiene cerca de 80 categorías de objetos ya entrenadas. Sus índices principales son:
* 0: Personas
* 1: Bicicletas
* 2: Autos
* 3: Motocicletas
* 4: Aviones
* 5: Autobuses
* 6: Trenes
* 7: Camiones

```python
DYNAMIC_CLASSES = [0, 1, 2, 3, 5, 7]
```

El modelo tiene dos funciones disponibles:
1) "model.predict()": Detecta una clase, pero no sabe distinguir entre dos objetos de la misma clase.
2) "model.track()": Le asigna un ID único a cada objeto incluyendo los de la misma clase

Se le pasan los parámetros del frame a analizar ("frame"), si deseamos mantener el rastro ("persist"), el umbral de confianza como filtro de seguridad (40%). Nos devuelve una lista ("results") que contiene las coordenadas de cada caja detectada, polígonos de las máscaras, y números de clases.

```python
results = model.track(frame, persist=True, device=0, conf=0.4, verbose=False)
```

Para acceder a ellos se hace mediante el índice [0] ya que es el único frame en ese momento que está siendo analizado. Luego se obtienen las coordenadas de los puntos de los cuadros que rodean un objeto y se guardan en una lista de arreglos llamada "mask":

```python
masks = results[0].masks.xy
```

Luego convertimos la lista de ID's asignados por el modelo a cad objeto en una lista de python con el siguiente comando:

```python
clss = results[0].boxes.cls.cpu().int().tolist()
```

Se recorre un conjunto de datos (los boxes, los ID) cada uno matcheado con su par mediante la función "zip()" en donde si el ID corresponde a una clase a buscar, se rellena su caja de color blanco (255):

```python
for mask_poly, cls in zip(masks, clss):

  if cls in DYNAMIC_CLASSES:
  
    # Rellenar la silueta en la máscara con blanco
    
    points = np.int32([mask_poly])
    
    cv2.fillPoly(slam_mask, points, 255)
```

Posteriormente, se calcula el centroide del objeto utilizando lso momentos aplicados en esa caja:

```python
M = cv2.moments(points)
if M['m00'] != 0:
    cx = int(M['m10'] / M['m00'])
    cy = int(M['m01'] / M['m00'])
    cv2.circle(frame, (cx, cy), 12, (0, 255, 255), 2) 
    cv2.circle(frame, (cx, cy), 6, (255, 255, 255), -1) 
```

