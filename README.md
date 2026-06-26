# Controlador Reactivo - FTG

## 1. Descripción del Enfoque

El controlador se basa en el algoritmo clásico **Follow the Gap (FTG)**, el cual permite al vehículo autónomo navegar la pista buscando el espacio libre más grande en los datos del sensor LiDAR. Se implementa lo siguiente:

* **Safety Bubble:** Se dibuja un radio virtual (0.50m) alrededor del punto más cercano detectado. Los rayos dentro de esta burbuja se asumen como obstáculos, obligando al vehículo a mantener un margen dinámico respecto a las paredes, especialmente en los vértices de las curvas.
* **Adaptive Look-Ahead:** El campo de visión del vehículo se recorta dinámicamente en función de la proximidad frontal a una pared. Esto evita que el vehículo intente girar prematuramente al detectar el espacio de la recta posterior antes de cruzar físicamente la curva.
* **Centroide de Profundidad Máxima:** En lugar de apuntar ciegamente al punto más profundo, el algoritmo aísla la "planicie" de profundidad máxima y calcula su centroide geométrico, garantizando salidas de curva centradas.
* **Steering Deadband & EWMA Filter:** Se aplica una "Zona Muerta" de 2.5 grados y un Filtro de Media Móvil Exponencialmente Ponderada. Esto elimina las micro-correcciones y el zigzagueo inducido por el ruido del sensor a altas velocidades.
* **Mapeo Continuo de Velocidad:** La velocidad funciona mediante una interpolación lineal inversa respecto al ángulo de dirección comandado. El auto frena de manera suave al entrar a la curva y acelera a fondo al enderezar el chasis.

## 2. Estructura del Código

El código está encapsulado en un único nodo de ROS2 llamado `ReactiveFollowGap`, el cual procesa la telemetría a través de las siguientes funciones principales:

* **`__init__(self)`:** Inicializa los suscriptores (`/scan`, `/ego_racecar/odom`), el publicador (`/drive`), e instancia los hiperparámetros críticos de competición (velocidades, radio de burbuja, ángulos máximos).
* **`preprocess_lidar(...)`:** Limpia los datos crudos del LiDAR (manejo de NaNs/Inf), recorta el campo de visión a 180° frontales, y aplica el horizonte de visión dinámico.
* **`find_max_gap(...)`:** Evalúa el arreglo de rayos filtrados y retorna los índices que delimitan la secuencia más larga de espacio libre consecutivo.
* **`find_best_point(...)`:** Calcula el centroide geométrico de los índices que representan la profundidad máxima dentro del *gap* seleccionado.
* **`scan_callback(...)`:** Es el pipeline principal a 40Hz. Calcula el horizonte dinámico -> procesa LiDAR -> dibuja burbuja -> encuentra gap -> calcula centroide -> aplica Deadband y suavizado -> calcula velocidad continua -> publica comando.
* **`odom_callback(...)`:** Registra la posición de inicio y calcula la distancia euclidiana en tiempo real para determinar el cruce por meta. Registra los tiempos y detiene el vehículo al completar 10 vueltas.

## 3. Instrucciones de Ejecución

### Prerrequisitos
* ROS2 (Humble).
* Simulador oficial configurado: [F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository).

### Instalación en el Workspace

1. Crea el paquete dentro del directorio `src` de tu espacio de trabajo de ROS2.
   ```bash
   $ cd ~/workspace_name/src
   $ ros2 pkg create --build-type ament_python ftg --dependencies rclpy sensor_msgs nav_msgs ackermann_msgs
   ```

2. Mueve o crea el archivo del script que se encuentra en este repositorio dentro de la estructura del paquete y asígnale permisos de ejecución.
    ```bash
    $ chmod +x ~/workspace_name/src/ftg/ftg/ftg_racer.py
    ```

3. Registra el entry point en el archivo ```setup.py``` del paquete creado.
    ```python
        entry_points={
        'console_scripts': [
            'racer = ftg.ftg_racer:main',
          ,
        },
    ```

4. Compila el espacio de trabajo.
    ```bash
    $ cd ~/workspace_name
    $ colcon build --packages-select ftg
    ```

## Ejecución

1. Lanzar el simulador 
    ```bash
    $ source ~/F1Tenth-Repository/install/setup.bash
    $ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    ```

2. Lanzar el controlador
    ```bash
    $ source ~/workspace_name/install/setup.bash
    $ ros2 run ftg_project racer
    ```



