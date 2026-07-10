# Controlador Reactivo  usando FTG

## 1. Descripción de Enfoque

El controlador propuesto se basa en el algoritmo clásico **Follow the Gap**, el cual permite al vehículo autónomo navegar la pista buscando el espacio libre más grande en los datos del sensor LiDAR. 

Se implementa lo siguiente:

* **Extracción de región de interés:** Antes de procesar todo el entorno, se extrae un cono de visión de ±15 grados justo frente al chasis. Éste actúa como un sensor de proximidad  que permite al auto saber inmediatamente si tiene la pista libre por delante o si está en la cola de un oponente.
* **Cambio de estados:** En lugar de mantener parámetros estáticos en todo el trayecto, el auto cambia su comportamiento en 2 modos basándose en la lectura del LiDAR:
    - **Modo crucero**: Si un obstáculo frontal está a más de 4.5m, el auto amplía su horizonte de visión. Agranda el cono de visión para alejarse de los muros y reduce la respuesta de dirección del volante.
    - **Modo misil**: Si se detecta un obstáculo a menos de 4.5m, el auto asume que está a punto de rebasar o en una horquilla. Se corta la visión de largo alcance, se encoge el padding y aumenta la respuesta de dirección del volante, lo que permite al chasis del auto rozar los márgenes de la pista.

* **Control cinemático:** A medida que el giro del volante aumenta, se resta velocidad de forma lineal. Si el volante está casi recto, se  satura la velocidad a la máxima.

* **Frenado de mitigación:** Si el vehículo está en *modo misil* , el obstáculo frontal está a menos de 3.5m, y la velocidad es mayor a 6.5, se fuerza temporalmente la velocidad a 6.5. Esto asegura que el auto tenga suficiente tracción y evite el subviraje para ejecutar una maniobra evasiva sin salirse.

* **Filtro paso bajo:** Se calcula una *Media Móvil Exponencial*. 
Esto suaviza los comandos de dirección. El coeficiente ```alpha``` define qué tanto se confía en la nueva lectura vs. la inercia anterior. Un ```alpha``` bajo filtra más el ruido, mientras que un ```alpha``` alto permite giros rápidos.


## 2. Estructura del Código

El código está encapsulado en un único nodo de ROS2 llamado `ReactiveFollowGap`, el cual procesa la telemetría a través de las siguientes funciones:

* **`__init__(self)`:** Inicializa el nodo. Declara y lee los parámetros de configuración. Crea las suscripciones al escáner LiDAR (```/scan```) y a la odometría (```/odom```), y el publicador para los comandos de motor y dirección (```AckermannDriveStamped```). Tambien inicializa variables de estado.

* **`preprocess_lidar(...)`:** Limpia y prepara los datos del LiDAR. Reemplaza valores nulos o infinitos por distancias seguras o ceros y anula las lecturas del LiDAR que están detrás del coche o a los lados extremos.

* **`find_max_gap(...)`:** Busca el arreglo de datos ya procesados del LiDAR para encontrar la secuencia contigua más grande de valores mayores a cero.

* **`find_best_point(...)`:** Una vez encontrada la brecha más grande, esta función busca el punto más alejado dentro de esa brecha.
Si hay varios puntos con la misma profundidad máxima, calcula el promedio de sus índices.

* **`scan_callback(...)`:** Es el pipeline principal, se ejecuta cada vez que llega un nuevo mensaje del LiDAR. El algoritmo es el siguiente: Extraer el cono de visión > Definir parametros de visión, burbuja y ```alpha``` > ```preprocess_lidar``` > Encontrar el punto más cercano y dibuja una burbuja de seguridad a su alrededor > Usar ```find_max_gap``` para encontrar el espacio abierto y ```find_best_point``` para decidir hacia dónde apuntar > Calcular la velocidad > Aplicar filtros > Publicar el mensaje  tipo ```AckermannDriveStamped```

* **`odom_callback(...)`:** Registra la posición de inicio y calcula la distancia en tiempo real para determinar el cruce por meta. Registra los tiempos y detiene el vehículo al completar 10 vueltas.

* **`stop_car(self)`:** Genera y publica un mensaje con velocidad 0.0 y ángulo de dirección 0.0 para detener el vehículo por completo al finalizar la carrera.

## 3. Instrucciones de Ejecución

### Prerrequisitos

* ROS2 Humble.
* Simulador oficial configurado del siguiente repositorio: [F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository).
* Herramienta de construcción ```colcon```.

### Ejecución de parte 1 (sin obstaculos)



1. Cree el paquete dentro del directorio `src` de un espacio de trabajo de ROS2 nuevo o preexistente.
   ```bash
   $ cd ~/workspace_name/src
   $ ros2 pkg create --build-type ament_python ftg --dependencies rclpy sensor_msgs nav_msgs ackermann_msgs
   ```

2. Copie el archivo del script que se encuentra en este repositorio (```FTG_Racer/src/ftg_racer.py```) dentro de la estructura del paquete creado previamente y asígnele permisos de ejecución.
    ```bash
    $ chmod +x ~/workspace_name/src/ftg/ftg/ftg_racer.py
    ```

3. Registre el entry point en el archivo ```setup.py``` del paquete creado.
    ```python
        entry_points={
        'console_scripts': [
            'racer = ftg.ftg_racer:main',
          ,
        },
    ```

4. Compile el espacio de trabajo.
    ```bash
    $ cd ~/workspace_name
    $ colcon build --symlink-install --packages-select ftg
    ```
5. Con el simulador instalado y configurado, copie los archivos ```.png``` y ```.yaml``` del repositorio actual (```FTG_Racer/maps```)  en el directorio de mapas del simulador (```F1Tenth-Repository/src/f1tenth_gym_ros/maps```)

6. Actualice la ruta del mapa a usar en el simulador (``Oschersleben_map``). Para ello, modifique el archivo ```F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml```:
    ```python
    map_path: '/home/user/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Oschersleben_map'
    ```

7. Compile el simulador.
    ```bash
    $ cd ~/F1Tenth-Repository
    $ colcon build
    ```

8. Ejecute el simulador.
    ```bash
    $ source ~/F1Tenth-Repository/install/setup.bash
    $ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    ```

9. El controlador cuenta con parámetros de ejecución. Para el auto principal:

    - `scan_topic`: `/scan`
    - `drive_topic`: `/drive`
    - `odom_topic`: `/ego_racecar/odom`
    - `max_speed`: `10.5` (no subir de 10.5)
    - `is_ego`: `True`

    Ejecute el controlador.

    ```bash
    $ source ~/workspace_name/install/setup.bash
    $ ros2 run ftg racer --ros-args -p scan_topic:=/scan -p drive_topic:=/drive -p odom_topic:=/ego_racecar/odom -p max_speed:=10.5 -p is_ego:=True
    ```


### Ejecución de parte 2 (con obstaculos)

Para continuar con la parte 2 es obligatorio haber completado con éxito la ejecución de la parte 1.

1. Dado que no es posible simular mas de 2 autos de forma simultánea, es necesario modificar los archivos del simulador para eliminar esa restricción. 

    Reemplace el archivo ```F1Tenth-Repository/src/f1tenth_gym_ros/f1tenth_gym_ros/gym_bridge.py``` con el encontrado en este repositorio: ```FTG_Racer/sim/gym_bridge.py``` 

2. Modifique y agregue los siguientes parámetros al archivo ```F1Tenth-Repository/src/f1tenth_gym_ros/config/sim.yaml```:

    ```python
    opp_scan_topic: '/opp_racecar/scan'
    opp_drive_topic: '/opp_racecar/drive' 

    # topics and namespaces for opp2
    opp2_namespace: 'opp2_racecar'
    opp2_odom_topic: 'odom'
    opp2_ego_odom_topic: 'opp2_ego_odom'
    ego_opp2_odom_topic: 'ego_opp2_odom'
    opp2_scan_topic: '/opp2_racecar/scan'
    opp2_drive_topic: '/opp2_racecar/drive'

    # map parameters
    map_path: '/home/user/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Oschersleben_obs_map' 

    # opponent parameters
    num_agent: 3

    # opp 1 starting pose on map
    sx1: -1.0
    sy1: 0.0
    stheta1: 0.0
    
    # opp2 starting pose on map
    sx2: -2.0
    sy2: 0.0
    stheta2: 0.0
    ```
3. Copie el archivo ```FTG_Racer/sim/opp2_racecar.xacro``` en la ruta:  ```F1Tenth-Repository/src/f1tenth_gym_ros/launch```.

4. Reemplace el archivo ```F1Tenth-Repository/src/f1tenth_gym_ros/launch/gym_bridge_launch.py``` con el encontrado en este repositorio: ```FTG_Racer/sim/gym_bridge_launch.py```

5. Recompile el simulador.
    ```bash
    $ cd ~/F1Tenth-Repository
    $ colcon build --packages-select f1tenth_gym_ros
    ```

6. Ejecute el simulador.
    ```bash
    $ source ~/F1Tenth-Repository/install/setup.bash
    $ ros2 launch f1tenth_gym_ros gym_bridge_launch.py
    ```

7. En este punto se espera que RViz muestre la pista con obstaculos estáticos, el auto principal y uno extra. Para agregar el seguno extra a la simulación siga los siguientes pasos:

    - En la GUI de RViz, de click en el boton `Add` abajo a la izquierda.
    - Seleccione _RobotModel_ de las opciones y presione `OK`.
    - Despliegue las opciones del nuevo _RobotModel_ que aparece en la ventana `Displays` a la izquieda.
    - Cambie la propiedad `Description Topic` a ``/opp2_robot_description``. El auto debe aparecer en la pista al hacer este ultimo paso.

8. Ejecute los controladores. Los parámetros de ejecución cambian para los autos extra:

    - `scan_topic`: `/opp_racecar/scan`, `/opp2_racecar/scan`
    - `drive_topic`: `/opp_racecar/drive`, `/opp2_racecar/drive`
    - `odom_topic`: `/opp_racecar/odom`, `/opp2_racecar/odom`
    - `max_speed`: `4.5`, `4.0` (no modificar)
    - `is_ego`: `False`

    Para iniciar la simulación, se debe ejecutar los controladores en 3 terminales distintas. 

    Extra (#1):
    ```bash
    $ source ~/workspace_name/install/setup.bash
    $ ros2 run ftg racer --ros-args -p scan_topic:=/opp_racecar/scan -p drive_topic:=/opp_racecar/drive -p odom_topic:=/opp_racecar/odom -p max_speed:=4.5 -p is_ego:=False
    ```
    Extra (#2):
    ```bash
    $ source ~/workspace_name/install/setup.bash
    $ ros2 run ftg racer --ros-args -p scan_topic:=/opp2_racecar/scan -p drive_topic:=/opp2_racecar/drive -p odom_topic:=/opp2_racecar/odom -p max_speed:=4.0 -p is_ego:=False
    ```
    Principal:
    ```bash
    $ source ~/workspace_name/install/setup.bash
    $ ros2 run ftg racer --ros-args -p scan_topic:=/scan -p drive_topic:=/drive -p odom_topic:=/ego_racecar/odom -p max_speed:=10.5 -p is_ego:=True
    ```
## 4. Demos

* [![Parte 1](https://img.youtube.com)](https://www.youtube.com/watch?v=uvA569BjL0c)
* [![Parte 2](https://img.youtube.com)](https://www.youtube.com/watch?v=9tHNwPfgrQA)





