# Lectura-de-fuerzas-y-pares-UR3e
## Descripción 
Este paquete de ROS permite acceder en tiempo real a la estimación de fuerzas y torques en el efector final del robot colaborativo UR3e a través de su interfaz RTDE (Real-Time Data Exchange).

El nodo principal, implementado en Python, se conecta periódicamente al controlador del robot para leer los datos de fuerza y torque calculados internamente, y los publica en un tópico de ROS para su posterior procesamiento o almacenamiento.

En concreto, la información se publica en el tópico:

- **/rtde_data**: vector con `[FX, FY, FZ, TX, TY, TZ]` expresados en Newton [N] y Newton-metro [Nm], correspondientes a las fuerzas y torques estimados en el TCP (Tool Center Point).

## Estructura del Proyecto
El repositorio contiene el paquete de ROS **fuerzas_ur**, cuya organización es la siguiente:

- **scripts/**: Contiene los scripts de Python relacionados con la lectura de fuerzas y torques del UR3e:
  - `nodo_fuerzas.py`: Nodo ROS que lee las fuerzas y torques del UR3e y los publica en el tópico `/rtde_data`.
  - `record_configuration.xml`: Archivo de configuración de RTDE.
  - `rtde/`: Carpeta que contiene la librería oficial de Universal Robots para Python, que permite conectarse al UR3e mediante RTDE y leer datos en tiempo real.
    
- **src/**: Carpeta destinada al código fuente en C++ (actualmente vacía, ya que el paquete utiliza únicamente Python).

- **package.xml**: Archivo de metadatos del paquete, donde se especifican las dependencias (`rospy`, `std_msgs`).  

- **CMakeLists.txt**: Archivo de configuración utilizado por catkin para la compilación e instalación del paquete.  

## Requisitos Previos
Antes de ejecutar el sistema, asegúrese de cumplir con los siguientes requisitos:
- Conectar el robot colaborativo UR3e y comprobar que está en modo 'Remote' *(ver esquina superior izquierda del Teach Pendant)*
- Encender y configurar los siguientes equipos:
  - Equipo 'Gauss'
  - Equipo 'Master'

## Configuración de Red
Es necesario comprobar y establecer las siguientes direcciones IP:
- **Robot UR3e**: `192.168.0.80`
- **Equipo 'Gauss'**: `192.168.0.115`
- **Equipo 'Master'**: `192.168.0.114` *(comprobar, esta IP puede variar)*

## Pasos de Ejecución
1. Iniciar `roscore` en el equipo Master:
   ```bash
   roscore
   ```
2. Lanzar el launcher del robot UR3e desde el equipo Master:
     ```bash
   roslaunch ur_robot_driver ur3e_bringup.launch robot_ip:=192.168.0.80
   ```
  *(donde 192.168.0.80 corresponde a la IP del UR3e)*
  
3. Lanzar el script 'nodo_fuerzas.py' desde el equipo 'Gauss':
   ```bash
   cd catkin_ws
   source devel/setup.bash
   rosrun fuerzas_ur nodo_fuerzas.py --host 192.168.0.80 --config src/fuerzas/ur/scripts/record_configuration.xml
   ```
   *(donde 192.168.0.80 corresponde a la IP del UR3e, y `~/catkin_ws/src/fuerzas_ur/scripts/record_configuration.xml` es la ruta al archivo de configuración RTDE)*
4. Una vez lanzado el script, se comprueba que el tópico `rtde_data` está publicando correctamente la información sobre las fuerzas y pares del TCP del robot:
   ```bash
   rostopic echo /rtde_data
   ```


