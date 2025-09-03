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



