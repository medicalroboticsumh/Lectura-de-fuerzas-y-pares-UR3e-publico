# Lectura-de-fuerzas-y-pares-UR3e
## Descripción 
Este paquete de **ROS** permite acceder en tiempo real a la estimación de fuerzas y torques en el efector final del robot colaborativo **UR3e** a través de su interfaz **RTDE (Real-Time Data Exchange)**.

El nodo principal, implementado en **Python**, se conecta periódicamente al controlador del robot para leer los datos de fuerza y torque calculados internamente, y los publica en un tópico de ROS para su posterior procesamiento o almacenamiento.

En concreto, la información se publica en el tópico:

- **/rtde_data**: vector con `[FX, FY, FZ, TX, TY, TZ]` expresados en **Newton [N]** y **Newton-metro [Nm]**, correspondientes a las fuerzas y torques estimados en el **TCP (Tool Center Point)**.

