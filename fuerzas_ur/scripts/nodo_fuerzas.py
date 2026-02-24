#!/usr/bin/env python3

import argparse
import logging
import sys
sys.path.append('..')
import rtde.rtde as rtde
import rtde.rtde_config as rtde_config
import time
import rospy
from std_msgs.msg import Float32MultiArray
<<<<<<< HEAD
import csv
import os
import json
import numpy as np
from datetime import datetime
from scipy.spatial.transform import Rotation as R


# ---------------------- Configuración ----------------------
OFFSET_FILE = "/home/mru/catkin_ws/src/fuerzas_ur/rtde_data_logs/offset.json"
DATA_DIR = "/home/mru/catkin_ws/src/fuerzas_ur/rtde_data_logs"
g = 9.81  # gravedad

# ---------------------- ROS node setup ----------------------
rospy.init_node("RTDE_NODE")
pub_force = rospy.Publisher('/rtde_data', Float32MultiArray, queue_size=10)
pub_pose = rospy.Publisher('/rtde_pose', Float32MultiArray, queue_size=10)
pub_force_local = rospy.Publisher('/rtde_data_local', Float32MultiArray, queue_size=10)  # tópico fuerzas locales
rate = rospy.Rate(100)

# ---------------------- Argument parser ----------------------
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='192.168.0.80', help='Robot IP')
parser.add_argument('--port', type=int, default=30004, help='RTDE port')
parser.add_argument('--frequency', type=int, default=125, help='Sampling frequency (Hz)')
parser.add_argument('--config', default='record_configuration.xml', help='RTDE config file')
parser.add_argument("--verbose", action="store_true", help="Increase verbosity")
parser.add_argument("--buffered", action="store_true", help="Use buffered receive")
parser.add_argument("--binary", action="store_true", help="Binary receive mode")
parser.add_argument("--calibrate", action="store_true", help="Compute and save new offset")
=======

# node
rospy.init_node("RTDE_NODE")
pub=rospy.Publisher('/rtde_data', Float32MultiArray,queue_size=10)
rate=rospy.Rate(100)


# parameters - include all RTDE target parameters to have access to
parser = argparse.ArgumentParser()
parser.add_argument('--host', default='192.168.0.80',help='name of host to connect to (localhost)')     # set robot IP
parser.add_argument('--port', type=int, default=30004, help='port number (30004)')
parser.add_argument('--samples', type=int, default=0,help='number of samples to record')
parser.add_argument('--frequency', type=int, default=125, help='the sampling frequency in Herz')
parser.add_argument('--config', default='record_configuration.xml', help='data configuration file to use (record_configuration.xml)')
parser.add_argument("--verbose", help="increase output verbosity", action="store_true")
parser.add_argument("--buffered", help="Use buffered receive which doesn't skip data", action="store_true")
parser.add_argument("--binary", help="save the data in binary format", action="store_true")
>>>>>>> a22d542814aed4e01da86732c97ac119cc3b4b60
args = parser.parse_args()

if args.verbose:
    logging.basicConfig(level=logging.INFO)

<<<<<<< HEAD
# ---------------------- RTDE configuration ----------------------
=======
>>>>>>> a22d542814aed4e01da86732c97ac119cc3b4b60
conf = rtde_config.ConfigFile(args.config)
output_names, output_types = conf.get_recipe('out')

con = rtde.RTDE(args.host, args.port)
con.connect()
<<<<<<< HEAD
=======

# settings
>>>>>>> a22d542814aed4e01da86732c97ac119cc3b4b60
con.get_controller_version()
con.send_output_setup(output_names, output_types, frequency=args.frequency)
con.send_start()

<<<<<<< HEAD
# ---------------------- Archivos ----------------------
os.makedirs(DATA_DIR, exist_ok=True)
timestamp_str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
outfile = os.path.join(DATA_DIR, f"rtde_data_{timestamp_str}.csv")
data_buffer = []
local_force_buffer = []  # buffer para fuerzas locales

# ---------------------- Cargar offset ----------------------
offset = np.zeros(6)
if os.path.exists(OFFSET_FILE) and not args.calibrate:
    with open(OFFSET_FILE, 'r') as f:
        offset = np.array(json.load(f))
    print(f"Offset cargado desde {OFFSET_FILE}: {offset}")
else:
    print("No se aplicará offset (modo calibración o archivo inexistente).")

# ---------------------- Grabación ----------------------
with open(outfile, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow([
        "timestamp",
        "Fx", "Fy", "Fz", "Tx", "Ty", "Tz",
        "Fx_local", "Fy_local", "Fz_local", "Tx_local", "Ty_local", "Tz_local",
        "X_pos", "Y_pos", "Z_pos", "RX", "RY", "RZ", "MASS_ESTIMATED_kg"
    ])

    print(f"Grabando datos RTDE en '{outfile}'... Ctrl+C para detener.")

    try:
        while not rospy.is_shutdown():
            time.sleep(0.001)
            state = con.receive_buffered(args.binary) if args.buffered else con.receive(args.binary)
            if state is None:
                continue

            # --- Fuerza y torque
            Fx, Fy, Fz, Tx, Ty, Tz = state.actual_TCP_force
            raw_force = np.array([Fx, Fy, Fz, Tx, Ty, Tz])
            corrected_force = raw_force - offset if not args.calibrate else raw_force

            # --- Posición y orientación del TCP
            X_pos, Y_pos, Z_pos, RX, RY, RZ = state.actual_TCP_pose
            pose = np.array([X_pos, Y_pos, Z_pos, RX, RY, RZ])

            # ---------------------- Fuerzas y torques en sistema local del efector final ----------------------
            r = R.from_rotvec([RX, RY, RZ])
            R_tcp = r.as_matrix()  # matriz de rotación 3x3

            # Transformación: F_local = R^T * F_global
            F_local = R_tcp.T @ corrected_force[:3]
            T_local = R_tcp.T @ corrected_force[3:]
            Fx_local, Fy_local, Fz_local = F_local
            Tx_local, Ty_local, Tz_local = T_local

            # Guardar fuerzas locales en buffer
            local_force_buffer.append(np.concatenate([F_local, T_local]))

            # --- Publicar por ROS
            msg_force = Float32MultiArray(data=corrected_force.tolist())
            msg_pose = Float32MultiArray(data=pose.tolist())
            msg_force_local = Float32MultiArray(data=[Fx_local, Fy_local, Fz_local, Tx_local, Ty_local, Tz_local])
            pub_force.publish(msg_force)
            pub_pose.publish(msg_pose)
            pub_force_local.publish(msg_force_local)

            # --- Masa estimada usando solo Fz
            masa_estimada = abs(corrected_force[2]) / g

            # --- Guardar en CSV
            writer.writerow([
                state.timestamp,
                *corrected_force,   # SRefBase = global
                *F_local, *T_local,  # SRefEnd-Effector = local
                *pose,
                masa_estimada
            ])
            data_buffer.append(np.concatenate([corrected_force, pose]))

            # --- Impresión en consola
            print(f"{state.timestamp:.3f} | "
                  f"SRefBase F/T: {corrected_force[:3]} N / {corrected_force[3:]} Nm | "
                  f"SRefEnd-Effector F/T: {F_local} N / {T_local} Nm | "
                  f"Pose: Pos (m)[{X_pos:.3f}, {Y_pos:.3f}, {Z_pos:.3f}] RotVec (rad)[{RX:.3f}, {RY:.3f}, {RZ:.3f}]| "
                  f"Mass ≈ {masa_estimada:.4f} kg")

            rate.sleep()

    except (KeyboardInterrupt, rtde.RTDEException):
        print("\nInterrumpido por el usuario o pérdida de conexión.")

    finally:
        con.send_pause()
        con.disconnect()

        # ---------------------- Media de datos ----------------------
        if len(data_buffer) > 0:
            data_np = np.array(data_buffer)
            mean_values = np.mean(data_np, axis=0)

            writer.writerow([])
            writer.writerow([
                "MEAN",
                *mean_values[:6],           # Fx,Fy,Fz,Tx,Ty,Tz medias, con el * lo escribo por separado (como X Y Z) sin * lo escribiria como vector [Fx, Fy, Fz...]
                X_pos, Y_pos, Z_pos,        # Pose real SIN media
                RX, RY, RZ,                 # Orientación real SIN media
                mean_values[-1]             # masa media
            ])#para escribir el csv
            masa_estimada_media = abs(mean_values[2])/ g  # Usa el Fz global
            writer.writerow(["MASS_MEAN_kg", masa_estimada_media])

            print("\nMedia Fuerzas (N)/Torques (Nm)(Fx, Fy, Fz, Tx, Ty, Tz) SRefBase:")
            print(mean_values[:6])
            print(f"Pos (m)[{X_pos:.3f}, {Y_pos:.3f}, {Z_pos:.3f}] RotVec (rad)[{RX:.3f}, {RY:.3f}, {RZ:.3f}]| ")
            print(f"Masa media estimada: {masa_estimada_media:.4f} kg")

        # ---------------------- Media fuerzas locales ----------------------
        if len(local_force_buffer) > 0:
            local_np = np.array(local_force_buffer)
            mean_local = np.mean(local_np, axis=0)
            print("\nMedia Fuerzas (N)/Torques (Nm) [Fx,Fy,Fz,Tx,Ty,Tz] SRefEnd-Effector:")
            print(mean_local)

        # ---------------------- Guardar offset si calibración ----------------------
        if args.calibrate:
            with open(OFFSET_FILE, 'w') as f:
                json.dump(mean_values[:6].tolist(), f, indent=4)
            print(f"Nuevo offset guardado en: {OFFSET_FILE}")

        print(f"Datos guardados en: {outfile}")

=======
# initialize variables
X = 0
Y = 0
Z = 0
RX = 0
RY = 0
RZ = 0
# main loop
i = 1
while not rospy.is_shutdown():
    if args.samples > 0 or rospy.is_shutdown():
        keep_running = False
    time.sleep(0.001)
    try:
        if args.buffered:
            state = con.receive_buffered(args.binary)
        else:
            state = con.receive(args.binary)
        if state is not None:
            X,Y,Z,RX,RY,RZ = state.actual_TCP_force
            date_and_time = state.timestamp
            print(str(date_and_time)+" TCP: force ["+str(X)+", "+str(Y)+", "+str(Z)+"] N, torque ["+str(RX)+", "+str(RY)+", "+str(RZ)+"] Nm")     
        if not rospy.is_shutdown():
            pub.publish(data= [X,Y,Z,RX,RY,RZ])
            rate.sleep()   
    except rtde.RTDEException:
        break
    except KeyboardInterrupt:
        break
    
con.send_pause()
con.disconnect()
>>>>>>> a22d542814aed4e01da86732c97ac119cc3b4b60
