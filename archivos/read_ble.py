#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Nodo de ROS para verificar y validar datos de Beacons BLE utilizando una base de datos Supabase.
El nodo procesa los datos recibidos desde un tópico, verifica la información contra la base de datos, 
detecta errores y genera marcadores visuales en RViz en caso de discrepancias.

Dependencias:
    - ROS (Robot Operating System)
    - Supabase (para la gestión de la base de datos)
    - visualization_msgs (para publicar marcadores)
    - nav_msgs (para recibir datos de odometría)
    - JSON y OS (para manejar datos y archivos locales)
"""

import rospy
from std_msgs.msg import String
from supabase import create_client, Client
import struct
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
import json
import os

class BeaconDataChecker:
    """
    Clase que verifica los datos de los Beacons BLE contra una base de datos en Supabase.
    Maneja errores de ubicación, temperatura y humedad, generando marcadores visuales en RViz 
    para los productos con problemas.
    """
    def __init__(self):
        # Conexión a la base de datos Supabase
        url = "https://byoaicickhqbidpdeber.supabase.co"
        key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImJ5b2FpY2lja2hxYmlkcGRlYmVyIiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTczNTg0MDA1MSwiZXhwIjoyMDUxNDE2MDUxfQ.rTeQ5zuYhwNBDb8aonOt23TFxS-2MeLkJX2OlSyK0RY"
        self.conn: Client = create_client(url, key)

        # Almacena los IDs de Beacons ya procesados para evitar duplicados
        self.received_ids = set()

        # Publicador para marcadores visuales en RViz
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # Última posición conocida del robot (se actualiza con datos de odometría)
        self.last_position = None

        # Archivo local para almacenar datos de marcadores generados
        self.marker_data_file = "productos_erroneos.json"
        self.load_marker_data()

        # Suscribirse a los tópicos relevantes
        rospy.Subscriber('/ble/message', String, self.process_beacon_data)  # Mensajes BLE
        rospy.Subscriber('/robot/robotnik_base_control/odom', Odometry, self.update_robot_position)  # Posición del robot

        rospy.loginfo("Nodo read_ble iniciado. Escuchando en /ble/message...")

    def load_marker_data(self):
        """
        Carga los datos de marcadores almacenados en un archivo JSON.
        Si no existe el archivo o hay errores, inicializa una lista vacía.
        """
        if os.path.exists(self.marker_data_file):
            try:
                with open(self.marker_data_file, "r") as file:
                    self.marker_data = json.load(file)
            except (json.JSONDecodeError, IOError) as e:
                rospy.logerr(f"Error cargando {self.marker_data_file}: {e}. Inicializando con una lista vacía.")
                self.marker_data = []
        else:
            self.marker_data = []

    def save_marker_data(self):
        """
        Guarda los datos de marcadores generados en un archivo JSON.
        """
        with open(self.marker_data_file, "w") as file:
            json.dump(self.marker_data, file, indent=4)

    def update_robot_position(self, msg):
        """
        Callback para actualizar la posición del robot.
        Este método almacena la última posición recibida desde el tópico de odometría.
        """
        self.last_position = msg.pose.pose.position

    def process_beacon_data(self, msg):
        """
        Procesa los datos de un beacon BLE recibido en formato hexadecimal.
        Realiza verificaciones contra la base de datos y genera marcadores en caso de errores.
        """
        try:
            # Convertir el paquete hexadecimal en bytes
            packet_hex = msg.data.strip()
            packet_bytes = bytes.fromhex(packet_hex)

            # Desempaquetar el paquete para obtener ID, temperatura y humedad
            beacon_id, temperature, humidity = struct.unpack(">IhH", packet_bytes)
            rospy.loginfo(f"Beacon ID: {beacon_id}, Temp: {temperature}°C, Hum: {humidity}%")

            # Evitar procesar un ID ya recibido
            if beacon_id in self.received_ids:
                return

            # Consultar en la base de datos información sobre el beacon
            response = self.conn.table("inventory").select("id, gtin, seccion, temp_min, temp_max, hum_min, hum_max").eq("uuid", beacon_id).execute()

            has_error = False
            error_type = ""

            if response.data:
                # Datos obtenidos de la base de datos
                db_data = response.data[0]
                product_id = db_data["id"]
                gtin_db = db_data["gtin"]
                seccion_db = db_data["seccion"]
                temp_min = db_data["temp_min"]
                temp_max = db_data["temp_max"]
                hum_min = db_data["hum_min"]
                hum_max = db_data["hum_max"]

                # Comprobar sección esperada según el ID del beacon
                id_prefix = int(str(beacon_id)[0])
                expected_sections = {
                    1: "Electrónica",
                    2: "Hogar",
                    3: "Coche y Moto",
                    4: "Deportes",
                    5: "Salud",
                    6: "Belleza",
                    7: "Otros",
                }
                expected_section = expected_sections.get(id_prefix, "Desconocida")

                # Verificar si hay discrepancias en la sección
                if seccion_db != expected_section:
                    rospy.logerr(f"Error en Sección: GTIN {gtin_db} debería estar en '{expected_section}', pero está en '{seccion_db}'.")
                    has_error = True
                    error_type = "Fuera de sección"

                # Verificar si la temperatura está dentro del rango permitido
                if not (temp_min <= temperature <= temp_max):
                    rospy.logerr(f"Error en Temperatura: GTIN {gtin_db} tiene {temperature}°C, fuera de rango ({temp_min}°C - {temp_max}°C).")
                    has_error = True
                    error_type = "Temperatura fuera de rango"

                # Verificar si la humedad está dentro del rango permitido
                if not (hum_min <= humidity <= hum_max):
                    rospy.logerr(f"Error en Humedad: GTIN {gtin_db} tiene {humidity}%, fuera de rango ({hum_min}% - {hum_max}%).")
                    has_error = True
                    error_type = "Humedad fuera de rango"

                if not has_error:
                    rospy.loginfo(f"Beacon ID: {beacon_id} validado correctamente en la base de datos.")
            else:
                rospy.logerr(f"Beacon ID: {beacon_id} no encontrado en la base de datos.")
                has_error = True
                error_type = "Producto no encontrado"

            # Actualizar el estado del producto en la base de datos si hay errores
            if has_error:
                self.update_product_state(db_data.get("id", 0), error_type)

            # Crear y publicar un marcador en RViz si hay errores y se conoce la posición del robot
            if has_error and self.last_position:
                self.create_marker(self.last_position, product_id)

            # Registrar el ID procesado
            self.received_ids.add(beacon_id)

        except ValueError as e:
            rospy.logerr(f"Error procesando datos del beacon: {e}")
        except Exception as e:
            rospy.logerr(f"Error al consultar la base de datos: {e}")

    def update_product_state(self, product_id, error_type):
        """
        Actualiza el estado del producto en la base de datos con un tipo de error específico.
        """
        try:
            self.conn.table("inventory").update({"estado": error_type}).eq("id", product_id).execute()
            rospy.loginfo(f"Estado del producto ID {product_id} actualizado a '{error_type}'.")
        except Exception as e:
            rospy.logerr(f"Error actualizando el estado del producto ID {product_id}: {e}")

    def create_marker(self, position, product_id):
        """
        Genera un marcador visual para RViz en la posición actual del robot y lo guarda en un archivo JSON.
        """
        marker_id = int(rospy.Time.now().to_sec())  # ID único basado en la hora actual
        marker = Marker()
        marker.header.frame_id = "robot_map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "error_markers"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = position
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.3  # Tamaño del marcador
        marker.color.a = 1.0  # Opacidad completa
        marker.color.r = 1.0  # Color rojo para errores

        # Publicar el marcador en RViz
        self.marker_pub.publish(marker)

        # Guardar los datos del marcador en el archivo JSON
        marker_data = {
            "marker_id": marker_id,
            "product_id": product_id,
            "position": {"x": position.x, "y": position.y, "z": position.z},
        }
        self.marker_data.append(marker_data)
        self.save_marker_data()

        rospy.loginfo(f"Marcador de error creado en ({position.x}, {position.y}, {position.z}) y almacenado en {self.marker_data_file}")

if __name__ == '__main__':
    """
    Bloque principal que inicializa el nodo y ejecuta el spin de ROS.
    """
    rospy.init_node('read_ble', anonymous=True)
    try:
        checker = BeaconDataChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
