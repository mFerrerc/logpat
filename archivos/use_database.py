#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import json
import os
from std_msgs.msg import String
from supabase import create_client, Client
from visualization_msgs.msg import Marker

class InventoryControl:
    def __init__(self):
        rospy.init_node('inventory_control', anonymous=True)

        # Configuración de conexión a Supabase
        url = "https://byoaicickhqbidpdeber.supabase.co"
        key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImJ5b2FpY2lja2hxYmlkcGRlYmVyIiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTczNTg0MDA1MSwiZXhwIjoyMDUxNDE2MDUxfQ.rTeQ5zuYhwNBDb8aonOt23TFxS-2MeLkJX2OlSyK0RY"
        self.conn: Client = create_client(url, key)

        # Publicadores para los tópicos de registro, edición, eliminación y markers
        self.register_pub = rospy.Publisher('/inventory_moviles/register', String, queue_size=10)
        self.update_pub = rospy.Publisher('/inventory_moviles/update', String, queue_size=10)
        self.delete_pub = rospy.Publisher('/inventory_moviles/delete', String, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        # Archivo JSON de productos erróneos
        self.error_file = "productos_erroneos.json"
        self.load_error_file()

        rospy.loginfo("Nodo de control de inventario inicializado.")
        self.menu_interactivo()

    def load_error_file(self):
        """Carga los datos de productos erróneos desde el archivo JSON."""
        if os.path.exists(self.error_file):
            try:
                with open(self.error_file, "r") as file:
                    self.error_data = json.load(file)
            except (IOError, json.JSONDecodeError):
                rospy.logwarn(f"Error cargando {self.error_file}. Inicializando como lista vacía.")
                self.error_data = []
        else:
            self.error_data = []

    def save_error_file(self):
        """Guarda los datos actualizados en el archivo JSON."""
        with open(self.error_file, "w") as file:
            json.dump(self.error_data, file, indent=4)

    def menu_interactivo(self):
        while not rospy.is_shutdown():
            print("\nSeleccione la acción a realizar:")
            print("1. Registrar producto")
            print("2. Modificar producto")
            print("3. Eliminar producto")
            print("4. Corregir error")
            accion = input("Ingrese el número de la acción (1-4): ")

            if accion == "1":
                self.registrar_producto()
            elif accion == "2":
                self.modificar_producto()
            elif accion == "3":
                self.eliminar_producto()
            elif accion == "4":
                self.corregir_error()
            else:
                print("Opción no válida. Intente nuevamente.")

    def corregir_error(self):
        """Corrige un error eliminando el marcador correspondiente en RViz y actualizando el estado."""
        if not self.error_data:
            print("No hay productos erróneos registrados.")
            return

        print("\nProductos erróneos detectados:")
        for idx, marker_info in enumerate(self.error_data):
            print(f"{idx + 1}. ID: {marker_info['product_id']}, Marker ID: {marker_info['marker_id']}")

        seleccion = input("Seleccione el número del producto para corregir (o presione Enter para cancelar): ")
        if not seleccion.isdigit() or not (1 <= int(seleccion) <= len(self.error_data)):
            print("Selección inválida. Volviendo al menú principal.")
            return

        seleccion_idx = int(seleccion) - 1
        marker_info = self.error_data[seleccion_idx]
        product_id = marker_info["product_id"]

        # Consultar detalles del producto en Supabase
        response = self.conn.table("inventory").select("*").eq("id", product_id).execute()
        if not response.data:
            print(f"No se encontraron detalles para el producto con ID {product_id}.")
            return

        producto = response.data[0]
        print("\nDetalles del producto seleccionado:")
        print(f"ID: {producto['id']}")
        print(f"GTIN: {producto['gtin']}")
        print(f"UUID: {producto['uuid']}")
        print(f"Nombre: {producto['nombre']}")
        print(f"Sección: {producto['seccion']}")
        print(f"Estado: {producto['estado']}")

        confirmacion = input("\n¿Desea eliminar el marcador y corregir el error? (S/N): ").upper()
        if confirmacion == "S":
            # Eliminar el marcador en RViz
            self.delete_marker(marker_info["marker_id"])

            # Actualizar el estado del producto en la base de datos
            self.update_product_state(product_id, "Correcto")

            # Eliminar el producto del JSON
            del self.error_data[seleccion_idx]
            self.save_error_file()

            rospy.loginfo(f"Producto erróneo con ID {product_id} corregido correctamente.")
            print("Producto corregido y eliminado.")
        else:
            print("Corrección cancelada. Volviendo al menú principal.")

    def delete_marker(self, marker_id):
        """Publica un mensaje para eliminar el marcador en RViz."""
        marker = Marker()
        marker.header.frame_id = "robot_map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "error_markers"
        marker.id = marker_id
        marker.action = Marker.DELETE

        self.marker_pub.publish(marker)
        rospy.loginfo(f"Marcador con ID {marker_id} eliminado en RViz.")

    def update_product_state(self, product_id, new_state):
        """Actualiza el campo 'estado' del producto en la base de datos."""
        try:
            self.conn.table("inventory").update({"estado": new_state}).eq("id", product_id).execute()
            rospy.loginfo(f"Estado del producto ID {product_id} actualizado a '{new_state}'.")
        except Exception as e:
            rospy.logerr(f"Error actualizando el estado del producto ID {product_id}: {e}")

if __name__ == '__main__':
    try:
        InventoryControl()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo detenido.")
