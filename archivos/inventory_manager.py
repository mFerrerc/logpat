#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import ast
import json
from std_msgs.msg import String
from supabase import create_client, Client

class InventoryManager:
    def __init__(self):
        # Conexión a la base de datos
        url = "https://byoaicickhqbidpdeber.supabase.co"
        key = "eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJpc3MiOiJzdXBhYmFzZSIsInJlZiI6ImJ5b2FpY2lja2hxYmlkcGRlYmVyIiwicm9sZSI6InNlcnZpY2Vfcm9sZSIsImlhdCI6MTczNTg0MDA1MSwiZXhwIjoyMDUxNDE2MDUxfQ.rTeQ5zuYhwNBDb8aonOt23TFxS-2MeLkJX2OlSyK0RY"
        self.conn: Client = create_client(url, key)

        # Suscribirse a los tópicos
        rospy.Subscriber('/inventory_moviles/register', String, self.register_product)
        rospy.Subscriber('/inventory_moviles/update', String, self.update_product)
        rospy.Subscriber('/inventory_moviles/delete', String, self.delete_product)

        # Variables para almacenar datos de los nuevos tópicos
        self.sender = ""
        self.sscc = ""
        self.content = ""
        self.batchlot = ""
        self.count = 0
        self.net_weight = 0.0
        self.expiration_date = ""

        # Suscripciones a los nuevos topics publicados por etiqueta.py
        rospy.Subscriber('/inventory/label/content', String, self.update_content)
        rospy.Subscriber('/inventory/label/batchlot', String, self.update_batchlot)
        rospy.Subscriber('/inventory/label/count', String, self.update_count)
        rospy.Subscriber('/inventory/label/weight', String, self.update_weight)
        rospy.Subscriber('/inventory/label/date', String, self.update_expiration_date)

        # Publicador para el resumen del inventario
        self.inventory_pub = rospy.Publisher('/inventory_moviles/summary', String, queue_size=10)

        rospy.loginfo("Nodo de gestión de inventario inicializado.")

    def register_product(self, msg):
        """Registrar un producto en la base de datos."""
        try:
            data = ast.literal_eval(msg.data)
            response = self.conn.table("inventory").insert(data).execute()
            rospy.loginfo(f"Producto registrado: {data['gtin']}")
        except Exception as e:
            rospy.logerr(f"Error registrando producto: {str(e)}")

    def update_product(self, msg):
        """Actualizar un producto en Supabase."""
        try:
            data = ast.literal_eval(msg.data)
            
            # Actualización dinámica con Supabase
            update_data = {}
            campos = ["uuid", "nombre", "seccion", "fabricante", "lote", "peso_neto", "volumen", "fecha_expiracion", "estado", "precio_compra", "precio_venta", "temp_min", "temp_max", "hum_min", "hum_max"]
            for i, campo in enumerate(campos):
                if campo in data:
                    update_data[campo] = data[campo]

            # Realizar la actualización solo si hay datos que modificar
            if update_data:
                response = self.conn.table("inventory").update(update_data).eq("gtin", data['gtin']).execute()
                rospy.loginfo(f"Producto actualizado con GTIN: {data['gtin']}")
            else:
                rospy.logwarn("No se recibieron campos válidos para actualizar.")
        except Exception as e:
            rospy.logerr(f"Error actualizando producto: {str(e)}")

    # Funciones de actualización a partir de los topics
    def update_content(self, msg):
        self.content = msg.data
        print(self.content)

    def update_batchlot(self, msg):
        self.batchlot = msg.data

    def update_count(self, msg):
        self.count = int(msg.data)

    def update_weight(self, msg):
        self.net_weight = float(msg.data)

    def update_expiration_date(self, msg):
        self.expiration_date = msg.data
        self.check_gtin_in_database()

    def check_gtin_in_database(self):
        try:
            response = self.conn.table("inventory").select("*").eq("gtin", self.content).execute()
            if response.data:
                rospy.loginfo(f"GTIN encontrado: {self.content}")
                existing_product = response.data[0]
                for _ in range(self.count):
                    self.add_new_product(existing_product)
            else:
                rospy.logwarn("GTIN no encontrado en la base de datos.")
        except Exception as e:
            rospy.logerr(f"Error consultando la base de datos: {str(e)}")

    def add_new_product(self, existing_product):
        try:
            new_product = {
                "gtin": self.content,
                "uuid": existing_product['uuid'],
                "stock": 1,
                "peso_neto": self.net_weight,
                "fecha_expiracion": self.expiration_date,
                "lote": self.batchlot,
                "nombre": existing_product['nombre'],
                "seccion": existing_product['seccion'],
                "fabricante": existing_product['fabricante'],
                "precio_compra": existing_product['precio_compra'],
                "precio_venta": existing_product['precio_venta'],
                "estado": existing_product['estado'],
                "volumen": existing_product['volumen'],
                "temp_min": existing_product['temp_min'],
                "temp_max": existing_product['temp_max'],
                "hum_min": existing_product['hum_min'],
                "hum_max": existing_product['hum_max'],
            }
            self.conn.table("inventory").insert(new_product).execute()
            rospy.loginfo(f"Nuevo producto agregado con GTIN: {self.content}")
        except Exception as e:
            rospy.logerr(f"Error agregando nuevo producto: {str(e)}")

    def delete_product(self, msg):
        try:
            data = ast.literal_eval(msg.data)
            if 'id' in data:
                self.conn.table("inventory").delete().eq("id", data['id']).execute()
                rospy.loginfo(f"Producto eliminado con ID: {data['id']}")
            elif 'gtin' in data:
                self.conn.table("inventory").delete().eq("gtin", data['gtin']).execute()
                rospy.loginfo(f"Productos eliminados con GTIN: {data['gtin']}")
        except Exception as e:
            rospy.logerr(f"Error eliminando producto: {str(e)}")

    def publish_inventory(self):
        try:
            response = self.conn.table("inventory").select("id, gtin, nombre, stock, estado").execute()
            for product in response.data:
                summary = f"ID: {product['id']}, GTIN: {product['gtin']}, Nombre: {product['nombre']}, Stock: {product['stock']}, Estado: {product['estado']}"
                self.inventory_pub.publish(summary)
        except Exception as e:
            rospy.logerr(f"Error publicando inventario: {str(e)}")

if __name__ == '__main__':
    rospy.init_node('inventory_manager', anonymous=True)
    manager = InventoryManager()
    rate = rospy.Rate(1)
    try:
        while not rospy.is_shutdown():
            manager.publish_inventory()
            rate.sleep()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo detenido.")
