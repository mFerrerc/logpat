#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int32
import struct
import random

def generate_packet(beacon_id, temperature, humidity):
    """
    Genera un paquete BLE simulando el modo Custom Advertising.
    El paquete contiene un ID de 4 bytes, temperatura de 2 bytes y humedad de 2 bytes en hexadecimal.
    """
    packet = struct.pack(">IhH", beacon_id, temperature, humidity)  # Big endian, 4 bytes ID, 2 bytes temp, 2 bytes humidity
    return ''.join(["{:02X}".format(ord(byte)) for byte in packet])  # Convertir cada byte a entero y luego a hexadecimal

def beacon_simulator():
    """
    Nodo ROS que simula Beacons basados en el estado recibido por /state y activados por /orden.
    """
    rospy.init_node('beacon_simulator', anonymous=True)
    pub = rospy.Publisher('/ble/message', String, queue_size=10)

    # Variables globales para control

    global current_order
    global current_state
    global zero_count, active_state, patroll, aux
    current_order = ""
    current_state = 0

    # Variables para mantener el progreso del bucle
    last_state = None
    beacon_index = 0

    
    active_state = None
    zero_count = 0
    aux = False
    patroll = False
    
    # Mapear tiempos para cada estado
    state_intervals = {
        1: 20,
        2: 25,
        3: 60,
        4: 40,
        5: 110,
        6: 80,
        7: 180
    }

    def orden_callback(msg):
        global current_order
        current_order = msg.data.lower()

    def state_callback(msg):
        global current_state, zero_count, active_state, patroll, aux
        state = msg.data

        # Incrementar contador de ceros si se reciben consecutivamente
        if state == 0:
            zero_count += 1
        else:
            zero_count = 0  # Reiniciar el contador si no es 0

        # Si se detectan tres ceros consecutivos, reiniciar a estado inicial
        if zero_count > 3:
            rospy.loginfo("Tres ceros consecutivos detectados. Volviendo al estado inicial.")
            active_state = None
            current_state = 0
            zero_count = 0
            aux = False
            patroll = False
            return

        # Mantener el estado activo si el nuevo estado es 0
        if state == 0 and active_state is not None:
            return

        # Actualizar el estado actual y el estado activo
        current_state = state
        active_state = state

    rospy.Subscriber('/orden', String, orden_callback)
    rospy.Subscriber('/section', Int32, state_callback)

    rate = rospy.Rate(1)  # Frecuencia de comprobación (1 Hz)

    rospy.loginfo("Simulador de Beacons inicializado.")

    while not rospy.is_shutdown():
        if current_order == "patrulla":
            patroll = True

        if patroll and (active_state is not None and 0 < active_state <= 7):
            aux = True
            interval = state_intervals[active_state] / 10

            # Si el estado cambia, reinicia el índice del beacon
            if active_state != last_state:
                last_state = active_state
                beacon_index = 0
                rospy.loginfo("Orden: patrulla. Estado: %d. Intervalo: %d segundos.", active_state, interval)

            # Continuar imprimiendo desde el último índice
            for i in range(beacon_index, 10):
                # Verificar si el estado ha cambiado a algo distinto (excepto 0)
                if active_state != last_state:
                    rospy.loginfo("Cambio de estado detectado. Reiniciando bucle para el nuevo estado.")
                    break

                # Generar valores de temperatura y humedad
                temperature = random.randint(8, 32)
                humidity = random.randint(38, 62)

                # Generar y publicar el paquete
                beacon_id = int("{:01d}000000{:01d}".format(active_state, i))
                packet_hex = generate_packet(beacon_id, temperature, humidity)
                message = "Beacon ID: {:d} | Temp: {:d} | Hum: {:d} | Packet: {}".format(beacon_id, temperature, humidity, packet_hex)

                rospy.loginfo(message)

                message = "{}".format(packet_hex)
                pub.publish(message)

                # Esperar el intervalo antes de publicar el siguiente beacon
                rospy.sleep(interval)

                # Actualizar el índice del beacon
                beacon_index = i + 1

        elif not patroll:
            rospy.loginfo_throttle(10, "Esperando orden 'patrulla' en el tópico /orden.")
        elif current_state == 0:
            rospy.loginfo_throttle(10, "Estado actual: 0. Continuando con el estado activo anterior.")

        rate.sleep()

if __name__ == '__main__':
    try:

        beacon_simulator()
    except rospy.ROSInterruptException:
        rospy.loginfo("SI.")
        pass

