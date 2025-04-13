#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""
Script en ROS que implementa una máquina de estados finitos usando `smach`.  
Permite al robot realizar patrullaje siguiendo waypoints, moverse a una zona de etiquetado  
y regresar a la base tras completar sus tareas. Utiliza `move_base` para la navegación.

Dependencias:
    - ROS (Robot Operating System)
    - rospy
    - smach
    - smach_ros

Instalación smach y move_base:
    sudo apt install ros-noetic-smach ros-noetic-smach-ros ros-noetic-move-base-msgs
    pip install smach smach_ros
"""

import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult
from tf.transformations import quaternion_from_euler
from std_msgs.msg import String, Int32, Float32
import time

class Base(smach.State):
    """
        Estado inicial donde el robot espera órdenes para patrullar o registrar productos.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['start_register', 'patrol', 'stay_base'])
        self.received_message = None
        # Mensaje del tópico '/orden' indicando la acción a tomar.
        rospy.Subscriber('/orden', String, self.message_callback)

    def message_callback(self, msg):
        self.received_message = msg.data

    def execute(self, userdata):
        rospy.loginfo("Esperando mensaje en el estado Base...")
        
        # Reiniciar la variable al entrar en el estado
        self.received_message = None

        while not rospy.is_shutdown():
            if self.received_message == 'etiquetado':
                rospy.loginfo("Mensaje 'etiquetado' recibido. Transición a StartRegister.")
                return 'start_register'                                                         # 'start_register': Si el mensaje recibido es 'etiquetado'.
            elif self.received_message == 'patrulla':
                rospy.loginfo("Mensaje 'patrulla' recibido. Transición a Patrulla.")
                return 'patrol'                                                                 # 'patrol': Si el mensaje recibido es 'patrulla'.
            rospy.sleep(0.1)
        return 'stay_base'                                                                      # 'stay_base': Si no se recibe ningún mensaje.

class ReturnToBase(smach.State):
    """
        Estado en el que el robot regresa a la base.
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['base'])
        self.goal_pub = rospy.Publisher('/robot/move_base_simple/goal', PoseStamped, queue_size=10)
        self.result_sub = rospy.Subscriber('/robot/move_base/result', MoveBaseActionResult, self.result_callback)
        self.result_status = None
        rospy.sleep(1)

    def result_callback(self, msg):
        self.result_status = msg.status.status

    def execute(self, userdata):
        # Definimos coordenadas de home y enviamos al robot a ellas
        rospy.loginfo("Moviendo a la base...")
        goal = PoseStamped()
        goal.header.frame_id = "robot_map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 18.0
        goal.pose.position.y = 8.0
        goal.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0.0, 0.0, 0.0)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        self.goal_pub.publish(goal)

        timeout = rospy.Time.now() + rospy.Duration(60)
        while self.result_status is None and rospy.Time.now() < timeout:
            rospy.sleep(0.5)

        # Comprobamos si ha llegado
        if self.result_status == 3:
            rospy.loginfo("Llegado a la base.")
            return 'base'               
        else:
            rospy.logerr("Error al intentar regresar a la base.")
            return 'base'

class StartRegister(smach.State):
    """
        Estado StartRegister: Mueve el robot a la zona de etiquetado
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['register', 'failed'])
        self.goal_pub = rospy.Publisher('/robot/move_base_simple/goal', PoseStamped, queue_size=10)
        self.result_sub = rospy.Subscriber('/robot/move_base/result', MoveBaseActionResult, self.result_callback)
        self.result_status = None
        rospy.sleep(1)

    def result_callback(self, msg):
        self.result_status = msg.status.status

    def execute(self, userdata):
        rospy.loginfo("Moviendo a la zona de etiquetado...")
        goal = PoseStamped()
        goal.header.frame_id = "robot_map"
        goal.header.stamp = rospy.Time.now()
        goal.pose.position.x = 17.22
        goal.pose.position.y = 10.28
        goal.pose.position.z = 0.0
        quaternion = quaternion_from_euler(0.0, 0.0, 1.558)
        goal.pose.orientation.x = quaternion[0]
        goal.pose.orientation.y = quaternion[1]
        goal.pose.orientation.z = quaternion[2]
        goal.pose.orientation.w = quaternion[3]
        self.goal_pub.publish(goal)

        timeout = rospy.Time.now() + rospy.Duration(120)
        while self.result_status is None and rospy.Time.now() < timeout:
            rospy.sleep(0.5)

        if self.result_status == 3:
            rospy.loginfo("Llegado a la zona de etiquetado.")
            return 'register'           # Si ha llegado a la zona de etiquetado, pasa al estado de registro.
        else:
            rospy.logerr("Error al intentar llegar a la zona de etiquetado.")
            return 'failed'             # Si no ha llegado, informa.

class Register(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['return_base'])
        self.received_message = None
        rospy.Subscriber('/orden', String, self.message_callback)   # - Mensaje del tópico '/orden' indicando la confirmación del etiquetado ('etiquetado').

    def message_callback(self, msg):
        if msg.data == 'etiquetado':
            self.received_message = True

    def execute(self, userdata):
        rospy.loginfo("Esperando mensaje 'etiquetado' para volver a la base...")
        
        # Reiniciar la variable al entrar en el estado
        self.received_message = False
        
        while not self.received_message:
            rospy.sleep(0.1)
        
        rospy.loginfo("Mensaje 'etiquetado' recibido. Volviendo a la base.")
        return 'return_base'                                        # Una vez recibido el mensaje de confirmación, el robot regresa a la base.

# Estado de Patrulla: Ejecuta el patrullaje siguiendo waypoints
class Patrol(smach.State):
    """
        Estado en el que el robot sigue una ruta de patrullaje definida por waypoints.
    """
    def __init__(self, waypoints):
        smach.State.__init__(self, outcomes=['base', 'failed'])
        self.waypoints = waypoints
        self.goal_pub = rospy.Publisher('/robot/move_base_simple/goal', PoseStamped, queue_size=10)
        self.result_sub = rospy.Subscriber('/robot/move_base/result', MoveBaseActionResult, self.result_callback)
        self.section_pub = rospy.Publisher('/section', Int32, queue_size=10)        # Publicador para el estado actual
        self.time_pub = rospy.Publisher('/section_time', Float32, queue_size=10)    # Publicador para el tiempo de secciones
        self.result_status = None
        rospy.sleep(1)

    def result_callback(self, msg):
        self.result_status = msg.status.status

    def execute(self, userdata):
        rospy.loginfo("Iniciando patrulla...")
        current_section = -1
        start_time = None

        for point in self.waypoints:
            self.result_status = None

            # Publicar el estado actual antes de mover al siguiente punto
            rospy.loginfo("Publicando estado: {}".format(point['state']))
            self.section_pub.publish(point['state'])

            # Si cambia de sección válida, calcula el tiempo transcurrido
            if point['state'] != current_section and point['state'] in [1, 2, 3, 4, 5, 6, 7]:
                if start_time is not None:
                    elapsed_time = time.time() - start_time
                    rospy.loginfo("Tiempo transcurrido en la sección {}: {:.2f} segundos".format(current_section, elapsed_time))
                    self.time_pub.publish(elapsed_time)
                current_section = point['state']
                start_time = time.time()

            # Definimos objetivo
            goal = PoseStamped()
            goal.header.frame_id = "robot_map"
            goal.header.stamp = rospy.Time.now()
            goal.pose.position.x = point['x']
            goal.pose.position.y = point['y']
            goal.pose.position.z = 0.0
            quaternion = quaternion_from_euler(0.0, 0.0, point['yaw'])
            goal.pose.orientation.x = quaternion[0]
            goal.pose.orientation.y = quaternion[1]
            goal.pose.orientation.z = quaternion[2]
            goal.pose.orientation.w = quaternion[3]
            self.goal_pub.publish(goal)

            timeout = rospy.Time.now() + rospy.Duration(120)
            while self.result_status is None and rospy.Time.now() < timeout:
                rospy.sleep(0.5)

            if self.result_status != 3:
                rospy.logerr("Error al mover a {point} o timeout.")
                return 'failed' # Ocurre un error durante el patrullaje.

        # Publicar el tiempo de la última sección si corresponde
        if start_time is not None and current_section in [1, 2, 3, 4, 5, 6, 7]:
            elapsed_time = time.time() - start_time
            rospy.loginfo("Tiempo transcurrido en la sección {}: {:.2f} segundos".format(current_section, elapsed_time))
            self.time_pub.publish(elapsed_time)

        rospy.loginfo("Patrulla completada. Volviendo a la base.")
        return 'base' # Patrullaje completado correctamente.


def main():
    """
        - Descripción: Función principal que inicializa el nodo ROS y mantiene el programa en ejecución.
    """
    # Inicializa el nodo ROS 
    rospy.init_node('patrol_state_machine', anonymous=True)

    # Definimos los puntos de la trayectoria
    waypoints = [
        {'x': 14.0, 'y': 8.2, 'yaw': 4.71, 'state': 0},
        {'x': 14.0, 'y': -10.0,'yaw': 3.14, 'state': 0},
        {'x': 7.5, 'y': -10.0,'yaw': 1.57, 'state': 0},
        {'x': 7.5, 'y': 2.5,'yaw': 1.57, 'state': 1},
        {'x': 7.5, 'y': 17.0,'yaw': 1.57, 'state': 2},
        {'x': 7.5, 'y': 30.0,'yaw': 3.14, 'state': 3},
        {'x': -10.0, 'y': 30.0,'yaw': 4.71, 'state': 0},
        {'x': -10.0, 'y': 9.0,'yaw': 0.0, 'state': 4},
        {'x': -8.1, 'y': 9.0,'yaw': 4.71, 'state': 0},
        {'x': -8.1, 'y': -12.5,'yaw': 3.14, 'state': 5},
        {'x': -13.0, 'y': -12.5,'yaw': 1.57, 'state': 5},
        {'x': -13.0, 'y': 9.0,'yaw': 3.14, 'state': 5},
        {'x': -24.0, 'y': 9.0,'yaw': 4.71, 'state': 0},
        {'x': -24.0, 'y': -12.5,'yaw': 3.14, 'state': 6},
        {'x': -28.5, 'y': -12.5,'yaw': 1.57, 'state': 6},
        {'x': -28.5, 'y': 9.0,'yaw': 1.57, 'state': 6},
        {'x': -28.5, 'y': 30.0,'yaw': 0.0, 'state': 7},
        {'x': -24.0, 'y': 30.0,'yaw': 4.71, 'state': 7},
        {'x': -24.0, 'y': 9.0,'yaw': 0.0, 'state': 7},
        {'x': 1.5, 'y': 9.0,'yaw': 4.71, 'state': 0},
        {'x': 1.5, 'y': 2.8,'yaw': 0.0, 'state': 0},
        {'x': 14.3, 'y': 2.8,'yaw': 0.0, 'state': 0},
        {'x': 18.0, 'y': 8.0,'yaw': 0.0, 'state': 0},
    ]

    # Creamos la máquina de estados
    sm = smach.StateMachine(outcomes=['succeeded', 'failed'])

    # Definimos transiciones dentro de la máquina
    with sm:
        smach.StateMachine.add('BASE', Base(), transitions={'start_register': 'START_REGISTER', 'patrol': 'PATROL', 'stay_base': 'BASE'})
        smach.StateMachine.add('START_REGISTER', StartRegister(), transitions={'register': 'REGISTER', 'failed': 'BASE'})
        smach.StateMachine.add('REGISTER', Register(), transitions={'return_base': 'RETURN_BASE'})
        smach.StateMachine.add('RETURN_BASE', ReturnToBase(), transitions={'base': 'BASE'})
        smach.StateMachine.add('PATROL', Patrol(waypoints), transitions={'base': 'BASE', 'failed': 'BASE'})

    # Ejecutamos la máquina de estados
    outcome = sm.execute()

if __name__ == '__main__':
    main()
