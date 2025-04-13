#!/usr/bin/env python2.7
# -*- coding: utf-8 -*-

"""
Este script procesa imágenes recibidas a través de ROS, realiza un recorte inicial basado en el color de la etiqueta,
preprocesa la imagen, activa y desactiva el proceso de recorte según el estado del tópico /caja_ready, guarda la imagen recortada
en un directorio compartido para que un programa en Python 3.9 realice la extracción de texto utilizando una librería OCR
más actualizada, espera el resultado, y luego publica los datos extraídos en tópicos ROS específicos.

Dependencias:
- ROS (Robot Operating System)
- OpenCV
- Pytesseract
- cv_bridge

Instalación de Tesseract y Pytesseract:
    sudo apt install tesseract-ocr
    pip install pytesseract==0.2.7
"""

import rospy
import cv2
import pytesseract
import json
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from std_msgs.msg import String, Bool
import re
import os
import sys
import time

# Configurar la ruta de Tesseract
pytesseract.pytesseract.tesseract_cmd = "/usr/bin/tesseract"

class LecturaEtiquetado(object):
    def __init__(self):
        # Inicializar el nodo y el bridge de OpenCV
        self.bridge = CvBridge()
        
        # Subscritor al tópico de la cámara
        rospy.Subscriber('/robot/front_rgbd_camera/rgb/image_raw', Image, self.callback_image)

        # Subscritor al tópico '/caja_ready'
        rospy.Subscriber('/caja_ready', Bool, self.callback_caja_ready)
        self.caja_ready_pub = rospy.Publisher('/caja_ready', Bool, queue_size=10)
        self.caja_ready = False
        self.previous_caja_ready = None  # Para rastrear cambios en el estado

        # Publishers para los datos extraídos
        self.pub_sscc = rospy.Publisher('/inventory/label/sscc', String, queue_size=10)
        self.pub_content = rospy.Publisher('/inventory/label/content', String, queue_size=10)
        self.pub_batchlot = rospy.Publisher('/inventory/label/batchlot', String, queue_size=10)
        self.pub_count = rospy.Publisher('/inventory/label/count', String, queue_size=10)
        self.pub_weight = rospy.Publisher('/inventory/label/weight', String, queue_size=10)
        self.pub_date_day = rospy.Publisher('/inventory/label/date', String, queue_size=10)

        # Directorio para guardar imágenes de depuración (opcional)
        self.debug_dir = "/tmp/lectura_etiquetado_debug"
        if not os.path.exists(self.debug_dir):
            os.makedirs(self.debug_dir)

        # Directorios para comunicación con el OCR en Python 3.9
        self.ocr_input_dir = "/tmp/ocr_input"
        self.ocr_output_dir = "/tmp/ocr_output"

        # Crear directorios si no existen
        if not os.path.exists(self.ocr_input_dir):
            os.makedirs(self.ocr_input_dir)
        if not os.path.exists(self.ocr_output_dir):
            os.makedirs(self.ocr_output_dir)

    def callback_caja_ready(self, msg):
        """
        Callback para manejar el estado de '/caja_ready'.
        Solo registra cambios en el estado.
        """
        if msg.data != self.previous_caja_ready:
            self.caja_ready = msg.data
            if self.caja_ready:
                rospy.loginfo("Estado de /caja_ready cambiado a: True (Caja lista. Iniciando proceso de recorte.)")
            else:
                rospy.loginfo("Estado de /caja_ready cambiado a: False (Proceso de recorte finalizado. Esperando otra caja.)")
            self.previous_caja_ready = msg.data

    def callback_image(self, msg):
        """
        Callback para manejar imágenes recibidas.
        Procesa la imagen solo si 'caja_ready' es True.
        """
        if not self.caja_ready:
            # No hacer nada si la caja no está lista
            return

        try:
            # Convertir mensaje ROS a imagen OpenCV
            np_arr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Recortar la imagen para aislar la etiqueta basada en color
            cropped_label_image = self.crop_to_label(np_arr)

            # Procesar la imagen
            cropped_image = self.preprocess_image(cropped_label_image)
            
            # Guardar la imagen recortada en el directorio compartido
            timestamp = int(time.time() * 1000)  # Timestamp en milisegundos
            image_filename = os.path.join(self.ocr_input_dir, "cropped_{0}.png".format(timestamp))
            cv2.imwrite(image_filename, cropped_image)

            # Esperar a que el OCR procese y lea el resultado
            result_filename = os.path.join(self.ocr_output_dir, "result.json".format(timestamp))
            timeout = 10  # segundos
            start_time = time.time()
            while not os.path.exists(result_filename):
                if time.time() - start_time > timeout:
                    rospy.logerr("Timeout esperando el resultado del OCR para %s", image_filename)
                    break
                time.sleep(0.1)
            
            if os.path.exists(result_filename):
                with open(result_filename, 'r') as f:
                    datos_extraidos = json.load(f)
                
                # Publicar los datos en los tópicos ROS
                self.publish_data(datos_extraidos)
                
                # Opcional: eliminar los archivos después de procesarlos
                try:
                    os.remove(image_filename)
                    os.remove(result_filename)
                except Exception as e:
                    rospy.logwarn("No se pudieron eliminar los archivos de OCR: %s", str(e))
            
        except CvBridgeError as e:
            rospy.logerr("Error en la conversión de imagen: %s", str(e))
        except ValueError as ve:
            rospy.logerr("Valor inválido: %s", str(ve))
        except Exception as e:
            rospy.logerr("Error al procesar la imagen: %s", str(e))
        finally:
            # Desactivar el proceso de recorte estableciendo /caja_ready a False
            self.caja_ready_pub.publish(Bool(data=False))

    def crop_to_label(self, image):
        """
        Recorta la imagen original para aislar la etiqueta basada en su color.
        """
        # Convertir la imagen a espacio de color HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Definir el rango de color para la etiqueta (gris aproximado)
        lower_gray = np.array([0, 0, 100])
        upper_gray = np.array([180, 30, 160])

        # Crear máscara para el color de la etiqueta
        mask = cv2.inRange(hsv_image, lower_gray, upper_gray)

        # Aplicar operaciones morfológicas para limpiar la máscara
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=2)

        # Encontrar contornos en la máscara
        contours_result = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # Compatibilidad con diferentes versiones de OpenCV
        if len(contours_result) == 3:
            _, contours, _ = contours_result
        else:
            contours = contours_result[0]

        if not contours:
            raise ValueError("No se detectaron contornos con el color de la etiqueta.")

        # Seleccionar el contorno más grande
        largest_contour = max(contours, key=cv2.contourArea)

        # Obtener el rectángulo delimitador del contorno
        x, y, w, h = cv2.boundingRect(largest_contour)

        # Recortar la imagen original a la región de la etiqueta
        cropped_image = image[y:y+h, x:x+w]

        return cropped_image

    def preprocess_image(self, image):
        """
        Realiza las etapas de preprocesamiento de la imagen para preparar la extracción de texto.
        """
        # Convertir a escala de grises
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        
        # Aplicar un filtro bilateral para preservar bordes y reducir ruido
        filtered_image = cv2.bilateralFilter(gray_image, 9, 75, 75)
        
        # Aplicar una umbralización adaptativa
        threshold_image = cv2.adaptiveThreshold(
            filtered_image, 255,
            cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 11, 2
        )
        
        # Aplicar operaciones morfológicas para cerrar huecos en las líneas
        kernel_size = (5, 5)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, kernel_size)
        morphed_image = cv2.morphologyEx(threshold_image, cv2.MORPH_CLOSE, kernel, iterations=2)

        # Detectar bordes con Canny
        edges = cv2.Canny(morphed_image, 50, 150, apertureSize=3)

        # Detectar líneas usando HoughLinesP con parámetros ajustables
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=150, minLineLength=150, maxLineGap=20)

        # Filtrar líneas horizontales más estrictamente
        horizontal_lines = []
        if lines is not None:
            for line in lines:
                if len(line) < 1:
                    continue
                if len(line[0]) != 4:
                    continue
                x1, y1, x2, y2 = line[0]
                delta_y = abs(y1 - y2)
                delta_x = abs(x2 - x1)
                if delta_x == 0:
                    angle = 90
                else:
                    angle = np.degrees(np.arctan2(delta_y, delta_x))
                if angle < 5 and delta_x > 100:
                    horizontal_lines.append((x1, y1, x2, y2))

        # Agrupar y fusionar líneas cercanas
        grouped_lines = self.group_and_merge_lines(horizontal_lines, y_threshold=10)

        # Ordenar las líneas por la coordenada y (posición vertical)
        grouped_lines = sorted(grouped_lines, key=lambda line: line[1])

        if len(grouped_lines) < 2:
            raise ValueError("No se detectaron suficientes líneas horizontales prominentes.")
        
        top_line = grouped_lines[0]
        bottom_line = grouped_lines[-1]

        # Coordenadas de recorte
        y_start = top_line[1]
        y_end = bottom_line[1]

        # Recortar la región de interés
        cropped_image = gray_image[y_start:y_end, :]

        return cropped_image

    def group_and_merge_lines(self, lines, y_threshold=10):
        """
        Agrupa líneas que están cerca verticalmente y las fusiona en una sola línea.
        """
        if not lines:
            return []

        # Ordenar las líneas por la coordenada y
        lines = sorted(lines, key=lambda line: line[1])

        grouped = []
        current_group = [lines[0]]

        for line in lines[1:]:
            if len(line) != 4:
                continue
            _, y1_current, _, _ = line
            _, y1_group, _, _ = current_group[-1]

            if abs(y1_current - y1_group) < y_threshold:
                current_group.append(line)
            else:
                merged_line = self.merge_lines(current_group)
                grouped.append(merged_line)
                current_group = [line]
        
        if current_group:
            merged_line = self.merge_lines(current_group)
            grouped.append(merged_line)
        
        return grouped

    def merge_lines(self, lines):
        """
        Fusiona un grupo de líneas en una sola línea extendida.
        """
        x_coords = []
        y_coords = []
        for line in lines:
            if len(line) != 4:
                continue
            x1, y1, x2, y2 = line
            x_coords.extend([x1, x2])
            y_coords.extend([y1, y2])
        
        if not x_coords or not y_coords:
            return (0, 0, 0, 0)

        x_min = min(x_coords)
        x_max = max(x_coords)
        y_avg = int(np.mean(y_coords))
        
        return (x_min, y_avg, x_max, y_avg)

    def publish_data(self, data):
        """
        Publica los datos obtenidos del archivo JSON en los tópicos ROS.
        Además, se muestra por consola la información que se está publicando.
        """
        sscc = data.get("sscc", "")
        content = data.get("content", "")
        batch = data.get("batch", "")
        count_ = data.get("count", "")
        weight = data.get("weight", "")
        date_ = data.get("date", "")

        # Publicar en tópicos
        self.pub_sscc.publish(sscc)
        self.pub_content.publish(content)
        self.pub_batchlot.publish(batch)
        self.pub_count.publish(count_)
        self.pub_weight.publish(weight)
        self.pub_date_day.publish(date_)

        print("\nPublicando datos en tópicos:\n"
              "  SSCC: {},\n"
              "  Content: {},\n"
              "  Batch: {},\n"
              "  Count: {},\n"
              "  Weight: {},\n"
              "  Date: {}".format(
                  sscc, content, batch, count_, weight, date_
              ))


def main():
    rospy.init_node('lectura_etiquetado', anonymous=True)
    LecturaEtiquetado()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
