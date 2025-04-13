#!/usr/bin/env python3.9
# -*- coding: utf-8 -*-

import os
import time
import json
import re
from watchdog.observers import Observer
from watchdog.events import FileSystemEventHandler
from PIL import Image
import pytesseract


class OCRHandler(FileSystemEventHandler):
    def __init__(self, input_dir, output_dir):
        self.input_dir = input_dir
        self.output_dir = output_dir

    def on_created(self, event):
        if event.is_directory:
            return
        if not event.src_path.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tiff")):
            return
        print("Nueva imagen detectada para OCR:", event.src_path)
        self.process_image(event.src_path)

    def process_image(self, image_path):
        try:
            # Leer la imagen con Pillow
            image = Image.open(image_path)
            if image is None:
                print("No se pudo leer la imagen:", image_path)
                return

            # Extraer texto con Tesseract
            extracted_text = pytesseract.image_to_string(image, lang='eng')

            # Procesar y guardar el texto en un archivo JSON
            data = self.process_extracted_text(extracted_text)

            if not data:
                print("No se pudo procesar el texto extraído")
                return

            result_filename = os.path.join(self.output_dir, "result.json")
            with open(result_filename, 'w') as f:
                json.dump(data, f, indent=4, ensure_ascii=False)

            print("OCR completado con Tesseract y resultado guardado en:", result_filename)

            # Publicar en tópicos
            self.publish_to_topics(data)

        except Exception as e:
            print("Error al procesar la imagen:", image_path)
            print(e)

    def process_extracted_text(self, text):
        """
        Procesa el texto extraído para mapearlo a las variables correspondientes usando líneas y su orden.
        """
        try:
            # Dividir el texto en líneas y limpiar espacios en blanco
            lines = [line.strip() for line in text.split("\n") if line.strip()]

            # Crear un diccionario mapeando las líneas con los tópicos esperados
            data = {}

            # Asociar las líneas según el ejemplo dado
            try:
                data["sscc"] = lines[1]  # Segunda línea para SSCC
                data["content"] = lines[3]  # Cuarta línea para CONTENT
                data["batch"] = lines[5]  # Sexta línea para BATCHILOT
                data["count"] = lines[7]  # Octava línea para COUNT
                data["weight"] = lines[9]  # Décima línea para NET WEIGHT

                # Procesar y normalizar el campo date
                raw_date = lines[11]  # Duodécima línea para USE BY
                date_parts = re.findall(r"\d+", raw_date)  # Extraer números
                if len(date_parts) == 3:
                    data["date"] = ".".join(date_parts)  # Normalizar con puntos
                else:
                    print("Advertencia: Formato de fecha inesperado, sin normalizar:", raw_date)
                    data["date"] = raw_date  # Mantener sin cambios si no se puede normalizar

            except IndexError:
                print("Error: No se encontraron suficientes líneas en el texto para mapear los tópicos.")
                return None

            # Verificar que todos los datos necesarios estén presentes
            for key, value in data.items():
                if not value:
                    print("Advertencia: Falta información para el tópico:", key)

            return data

        except Exception as e:
            print("Error al procesar el texto por líneas:", e)
            return None

    def publish_to_topics(self, data):
        """
        Publica los datos extraídos en los tópicos correspondientes.
        """
        try:
            # Simular publicación en tópicos
            print("\nTexto extraído:")
            print("  SSCC:", data['sscc'])
            print("  CONTENT", data['content'])
            print("  LOTE:", data['batch'])
            print("  COUNT:", data['count'])
            print("  WEIGHT:", data['weight'])
            print("  DATE:", data['date'])
        except Exception as e:
            print("Error al publicar en los tópicos:", e)


if __name__ == "__main__":
    input_dir = "/tmp/ocr_input"
    output_dir = "/tmp/ocr_output"

    # Crear directorios si no existen
    if not os.path.exists(input_dir):
        os.makedirs(input_dir)
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Inicializar el monitor de archivos
    event_handler = OCRHandler(input_dir, output_dir)
    observer = Observer()
    observer.schedule(event_handler, path=input_dir, recursive=False)
    observer.start()
    print("Iniciando el monitor de OCR con Tesseract en:", input_dir)

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        observer.stop()
    observer.join()
