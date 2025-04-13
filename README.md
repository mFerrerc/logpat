# LOGPAT (Logistic Patrol)
Este proyecto aborda la automatización de un almacén logístico empleando un robot móvil SUMMIT-XL STEEL. 

![image](https://github.com/user-attachments/assets/fa293287-b242-4665-8197-886c804cda9c)

La finalidad principal es que el robot pueda, de forma autónoma, registrar la entrada de nuevos paquetes, patrullar el entorno para monitorizar el estado de los productos almacenados y, finalmente, retornar a su base de carga. De este modo, se optimiza la gestión de inventarios, fusionando la potencia de la robótica móvil y la comunicación en tiempo real con el sistema maestro de la empresa.

La iniciativa se desarrolló íntegramente sobre el framework ROS, simulando la operación en un entorno virtual de Gazebo. Aun así, cada parte del flujo se concibió para llevarse posteriormente a un sistema real sin grandes complicaciones. Durante el patrullaje, se incluyen capturas de datos de sensores (temperatura, humedad), recibidos a través de beacons BLE instalados en cada paquete, y la información generada se almacena y gestiona en una base de datos Supabase, que facilita la actualización y consulta de registros desde cualquier nodo del sistema.

## Descripción General
Para controlar el flujo de tareas, se diseñó una máquina de estados que asegura una ejecución clara y ordenada:

1. El robot parte desde su base y espera el comando correspondiente.

2. Si se indica “etiquetado”, se dirige a la zona de recepción de mercancías y procede a leer las etiquetas (OCR), registrándolas en la base de datos.

3. Tras concluir la lectura, el robot vuelve a la base o, si recibe la orden, inicia una patrulla por las diferentes secciones del almacén.

4. Durante la patrulla, el robot comprueba los datos enviados por los beacons BLE (temperatura y humedad) para detectar anomalías en los productos.

5. Al terminar el recorrido, retorna de nuevo a la base y espera un nuevo ciclo de órdenes.

![image](https://github.com/user-attachments/assets/440b8f15-ba14-4635-bce5-763486ab7ab0)

La aproximación se diseñó para un entorno logístico que exige inmediatez y precisión en la detección de incidencias. Gracias a los beacons BLE, se pueden leer datos en tiempo real y, de existir un valor fuera de rango, notificar al sistema maestro o al personal de almacén. Esto garantiza un control preventivo, evitando la distribución de mercancía dañada.

![image1](https://github.com/user-attachments/assets/71e12f5b-65c6-42db-8804-913a17ce8517)![image](https://github.com/user-attachments/assets/94742cd2-1938-49bb-a43c-479749829261)

## Elementos Destacados
### Simulación en Gazebo
Para comprobar la viabilidad del proyecto, se construyó un entorno virtual que refleja un almacén real, con sus estanterías y áreas de almacenamiento. El robot SUMMIT-XL STEEL, así como los objetos que lleva cada beacon, se añadieron al mundo de Gazebo. Con esta aproximación, se validaron las rutas de patrulla, los movimientos de vuelta a la base y las lecturas de etiquetas sin necesidad de arriesgar o retrasar la implantación por espera de componentes físicos.

### Visión Artificial y OCR
El proceso de lectura de etiquetas se resolvió con Tesseract OCR, precedido de un preprocesado de la imagen capturada por la cámara RGB del robot. Se recortó la zona de la etiqueta en función de parámetros de color, se convirtieron las imágenes a escala de grises y se limpiaron ruidos para mejorar la extracción de texto. Posteriormente, se obtuvieron datos clave como el GTIN, el número de lote o el peso. Toda esa información fue dirigida a la base de datos, haciendo más sencillo el alta de nuevos productos.

### Beacons BLE
Cada paquete cuenta con un pequeño beacon BLE que emite datos de temperatura y humedad. Para simular esta parte, se diseñó un nodo ROS que genera en tiempo real los paquetes de datos en formato hexadecimal. El robot los recibe cuando patrulla la zona y comprueba en la base de datos si los valores están en rango y si el paquete asociado se encuentra en su ubicación correcta. De no ser así, el sistema marca la incidencia y lanza alertas que facilitan su localización.

### Base de Datos en Supabase
La base de datos empleada está alojada en la nube mediante Supabase, que ofrece servicios basados en PostgreSQL con una API sencilla para altas, bajas y modificaciones de registros. Con ello, cualquier nodo ROS puede crear entradas nuevas o actualizar campos existentes en el inventario, sin necesidad de instalar un sistema de bases de datos complejo de manera local. Además, el proyecto utiliza diversos scripts para gestionar, corregir y visualizar la información, así como para eliminar marcadores de error cuando el problema ha sido solventado.

### Tecnologías Empleadas
- **Robot Operating System (ROS):** Orquestación de nodos, comunicación de tópicos y acciones de navegación.

- **Gazebo + RViz:** Entorno de simulación realista y visualización en 3D de la escena y los sensores.

- **OpenCV:** Preprocesado de imágenes (recorte y limpieza) para luego aplicar el OCR.

- **Tesseract OCR:** Extracción de texto relevante de las etiquetas.

- **Beacons BLE (simulados):** Generación de paquetes de datos en formato hexadecimal, probando la lectura de sensores (temperatura, humedad).

- **Supabase (PostgreSQL):** Almacenamiento en la nube de la información de cada lote y consulta concurrente desde distintos nodos.

- **Python:** Desarrollo de los nodos ROS, scripts de OCR y enlace con la base de datos.

- **Docker / Conda (opcional):** Facilitan la consistencia de entornos, especialmente por las versiones de Python y librerías (psycopg2, supabase-py, etc.).

## Cómo Funciona la Aplicación
A un nivel alto, el robot escucha órdenes de acción (p. ej., “etiquetado” o “patrulla”), transicionando por los estados de su máquina de estados. Para registrar nuevos paquetes, se posiciona en el área de etiquetado y ejecuta sus scripts de OCR, enviando la información obtenida a la base de datos. Si se selecciona modo “patrulla”, recorre la secuencia de waypoints en Gazebo, simulando la lectura de beacons BLE. Si se detectan errores (por ejemplo, temperatura fuera de rango), se actualiza el campo “estado” del producto en la base de datos y se publican marcadores en RViz. Finalizada la patrulla, el robot retorna a base, quedando listo para repetir el ciclo cuando se precise.

En paralelo, cualquier operador humano puede consultar y modificar la información en Supabase (mediante la interfaz web o llamando a scripts en Python). Igualmente, el sistema está concebido para que, en un despliegue físico, los componentes BLE y las cámaras del robot se sustituyan sin grandes cambios, manteniendo la misma lógica de control y la misma base de datos compartida.

## Demostración en Vídeo
En el siguiente enlace se ilustra el comportamiento global de la aplicación, abarcando lectura de etiquetas, patrulla, detección de valores fuera de rango y actualización de la base de datos:

https://www.youtube.com/watch?v=PkZ8wWkjVw4

##
Este proyecto, al unir la robustez de la navegación autónoma con la integración en la nube de Supabase, persigue un flujo logístico más inteligente y seguro, donde cada paquete se registra y monitoriza en tiempo real sin intervención manual permanente. De este modo, se concreta una telemetría continua que, combinada con la capacidad de corrección en caso de incidencias, mejora la calidad y fiabilidad de la cadena de suministros.

## Cargar Docker

Se ha preparado una imagen de Docker para unificar librerías y facilitar la puesta en marcha. Para cargar la imagen del Docker y lanzar el contenedor, ejecuta:
```
docker run -it --name=proyecto_robots_moviles \
  --network host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  --privileged \
  proyecto_robots_moviles:1.0
```
En caso de que necesites abrir más terminales dentro del contenedor, es suficiente con:
```
docker exec -it proyecto_robots_moviles /bin/bash
```

De este modo, podrás lanzar nodos de ROS en diferentes terminales sin cerrar el contenedor principal.