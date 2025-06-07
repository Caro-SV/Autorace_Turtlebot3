# Importación de librerías necesarias
import cv2                          # OpenCV para procesamiento de imágenes
import numpy as np                 # NumPy para manejo de arrays
import rclpy                       # rclpy para crear nodos en ROS2
from rclpy.node import Node        # Clase base para crear un nodo ROS2
from rclpy.qos import QoSProfile, qos_profile_sensor_data  # Perfiles de calidad de servicio para suscripciones
from sensor_msgs.msg import Image  # Tipo de mensaje para imágenes
from cv_bridge import CvBridge, CvBridgeError  # Utilidad para convertir entre OpenCV y ROS

# Clase principal que define el nodo para calibración de perspectiva
class PerspectiveCalibrator(Node):
    def __init__(self):
        super().__init__('perspective_calibrator')  # Inicializa el nodo con nombre

        # Variables internas
        self.cvImage = None                          # Imagen actual capturada
        self.bridge = CvBridge()                     # Puente entre imagen ROS y OpenCV
        self.src_points = []                         # Lista de puntos seleccionados por el usuario
        self.selected_points = 0                     # Contador de puntos seleccionados
        self.max_points = 4                          # Número máximo de puntos requeridos
        self.transformed_image = None                # Imagen resultante después de la transformación
        self.waiting_for_image_logged = False        # Bandera para evitar mensajes repetidos

        # Crear suscripción al tópico de imagen
        qos = QoSProfile(depth=10)
        self.imageSub = self.create_subscription(
            Image, '/camera/image_raw', self.imageCallback, qos_profile=qos_profile_sensor_data
        )

        # Ventana para mostrar la imagen y capturar clics del mouse
        cv2.namedWindow('Raw Image', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback('Raw Image', self.mouse_callback)

    # Callback para recibir imágenes del tópico
    def imageCallback(self, msg):
        try:
            # Convierte la imagen del mensaje ROS a formato OpenCV
            self.cvImage = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error("¡No se pudo capturar la imagen de la cámara: %s" % str(e))

    # Método principal que muestra la imagen y espera interacción del usuario
    def opencv_display(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)

            # Verifica si la imagen está disponible
            if self.cvImage is None or self.cvImage.size == 0:
                if not self.waiting_for_image_logged:
                    self.get_logger().info("Esperando imagen de la cámara...")
                    self.waiting_for_image_logged = True
                cv2.waitKey(10)
                continue
            else:
                self.waiting_for_image_logged = False

            # Copia la imagen actual para dibujar sobre ella
            display_img = self.cvImage.copy()

            # Dibuja los puntos seleccionados
            for i, point in enumerate(self.src_points):
                point_int = (int(point[0]), int(point[1]))
                cv2.circle(display_img, point_int, 3, (0, 255, 0), -1)
                cv2.putText(display_img, f"{i + 1}", point_int, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 0, 0), 1)

            # Texto con instrucciones en pantalla
            instrucciones = [
                "Seleccione 4 puntos en orden:",
                "1. Superior Izquierda",
                "2. Superior Derecha",
                "3. Inferior Derecha",
                "4. Inferior Izquierda",
                "Presione 'a' para aplicar transformacion",
                "Presione 'q' para salir"
            ]

            # Parámetros para mostrar texto centrado
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.5
            thickness = 1
            color = (0, 255, 255)
            line_height = 20

            bloque_alto = len(instrucciones) * line_height
            start_y = int((display_img.shape[0] - bloque_alto) / 2)

            for i, texto in enumerate(instrucciones):
                (text_width, _), _ = cv2.getTextSize(texto, font, font_scale, thickness)
                x = int((display_img.shape[1] - text_width) / 2)
                y = start_y + i * line_height
                cv2.putText(display_img, texto, (x, y), font, font_scale, color, thickness)

            # Muestra la imagen original y transformada (si existe)
            cv2.imshow('Raw Image', display_img)
            if self.transformed_image is not None:
                cv2.imshow('Transformed Image', self.transformed_image)

            # Teclas para interacción
            key = cv2.waitKey(1) & 0xFF
            if key == ord('a') and self.selected_points == self.max_points:
                self.apply_transformation()  # Aplica transformación si ya hay 4 puntos
            elif key == ord('q'):
                self.get_logger().info("Cerrando el nodo...")
                break

    # Callback de eventos del mouse para seleccionar puntos en la imagen
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if self.selected_points < self.max_points:
                self.src_points.append([x, y])
                self.selected_points += 1
                self.get_logger().info(f"Punto seleccionado {self.selected_points}: ({x}, {y})")

    # Aplica la transformación de perspectiva con los puntos seleccionados
    def apply_transformation(self):
        if self.selected_points == self.max_points:
            height1, width1 = self.cvImage.shape[:2]
            self.dst_points = np.float32([
                [0, 0],
                [width1, 0],
                [width1, height1],
                [0, height1]
            ])
            src_points_array = np.float32(self.src_points)
            M = cv2.getPerspectiveTransform(src_points_array, self.dst_points)
            self.transformed_image = cv2.warpPerspective(self.cvImage, M, (width1, height1))
            self.save_values(src_points_array)

    # Guarda los puntos seleccionados y los puntos destino en un archivo de texto
    def save_values(self, src_points_array):
        with open("perspective_values.txt", "w") as file:
            for i in range(4):
                file.write(f"src{i}={src_points_array[i][0]},{src_points_array[i][1]}\n")
                file.write(f"dst{i}={self.dst_points[i][0]},{self.dst_points[i][1]}\n")
        self.get_logger().info("Valores de perspectiva guardados en perspective_values.txt")

# Función principal que inicializa el nodo y ejecuta el ciclo principal
def main(args=None):
    rclpy.init(args=args)
    calibrator = PerspectiveCalibrator()

    try:
        calibrator.opencv_display()
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

# Punto de entrada del programa
if __name__ == '__main__':
    main()