import cv2, numpy as np, rclpy, warnings
from threading import Timer # Libreria para detener el robot despues de un tiempo
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

class Camara(Node):
    def __init__(self):
        super().__init__('autorace_turtlebot')
        # Inicializar variables y configuraciones iniciales
        self.imagen = []  # Variable para almacenar la imagen recibida
        self.puente = CvBridge()  # Puente entre ROS y OpenCV
        self.estado_inicial = False  # Indica si ya se ha recibido una imagen
        self.ajuste_carril_izquierdo = None  # Coeficientes del polinomio ajustado al carril izquierdo
        self.ajuste_carril_derecho = None  # Coeficientes del polinomio ajustado al carril derecho
        self.ultimo_angular_z = 0.0  # Almacena el último comando angular válido
        self.curva_cerrada_detectada = False  # Indicador de detección de curva cerrada

        # Variables para el manejo del error del control PID
        self.error_previo = 0.0 # Guarda el error anterior del control 
        self.error_suavizado = 0.0 # Filtra el error actual
        self.error_acumulado = 0.0 # Alacena el error acumulado
        self.tiempo_previo = self.get_clock().now() # Guarda el tiempo del último ciclo de control

        # Variables para control del robot al detectar la linea roja
        self.detener_robot = False # Bandera que indica si el robot debe detenerse completamente
        self.detecto_linea_roja = False # Bandera que indica si el robot ya detectó la línea roja
        self.en_giro_post_rojo = False # Bandera que indica si el robot está girando luego de detectar la línea roja

        # Configuración de QoS para la transmisión de video
        perfil_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Política de confiabilidad
            history=HistoryPolicy.KEEP_LAST,  # Mantener los últimos mensajes
            depth=10  # Profundidad de la cola
        )

        # Crear suscriptor para el tópico de la cámara
        self.suscriptor = self.create_subscription(Image, '/camera/image_raw', self.visualizar_imagen, perfil_qos)

        # Crear publicador para enviar comandos de velocidad
        self.publicador_velocidad = self.create_publisher(Twist, '/cmd_vel', 10)

        # Crear un temporizador para actualizar el procesamiento de la imagen
        self.temporizador_actualizacion = self.create_timer(0.010, self.callback_actualizacion)

        self.get_logger().info("Uso de la cámara para el seguimiento de carril")

    # Método para recibir y procesar la imagen de la cámara
    def visualizar_imagen(self, msg):
        self.imagen = msg  # Almacenar el mensaje de imagen
        self.cv_imagen = self.puente.imgmsg_to_cv2(self.imagen, "bgr8")  # Convertir a formato OpenCV
        self.estado_inicial = True  # Indicar que ya se ha recibido una imagen

    # Callback que se ejecuta periódicamente para procesar la imagen
    def callback_actualizacion(self):
        if self.estado_inicial:
            self.detectar_carril()  # Procesar la imagen para detectar el carril

    # Método para aplicar una transformación de perspectiva a la imagen
    def aplicar_transformacion_perspectiva(self, img):
        alto, ancho = img.shape[:2]

        # Definir puntos de origen (src) y destino (dst) para la transformación
        origen = np.float32([
            [107.0,3.0],    # Punto superior izquierdo
            [588.0,1.0],   # Punto superior derecho
            [717.0,367.0],  # Punto inferior derecho
            [1.0,377.0]      # Punto inferior izquierdo
        ])

        # Puntos de destino para "enderezar" la perspectiva
        destino = np.float32([
            [0.0, 0.0],        # Nuevo punto superior izquierdo
            [720.0,0.0],     # Nuevo punto superior derecho
            [720.0,480.0],  # Nuevo punto inferior derecho
            [0.0,480.0]      # Nuevo punto inferior izquierdo
        ])

        M = cv2.getPerspectiveTransform(origen, destino) # Obtener la matriz de transformación de perspectiva
        imagen_transformada = cv2.warpPerspective(img, M, (ancho, alto)) # Aplicar la transformación a la imagen
        return imagen_transformada  # Retornar la imagen transformada

    # Método principal para detectar el carril en la imagen
    def detectar_carril(self):
        imagen_cortada = self.cv_imagen  # Usar la imagen capturada

        # Aplicar la transformación de perspectiva
        imagen_transformada = self.aplicar_transformacion_perspectiva(imagen_cortada)

        # Definir una región de interés (ROI) en la parte inferior de la imagen para anticipar curvas
        alto, ancho = imagen_transformada.shape[:2]
        parte_superior_roi = int(alto * 0.8)  # Definir el límite superior del ROI 
        parte_inferior_roi = alto             # Límite inferior del ROI es el final de la imagen
        izquierda_roi = 0                     # Límite izquierdo del ROI
        derecha_roi = ancho                   # Límite derecho del ROI
        imagen_roi = imagen_transformada[parte_superior_roi:parte_inferior_roi, izquierda_roi:derecha_roi]  # Extraer el ROI
        self.img_roi= imagen_roi

        # Convertir la imagen del ROI a espacio de color HSV
        imagen_hsv = cv2.cvtColor(imagen_roi, cv2.COLOR_BGR2HSV)

        # Definir el rango de color amarillo en HSV para detectar líneas amarillas
        amarillo_bajo = np.array([20, 100, 100])
        amarillo_alto = np.array([30, 255, 255])
        # Definir el rango de dos tonos de rojo en HSV para detectar la linea roja 
        bajo_rojo = np.array([1, 100, 20])
        alto_rojo = np.array([0, 255, 255])
        bajo_rojo2 = np.array([175, 100, 20])
        alto_rojo2 = np.array([179, 255, 255])

        
        mascara_amarilla = cv2.inRange(imagen_hsv, amarillo_bajo, amarillo_alto) # Crear una máscara para el color amarillo
        maskRed1 = cv2.inRange(imagen_hsv, bajo_rojo, alto_rojo) # Crear una máscara para el tono 1 del color rojo 
        maskRed2 = cv2.inRange(imagen_hsv, bajo_rojo2, alto_rojo2) # Crear una máscara para el tono 2 del color rojo 
        maskRed = cv2.add(maskRed1, maskRed2) # Combinar ambos tonos de rojo

        imagen_gris = cv2.cvtColor(imagen_roi, cv2.COLOR_BGR2GRAY) # Convertir la imagen a escala de grises

        # Aplicar umbralización binaria para resaltar las líneas blancas
        _, imagen_binaria = cv2.threshold(imagen_gris, 150, 255, cv2.THRESH_BINARY)

        mascara_combinada = cv2.bitwise_or(imagen_binaria, mascara_amarilla) # Combinar la máscara amarilla y la imagen binaria
        mascara_comb_roja = cv2.bitwise_and(imagen_binaria, maskRed) # Combinar la máscara roja y la imagen binaria
        self.max_mascara = np.max(mascara_comb_roja) # Determina el valor maximo de la matriz de la mascara roja

        # Aplicar una operación morfológica de cierre para eliminar pequeños agujeros en los objetos
        kernel = np.ones((5, 5), np.uint8)
        imagen_cerrada = cv2.morphologyEx(mascara_combinada, cv2.MORPH_CLOSE, kernel)
        self.img_cer = imagen_cerrada # Se guarda la imagen cerrada para usarla posteriormente 

        # Calcular el histograma vertical de la mitad inferior de la imagen
        histograma = np.sum(imagen_cerrada[imagen_cerrada.shape[0]//2:, :], axis=0)

        # Encontrar los picos del histograma que corresponden a las posiciones base de los carriles
        punto_medio = np.int(histograma.shape[0] // 2)
        base_izquierda = np.argmax(histograma[:punto_medio])          # Posición base del carril izquierdo
        base_derecha = np.argmax(histograma[punto_medio:]) + punto_medio  # Posición base del carril derecho

        # Configurar los parámetros de las ventanas deslizantes
        num_ventanas = 5                                   # Número de ventanas en las que se dividirá el eje vertical
        altura_ventana = np.int(imagen_cerrada.shape[0] // num_ventanas)  # Altura de cada ventana
        margen = 50                                        # Ancho horizontal de las ventanas
        min_pixeles = 50                                   # Número mínimo de píxeles para recentrar las ventanas

        # Inicializar las posiciones actuales de los carriles para el primer nivel
        carril_izquierdo_actual = base_izquierda
        carril_derecho_actual = base_derecha

        # Listas para almacenar los índices de los píxeles que pertenecen a cada carril
        indices_carril_izquierdo = []
        indices_carril_derecho = []

        # Identificar los píxeles no cero en la imagen procesada
        no_ceros = imagen_cerrada.nonzero()
        no_ceros_y = np.array(no_ceros[0])
        no_ceros_x = np.array(no_ceros[1])

        ajuste_izquierdo = None  # Para almacenar el ajuste polinomial del carril izquierdo
        ajuste_derecho = None    # Para almacenar el ajuste polinomial del carril derecho

        # Iterar sobre cada ventana
        for ventana in range(num_ventanas):
            # Definir los límites de las ventanas
            limite_y_bajo = imagen_cerrada.shape[0] - (ventana + 1) * altura_ventana
            limite_y_alto = imagen_cerrada.shape[0] - ventana * altura_ventana
            limite_x_izquierdo_bajo = carril_izquierdo_actual - margen
            limite_x_izquierdo_alto = carril_izquierdo_actual + margen
            limite_x_derecho_bajo = carril_derecho_actual - margen
            limite_x_derecho_alto = carril_derecho_actual + margen

            # Identificar los píxeles no cero dentro de las ventanas
            buenos_indices_izquierdo = ((no_ceros_y >= limite_y_bajo) & (no_ceros_y < limite_y_alto) &
                                        (no_ceros_x >= limite_x_izquierdo_bajo) & (no_ceros_x < limite_x_izquierdo_alto)).nonzero()[0]
            buenos_indices_derecho = ((no_ceros_y >= limite_y_bajo) & (no_ceros_y < limite_y_alto) &
                                      (no_ceros_x >= limite_x_derecho_bajo) & (no_ceros_x < limite_x_derecho_alto)).nonzero()[0]

            # Agregar los índices encontrados a las listas
            indices_carril_izquierdo.append(buenos_indices_izquierdo)
            indices_carril_derecho.append(buenos_indices_derecho)

            # Si se encuentran suficientes píxeles, recentrar la ventana para el siguiente nivel
            if len(buenos_indices_izquierdo) > min_pixeles:
                carril_izquierdo_actual = np.int(np.mean(no_ceros_x[buenos_indices_izquierdo]))
            if len(buenos_indices_derecho) > min_pixeles:
                carril_derecho_actual = np.int(np.mean(no_ceros_x[buenos_indices_derecho]))

        # Concatenar los índices de todos los niveles
        indices_carril_izquierdo = np.concatenate(indices_carril_izquierdo)
        indices_carril_derecho = np.concatenate(indices_carril_derecho)

        # Extraer las coordenadas de los píxeles que pertenecen a cada carril
        carril_izquierdo_x = no_ceros_x[indices_carril_izquierdo]
        carril_izquierdo_y = no_ceros_y[indices_carril_izquierdo]
        carril_derecho_x = no_ceros_x[indices_carril_derecho]
        carril_derecho_y = no_ceros_y[indices_carril_derecho]

        # Ajustar un polinomio de segundo grado (parábola) a los puntos de cada carril
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', np.RankWarning)  # Ignorar advertencias si el ajuste no es posible
            if len(carril_izquierdo_x) > 0 and len(carril_izquierdo_y) > 0:
                ajuste_izquierdo = np.polyfit(carril_izquierdo_y, carril_izquierdo_x, 2)
                self.ajuste_carril_izquierdo = np.polyval(ajuste_izquierdo, np.linspace(0, imagen_cerrada.shape[0] - 1, imagen_cerrada.shape[0]))

            if len(carril_derecho_x) > 0 and len(carril_derecho_y) > 0:
                ajuste_derecho = np.polyfit(carril_derecho_y, carril_derecho_x, 2)
                self.ajuste_carril_derecho = np.polyval(ajuste_derecho, np.linspace(0, imagen_cerrada.shape[0] - 1, imagen_cerrada.shape[0]))

        # Detectar si hay una curva cerrada, suave o una recta
        self.curva_cerrada_detectada = self.detectar_curva_cerrada()
        if ajuste_izquierdo is not None and ajuste_derecho is not None:
            self.dibujar_carriles_detectados(imagen_roi, ajuste_izquierdo, ajuste_derecho)

        # Mostrar la imagen con los carriles detectados
        final = cv2.resize(imagen_roi, (800, 400))          # Redimensionar para mostrar
        cv2.imshow("Imagen con Curvas Ajustadas", final)    # Mostrar la imagen
        cv2.waitKey(1)                                      # Esperar un milisegundo para actualizar la ventana

        # Controlar el robot en base a la detección de los carriles
        self.controlar_robot(imagen_roi)

    # Método para dibujar los carriles detectados en la imagen
    def dibujar_carriles_detectados(self, img, ajuste_izquierdo, ajuste_derecho):
        # Crear una imagen en negro con las mismas dimensiones para dibujar los carriles
        img_carril = np.zeros_like(img)

        # Generar los puntos de la curva para cada carril
        ploty = np.linspace(0, img.shape[0] - 1, img.shape[0])  # Coordenadas Y
        try:
            ajuste_izquierdo_x = np.polyval(ajuste_izquierdo, ploty)  # Coordenadas X del carril izquierdo
            ajuste_derecho_x = np.polyval(ajuste_derecho, ploty)      # Coordenadas X del carril derecho
        except TypeError:
            return  # Si no hay suficientes datos, salir del método

        # Crear matrices de puntos para dibujar las líneas del carril
        puntos_linea_izquierda = np.array([np.transpose(np.vstack([ajuste_izquierdo_x, ploty]))])
        puntos_linea_derecha = np.array([np.flipud(np.transpose(np.vstack([ajuste_derecho_x, ploty])))])

        # Combinar los puntos de ambos carriles
        puntos_carril = np.hstack((puntos_linea_izquierda, puntos_linea_derecha))

        # Rellenar el área entre los carriles
        cv2.fillPoly(img_carril, np.int_([puntos_carril]), (0, 255, 0))  # Color verde

        # Dibujar el centro del carril
        centro_carril_x = (ajuste_izquierdo_x + ajuste_derecho_x) / 2  # Coordenadas X del centro
        cv2.line(img_carril, (int(centro_carril_x[-1]), img.shape[0]), (int(centro_carril_x[0]), 0), (255, 0, 0), 5)  # Línea azul

        # Superponer las líneas y áreas en la imagen original
        img_combinada = cv2.addWeighted(img, 1, img_carril, 0.5, 0)
        np.copyto(img, img_combinada)  # Actualizar la imagen original
    
    def calcular_radio_curvatura(self, ajuste, y_eval): # Método para calcular el radio de curvatura del carril 
        if ajuste is None:
            return float('inf')  # Si no hay ajuste, establecer curvatura infinita (recta)
        A, B, C = ajuste[:3]  # Coeficientes de la ecuación cuadrática 
        curvatura = ((1 + (2*A*y_eval + B)**2) ** 1.5) / abs(2*A) # Ecuacion para el calculo del radio de la curvatura
        return curvatura
    
    def detectar_curva_cerrada(self): # Método para detectar si el robot está entrando en una curva cerrada 
        if self.ajuste_carril_izquierdo is not None and self.ajuste_carril_derecho is not None:
            y_eval = self.img_cer.shape[0]  # Evaluamos la curvatura en la parte inferior de la imagen
            radio_izquierdo = self.calcular_radio_curvatura(self.ajuste_carril_izquierdo, y_eval) # Se calcula el radio del carril izquierdo
            radio_derecho = self.calcular_radio_curvatura(self.ajuste_carril_derecho, y_eval) # Se calcula el radio del carril
            
            self.radio_prom = (min(radio_izquierdo , radio_derecho)) / 1000000000  # Se encuentra el radio menor entre ambos carriles 

            umbral_Cerrado = 200  # Umbral para curvas Cerradas
            umbral_recta = 10 # Umbral para rectas
            
            if self.radio_prom < umbral_recta: # Rectas
                curva = 0
            elif umbral_recta < self.radio_prom < umbral_Cerrado: # Curvas suaves
                curva = 1
            else: # Curvas cerradas
                curva = 2 
        return curva  # Devuelve el valor de la curva segun su tipo
    
    def Deteccion_linea_roja(self): # Método para la deteccion de la linea roja 
        vel = Twist()

        if self.detener_robot: # Si esta es TRUE se detiene el robot
            vel.linear.x, vel.angular.z = 0.0, 0.0
            self.publicador_velocidad.publish(vel)
            return True
 
        if self.en_giro_post_rojo: # Gira un poco a la izquierda para terminar el giro
            vel.linear.x, vel.angular.z = 0.05, 0.5
            self.publicador_velocidad.publish(vel)
            return True

        if self.detecto_linea_roja: # Avanza en linea recta hacia la linea
            vel.linear.x, vel.angular.z = 0.2, 0.0
            self.publicador_velocidad.publish(vel)
            return True

        if 200 < self.max_mascara <= 255 and not self.detecto_linea_roja: # Si se detecta la linea roja con la mascara y no se ha detectado antes
            self.en_giro_post_rojo = True # Bandera para terminar el giro

            def terminar_giro():
                self.en_giro_post_rojo = False # Deja de girar
                self.detecto_linea_roja = True # Ya se detectó la linea roja

                def detener_robot_final():
                    self.detener_robot = True # Detiene completamente el robot
                    self.get_logger().info("Ruta completada con Éxito! 🚘😎🎉")
                Timer(2.0, detener_robot_final).start() # Timer 2 para detener completamente el robot

            Timer(1.0, terminar_giro).start() # Timer 1 para terminar el movimiento

            vel.linear.x, vel.angular.z = 0.05, 0.5 # Se establece las velocidades para terminar el movimiento
            self.publicador_velocidad.publish(vel)
            return True
        return False
    
    def controlar_robot(self, img): # Método para controlar el robot en base a la detección del carril 
        vel = Twist()

        if self.Deteccion_linea_roja(): # Condicional para cuando se detecta la linea roja
            return
        
        x_izq_eval = float(self.ajuste_carril_izquierdo[-1]) # Se obtiene el último punto del ajuste polinomial del carril izquierdo
        x_der_eval = float(self.ajuste_carril_derecho[-1]) # Se obtiene el último punto del ajuste polinomial del carril derecho
        x_centro_eval = (x_izq_eval + x_der_eval) / 2 # Se calcula el centro entre ambos carriles
        x_centro_imagen = img.shape[1] // 2 # Se halla el centro de la imagen
        error = x_centro_eval - x_centro_imagen # Se calcula el error de posición entre el centro del carril y el centro de la imagen

        tiempo_actual = self.get_clock().now() # Obtiene el tiempo actual
        delta_t = (tiempo_actual - self.tiempo_previo).nanoseconds / 1e9  or 1e-6 # Calcula el tiempo transcurrido entre el instante actual y el anterior en seg, evitando el cero
        self.tiempo_previo = tiempo_actual # Actualiza el tiempo anterior con el actual

        # Variables Controlador PID
        alpha = 0.9 # Coeficiente de suavizado
        self.error_suavizado = alpha * self.error_suavizado + (1 - alpha) * error # Filtro exponencial para evitar movimientos bruscos de control Proporcional
        self.error_acumulado += self.error_suavizado * delta_t # Control Integral
        derivada_error = (self.error_suavizado - self.error_previo) / delta_t # Control Derivativo
        self.error_previo = self.error_suavizado # Guarda el valor actual del error

        tipo_curva = self.curva_cerrada_detectada 
        if tipo_curva == 0: 
            kP, kI, kD = 0.009, 0.0001, 0.0001 # Parametros para linea recta
        elif tipo_curva == 1:
            kP, kI, kD = 0.015, 0.0002, 0.0002 # Parametros para curva abierta
        elif tipo_curva == 2:
            kP, kI, kD = 0.030, 0.0003, 0.0003 # Parametros para curva cerrada

        velocidad_lineal = max(0.1, min(0.23, 1.0 / float(self.radio_prom))) # Velocidad lineal del robot segun el radio de curvatura

        pid = float(kP * self.error_suavizado + kI * self.error_acumulado + kD * derivada_error) # Controlador PID

        if self.ajuste_carril_izquierdo is not None and self.ajuste_carril_derecho is None: # Si se detecta el carril izquierdo, gira a la izquierda centrado
            vel.angular.z = pid
        elif self.ajuste_carril_derecho is not None and self.ajuste_carril_izquierdo is None: # Si se detecta el carril derecho, gira a la derecha centrado
            vel.angular.z = -pid
        elif self.ajuste_carril_izquierdo is not None and self.ajuste_carril_derecho is not None: # Si se detectan ambos carriles
            centro_carril_x = np.mean([self.ajuste_carril_izquierdo, self.ajuste_carril_derecho], axis=0) # Vuelve a calcular el centro del carril
            centro_carril = float(centro_carril_x[-1]) # Guarda el ultimo punto detectado
            self.centro = centro_carril
            error = (self.img_roi.shape[1] / 2.0) - centro_carril # Calcula el error del centro del carril con el de la imagen
            self.error_acumulado += error * delta_t # Control integral
            derivada_error = (error - self.error_previo) / delta_t # Control Derivativo
            self.error_previo = error # Guarda el valor actual del error
            pid = float(kP * error + kI * self.error_acumulado + kD * derivada_error) # Controlador PID para las rectas
            vel.angular.z = pid # La velocidad angular toma el valor del PID

        self.get_logger().info(f"Vel_Lineal: {velocidad_lineal}")
        vel.linear.x = velocidad_lineal # Se actualiza la velocidad lineal segun el caso
        self.publicador_velocidad.publish(vel) # Publica la velocidad

def main(args=None):
    rclpy.init(args=args)          # Inicializar ROS 2
    nodo = Camara()                # Crear instancia del nodo Camara
    rclpy.spin(nodo)               # Mantener el nodo en ejecución
    nodo.destroy_node()            # Destruir el nodo al finalizar
    rclpy.shutdown()               # Apagar ROS 2
    cv2.destroyAllWindows()        # Cerrar las ventanas de OpenCV

if __name__ == '__main__':
    main()  # Ejecutar la función principal