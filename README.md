<h1 align="center">Nodo de Detección de Carriles con OpenCV y ROS2</h1>

<p align="center">
  <i>Este nodo permite detectar líneas delimitadoras de carril y una linea roja de parada en una pista simulada usando visión por computadora con OpenCV y ROS2.</i>
</p>

<h2>Descripción</h2>
<p>El nodo <code>autorace_turtlebot</code> procesa imágenes capturadas por una cámara montada en un robot móvil. Detecta carriles para seguir el camino y líneas rojas que indican alto.</p>

<p>Este nodo usa OpenCV para filtrar colores y ROS2 para publicar comandos de movimiento en base a la detección.</p>
<p>En la imagen se muestra la pista de pruebas con la que se simula este nodo.</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/fd40699b-a204-4df4-9024-701d6c06271a" alt="Image">
</p>
<p><strong>Nota:</strong> El nodo puede ejecutarse en cualquier pista simulada de ROS2 y gazebo debido a sus caracteristicas.</p>

<h2>Funcionamiento General</h2>

<p>El nodo realiza lo siguiente:</p>
<ul>
  <li>Se suscribe a <code>/camera/image_raw</code> para recibir imágenes en tiempo real.</li>
  <li>Convierte las imágenes al espacio HSV y crea máscaras para líneas amarillas, blancas y rojas.</li>
  <li>Calcula centroides de las áreas detectadas y genera comandos de movimiento en <code>/cmd_vel</code>.</li>
</ul>

<p>Comportamiento del robot:</p>
<ul>
  <li>Si detecta una línea roja: <strong>Se detiene</strong>.</li>
  <li>Detecta las líneas blanca y amarilla para determinar el centro del carril: <strong>Sigue el carril</strong> ajustando la velocidad lineal y angular por medio de control PID.</li>
  <li>Calcula el radio de los carriles: <strong> Detecta curvas cerradas y tramos rectos</strong> Aumenta o disminuye la velocidad lineal dependiendo de la trayectoria.</li>
  <li>Si no detecta líneas: <strong>Se detiene</strong> por seguridad.</li>
</ul>

<h2>Instrucciones de Ejecución</h2>
<p>Agregar los nodos al archivo setup.py, cambiando el Nombre_paquete por el nombre de su paquete</p> 
<pre style="background-color:#e6ffed;padding:10px;border-radius:8px;">
<code>entry_points={
        'console_scripts': [
            'autorace_turtlebot = Nombre_paquete.autorace_turtlebot:main',
            'perpective_calibrator = Nombre_paquete.perpective_calibrator:main',
        ],
    },
</code></pre>
<p>Antes de correr el nodo principal se debe correr el nodo <code>perpective_calibrator.py</code> para ajustar la perspectiva de la cámara del robot seleccionando 4 puntos para la trasnformación, como se ve en la imagen.</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/330fc088-f07d-4ba6-8737-674b537d0d6e" alt="Image">
</p>
<p>Una vez terminado de seleccionar los puntos, el nodo generará un archivo .txt donde estarán las coordenadas que se deben colocar en esta sección del nodo <code>autorace_turtlebot</code></p>
<pre style="background-color:#e6ffed;padding:10px;border-radius:8px;">
<code>origen = np.float32([
            [107.0,3.0],    # Punto superior izquierdo
            [588.0,1.0],   # Punto superior derecho
            [717.0,367.0],  # Punto inferior derecho
            [1.0,377.0]      # Punto inferior izquierdo
        ])
destino = np.float32([
            [0.0, 0.0],        # Nuevo punto superior izquierdo
            [720.0,0.0],     # Nuevo punto superior derecho
            [720.0,480.0],  # Nuevo punto inferior derecho
            [0.0,480.0]      # Nuevo punto inferior izquierdo
        ])
</code></pre>

<h3>Pasos para iniciar Autorace</h3>
<p>Ejecuta los siguientes comandos desde una terminal de Ubuntu con el entorno ROS2 configurado:</p>
<p>1. Inicializar ROS2</p>
<pre style="background-color:#e6ffed;padding:10px;border-radius:8px;">
<code>cd ~/ros2_ws
colcon build
source install/setup.bash
</code></pre>

<p>2. Abrir Gazebo con la pista autorace</p>

<p>3. En otra terminal, ejecutar el nodo</p>
<pre style="background-color:#e6ffed;padding:10px;border-radius:8px;">
<code>ros2 run <NOMBRE_DEL_PAQUETE> autorace_turtlebot</code></pre>

<h2>Resultados</h2>
<p>Durante la ejecución se imprime en consola la velocidad lineal del robot, lo que permite observar como esta aumenta o disminuye conforme este se acerca a un curva como se ve en la imagen.</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/d62b9711-ddc9-4415-934a-69d80fde2bdb" alt="Image">
</p>

<p>Al tiempo se proyecta en una ventana la vision por computadora del robot donde se detalla la linea azul que identifica el centro del carril y las lineas delimitadoras del carril.</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/1ded62cf-f172-4afd-aef9-7ff4ea79798f" alt="Image">
</p>

<p>Finalmete un ejemplo del funcionamiento.</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/1a860bb9-a803-44a2-b5bf-6f1714ff0252" alt="Image">
</p>

<h2>Estructura del Nodo</h2>
<ul>
  <li><code>Image_callback</code>: Recibe las imágenes y procesa los colores para obtener máscaras.</li>
  <li><code>Cv_bridge</code>: Convierte los datos de ROS a formato OpenCV.</li>
  <li><code>Control de movimiento</code>: Calcula el error del centroide blanco respecto al centro de la imagen y ajusta la dirección.</li>
  <li><code>Detección de línea roja</code>: Si encuentra una máscara roja válida, el robot se detiene.</li>
</ul>
