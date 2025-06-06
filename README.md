<h1 align="center">Nodo de Detección de Carriles con OpenCV y ROS2</h1>

<p align="center">
  <i>Este nodo permite detectar líneas delimitadoras de carril y una linea roja de parada en una pista simulada usando visión por computadora con OpenCV y ROS2.</i>
</p>

<h2>Descripción</h2>
<p>El nodo <code>LaneDetectionNode</code> procesa imágenes capturadas por una cámara montada en un robot móvil. Detecta carriles para seguir el camino y líneas rojas que indican alto.</p>

<p>Este nodo usa OpenCV para filtrar colores y ROS2 para publicar comandos de movimiento en base a la detección.</p>
<p>En la imagen se muestra la pista de pruebas con la que se simula este nodo.</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/fd40699b-a204-4df4-9024-701d6c06271a" alt="Image">
</p>
<h2>Funcionamiento General</h2>

<p>El nodo realiza lo siguiente:</p>
<ul>
  <li>Se suscribe a <code>/camera/image_raw</code> para recibir imágenes en tiempo real.</li>
  <li>Convierte las imágenes al espacio HSV y crea máscaras para líneas blancas y rojas.</li>
  <li>Calcula centroides de las áreas detectadas y genera comandos de movimiento en <code>/cmd_vel</code>.</li>
</ul>

<p>Comportamiento del robot:</p>
<ul>
  <li>Si detecta una línea roja: <strong>Se detiene</strong>.</li>
  <li>Si detecta una línea blanca: <strong>Sigue el carril</strong> ajustando la dirección.</li>
  <li>Si no detecta líneas: <strong>Se detiene</strong> por seguridad.</li>
</ul>

<h2>Ejecución</h2>

<p><strong>1. Instalar la dependencia necesaria:</strong></p>

<pre style="background-color:#e8f5e9;padding:10px;border-radius:5px"><code>pip install opencv-python</code></pre>

<p><strong>2. Asegurarse que la simulación tenga una cámara activa en el tópico <code>/camera/image_raw</code>.</strong></p>

<p><strong>3. Compilar el paquete:</strong></p>

<pre style="background-color:#e8f5e9;padding:10px;border-radius:5px"><code>colcon build --packages-select &lt;nombre_paquete&gt;</code></pre>

<p><strong>4. Fuente el entorno:</strong></p>

<pre style="background-color:#e8f5e9;padding:10px;border-radius:5px"><code>source install/setup.bash</code></pre>

<p><strong>5. Ejecuta el nodo:</strong></p>

<pre style="background-color:#e8f5e9;padding:10px;border-radius:5px"><code>ros2 run &lt;nombre_paquete&gt; vision_lane_detection_node</code></pre>

<h2>Resultados</h2>
<p>Durante la ejecución se imprime en consola la velocidad lineal del robot, lo que permite observar como esta aumenta o disminuye conforme este se acerca a un curva como se ve en la imagen.</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/d62b9711-ddc9-4415-934a-69d80fde2bdb" alt="Image">
</p>

<p>Al tiempo se proyecta en una ventana la vision por computadora del robot donde se detalla la linea azul que identifica el centro del carril y las lineas delimitadoras del carril.</p>
<p align="center">
  <img src="https://github.com/user-attachments/assets/1ded62cf-f172-4afd-aef9-7ff4ea79798f" alt="Image">
</p>

<h2>Estructura del Nodo</h2>
<ul>
  <li><code>Image_callback</code>: Recibe las imágenes y procesa los colores para obtener máscaras.</li>
  <li><code>Cv_bridge</code>: Convierte los datos de ROS a formato OpenCV.</li>
  <li><code>Control de movimiento</code>: Calcula el error del centroide blanco respecto al centro de la imagen y ajusta la dirección.</li>
  <li><code>Detección de línea roja</code>: Si encuentra una máscara roja válida, el robot se detiene.</li>
</ul>
