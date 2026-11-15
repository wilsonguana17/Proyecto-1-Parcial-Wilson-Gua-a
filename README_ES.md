# 🏎️ F1TENTH Sepang | Follow the Gap reactivo V3.1

[English](README.md) · **Español**  
**ROS 2 Humble · Ubuntu 22.04 · LiDAR · Dirección Ackermann**

Controlador reactivo para recorrer el circuito **Sepang** en el simulador F1TENTH, **sin una trayectoria predefinida**. Incluye un **cronómetro de 10 vueltas**, el mapa de Sepang y un script de configuración. El controlador decide hacia dónde conducir usando el LiDAR; la odometría permite supervisar el movimiento del vehículo.

## 🎥 Demostración de 10 vueltas

[![Ver la demostración de 10 vueltas en Sepang](https://img.youtube.com/vi/9m-QhL65yLw/hqdefault.jpg)](https://youtu.be/9m-QhL65yLw)

**[▶ Ver en YouTube](https://youtu.be/9m-QhL65yLw)** — Haz clic en la miniatura para abrir el video.

## 💡 ¿Cómo funciona Follow the Gap?

1. **Lectura del LiDAR:** `preprocess_lidar()` toma el sector frontal ±90°, trata lecturas inválidas, limita las distancias a 4.5 m y suaviza con una ventana de 5 muestras.
2. **Burbuja de seguridad:** `choose_gap()` anula una región alrededor del obstáculo más cercano, con un **radio de 0.60 m**.
3. **Detección de espacio libre:** busca regiones consecutivas con distancia procesada mayor que **1.20 m** y selecciona el gap más ancho.
4. **Selección del objetivo:** combina **60 % del punto más despejado** del gap con **40 % de su centro**.
5. **Dirección y velocidad:** `scan_callback()` limita y suaviza el giro, y restringe la velocidad según la curva, la aceleración lateral y la distancia frontal.
6. **Ajuste de velocidad en rectas:** si el ángulo de dirección, la velocidad angular y una odometría reciente indican condiciones estables, el controlador comprueba un pasillo local con LiDAR y puede aumentar de forma limitada la velocidad frontal. Si pierde confianza o detecta poco espacio, mantiene el límite de frenado conservador.
7. **Movimiento:** publica un mensaje `AckermannDriveStamped` en `/drive`.

**Flujo ROS 2:** `/scan` + `/ego_racecar/odom` → **Follow the Gap V3.1** → `/drive`. El cronómetro solo utiliza odometría: no envía comandos de movimiento.

## 🧩 Explicación del código

Archivo principal: [`controllers/gap_follow_sepang_v31_en.py`](controllers/gap_follow_sepang_v31_en.py)

### Comunicación ROS 2

El nodo recibe los datos del LiDAR y la odometría, y publica los comandos Ackermann:

~~~python
self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
self.odom_sub = self.create_subscription(Odometry, '/ego_racecar/odom', self.odom_callback, 10)
self.drive_pub = self.create_publisher(AckermannDriveStamped, '/drive', 10)
~~~

### Funciones principales

| Función o etapa | Responsabilidad |
|---|---|
| `preprocess_lidar()` | Limpia, recorta y suaviza las mediciones del LiDAR |
| `choose_gap()` | Aplica la burbuja, detecta el gap más ancho y escoge el objetivo |
| Estimación frontal | Obtiene una distancia de referencia con el percentil 20 de las lecturas válidas del LiDAR dentro de ±12° |
| `corridor_clearance()` | Estima el recorrido libre dentro de un pasillo local de anchura finita |
| `confidence_component()` | Calcula la confianza para mejorar la velocidad en rectas |
| `odom_callback()` | Actualiza velocidad real, velocidad angular y posición |
| `scan_callback()` | Calcula giro, restricciones, mejora en rectas, aceleración y publicación en `/drive` |

La dirección **no se elige solo por el centro del gap**. El fragmento real de `choose_gap()` es:

~~~python
best_rel = int(np.argmax(processed[gap_start:gap_end + 1]))
center = (gap_start + gap_end) // 2
return int(0.6 * (gap_start + best_rel) + 0.4 * center)
~~~

El ángulo elegido se limita a **±0.4189 rad** (aproximadamente 24°). Los comandos de dirección sucesivos se suavizan con `steer_alpha = 0.65`.

**Cálculo de velocidad:** se compara la velocidad máxima configurada con los límites por giro, curva y distancia frontal. El menor determina la velocidad objetivo. Cuando hay suficiente espacio y confianza en una recta, el análisis del pasillo puede aumentar el límite frontal hasta `front_boost_max × confidence`, sin anular las demás restricciones. Después se limitan los cambios de aceleración y frenado.

### ¿Cómo calcula el controlador la velocidad?

Dentro de `scan_callback()` se calculan **tres límites independientes**: uno disminuye la velocidad cuando el volante gira mucho (`speed_original`), otro considera la aceleración lateral admisible en curvas (`speed_curve`) y el tercero estima una velocidad de frenado según la distancia frontal disponible. A continuación se muestran dos de las expresiones empleadas:

~~~python
speed_original = max(self.MIN_SPEED, self.max_speed * (
    1.0 - (abs(steering) / self.MAX_STEER_ANGLE) ** 1.8
))
speed_curve = math.sqrt(
    self.lateral_accel_max * self.WHEELBASE_M /
    max(abs(math.tan(steering)), 1e-5)
)
~~~

El **límite por frenado frontal** se calcula a partir de la distancia libre del LiDAR, descontando un margen: `v_frontal = √(2 × desaceleración_frenado × max(0, distancia_frontal − margen))`. Así, el vehículo recibe un límite menor cuando dispone de menos espacio para detenerse.

El controlador también utiliza `corridor_clearance()` para evaluar una trayectoria local mediante muestras en el centro y a ambos lados del pasillo. Si la odometría es reciente, el vehículo gira poco y hay suficiente espacio frontal, `straight_confidence` permite **aumentar gradualmente** el límite frontal. Si estas condiciones dejan de cumplirse, el aumento se desactiva y se mantiene el cálculo conservador. Esta mejora no sustituye los límites por curvas o giro.

~~~python
target = max(0.0, min(
    self.max_speed, speed_original, speed_curve, speed_front
))
drive.drive.steering_angle = float(steering)
drive.drive.speed = float(command)
self.drive_pub.publish(drive)
~~~

`target` es la velocidad deseada; `command` es la velocidad enviada **después de limitar cuánto puede cambiar entre actualizaciones** mediante `accel_limit` y `decel_limit`. De este modo, el carro no recibe un salto instantáneo de velocidad cada vez que cambia el LiDAR.

**Configuración usada en este proyecto:**

~~~bash
python3 controllers/gap_follow_sepang_v31_en.py --ros-args -p max_speed:=12.0 -p front_boost_max:=0.8
~~~

Estos números son parámetros de control, no garantías de velocidad real alcanzada.

## 📂 Estructura del repositorio

~~~text
Proyecto-1-Parcial-Wilson-Gua-a/
├── README.md                          # Guía en inglés
├── README_ES.md                       # Guía en español
├── controllers/
│   └── gap_follow_sepang_v31_en.py    # Controlador y telemetría en inglés
├── timing/
│   └── lap_timer_sepang.py            # Cronómetro independiente de 10 vueltas
├── maps/
│   ├── Sepang_map.png
│   └── Sepang_map.yaml
├── scripts/
│   └── configure_sepang.py            # Configuración de mapa y punto de salida
├── results/
│   └── summary.csv                    # Registro experimental anterior
└── .gitignore
~~~

El **simulador es una dependencia externa**; no está incluido en este repositorio.

## ⚙️ Instalación en Ubuntu 22.04

**1. Instala ROS 2 Humble Desktop** siguiendo la [guía oficial para Ubuntu](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html) y verifica:

~~~bash
source /opt/ros/humble/setup.bash
ros2 --help
~~~

**2. Instala herramientas y dependencias ROS 2:**

~~~bash
sudo apt update
sudo apt install -y git python3-pip python3-numpy python3-colcon-common-extensions python3-rosdep2
sudo apt install -y ros-humble-ackermann-msgs ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager ros-humble-xacro ros-humble-robot-state-publisher
~~~

Inicializa `rosdep` una sola vez si hace falta (`sudo rosdep init`), y ejecuta `rosdep update`.

**3. Instala F1TENTH Gym y descarga el simulador y nuestro proyecto:**

~~~bash
cd ~
git clone https://github.com/f1tenth/f1tenth_gym.git
cd ~/f1tenth_gym
python3 -m pip install -e .

cd ~
git clone https://github.com/widegonz/F1Tenth-Repository.git
git clone https://github.com/wilsonguana17/Proyecto-1-Parcial-Wilson-Gua-a.git
~~~

Consulta los requisitos adicionales de [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym) y del [simulador ROS](https://github.com/widegonz/F1Tenth-Repository).

**4. Configura Sepang y compila el simulador:**

~~~bash
python3 ~/Proyecto-1-Parcial-Wilson-Gua-a/scripts/configure_sepang.py \
  --workspace ~/F1Tenth-Repository

cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
rosdep install -i --from-paths src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
~~~

El script copia ambos archivos del mapa y respalda `sim.yaml` antes de actualizar `map_path`, `map_img_ext`, `num_agent` y la posición inicial de Sepang (`sx=-14.14`, `sy=-1.14`, `stheta=0.08`).

Si ya tienes el simulador funcionando en `~/FollowTheGap-F1Tenth`, puedes indicar esa ruta en `--workspace` y cargar luego el `install/setup.bash` de ese mismo entorno.

> **Compatibilidad:** se probó el controlador en un entorno Ubuntu/ROS 2 Humble ya configurado. Una instalación nueva podría necesitar ajustes de versiones en dependencias externas del simulador.

## 🚀 Ejecución de Sepang (3 terminales)

**Terminal 1 — simulador:**

~~~bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
~~~

**Terminal 2 — inicia el cronómetro con el carro detenido:**

~~~bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 timing/lap_timer_sepang.py
~~~

**Terminal 3 — ejecuta el controlador al final:**

~~~bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 controllers/gap_follow_sepang_v31_en.py --ros-args \
  -p max_speed:=12.0 -p front_boost_max:=0.8
~~~

Detén el controlador con `Ctrl+C`. **No ejecutes dos controladores a la vez.** El cronómetro **no detiene automáticamente** el vehículo.

## ⏱️ Cronómetro, telemetría y problemas frecuentes

### ¿Cómo funciona el contador de vueltas?

Archivo: [`timing/lap_timer_sepang.py`](timing/lap_timer_sepang.py). Es **otro nodo ROS 2**: se suscribe a `/ego_racecar/odom`, pero no publica en `/drive` ni modifica el movimiento del carro.

- **Referencia inicial:** `odom_callback()` guarda las primeras coordenadas `(x, y)` y la orientación inicial; `yaw_from_quaternion()` transforma la orientación de ROS a un ángulo de guiñada.
- **Inicio del cronómetro:** espera hasta que el vehículo se aleje **0.4 m** de la posición inicial y entonces registra `lap_start_time` mediante `time.monotonic()`.
- **Detección de una vuelta:** el vehículo debe alejarse **5 m** para rearmar la meta y regresar a menos de **2 m** del inicio. Además, se requieren **20 s**, **80 m recorridos estimados** y una orientación compatible con la inicial.
- **Registro:** incrementa `count`, calcula el tiempo de la vuelta y la mejor vuelta, y escribe una fila en un CSV. Al llegar a **10 vueltas**, finaliza el conteo, aunque **no detiene el controlador**.

El fragmento principal que evita contar vueltas falsas es:

~~~python
if distance_to_start >= self.REARM_RADIUS_M:
    self.armed = True

if not self.armed or distance_to_start > self.START_RADIUS_M:
    return

elapsed = now - self.lap_start_time
heading_dot = math.cos(yaw - self.start_yaw)
if (
    elapsed < self.MIN_LAP_TIME_S
    or self.lap_distance < self.MIN_LAP_DISTANCE_M
    or heading_dot < self.MIN_HEADING_DOT
):
    return

self.count += 1
~~~

La distancia recorrida se **estima acumulando desplazamientos entre mediciones de odometría**; saltos de posición mayores a 20 m se descartan. Los resultados se guardan como `sepang_lap_results_YYYYMMDD_HHMMSS.csv` en el directorio de ejecución. El contador usa tiempo real monotónico (no necesariamente tiempo simulado), y **no detecta colisiones**.

El controlador registra etiquetas en inglés como `ActualSpeed`, `CommandSpeed`, `TargetSpeed`, `CorridorDistance`, `Confidence`, `Mode` y `LimitedBy`. El campo `Mode` permite identificar si la mejora de velocidad en rectas está desactivada, parcialmente activa o completamente habilitada.

- **No se mueve el carro:** revisa los tópicos con `ros2 topic list` y comprueba que solo un controlador publique en `/drive`.
- **Aparece otro mapa:** vuelve a ejecutar `configure_sepang.py` en el entorno correcto y reinicia el simulador.
- **Faltan módulos ROS:** carga ROS 2 y el workspace compilado.
- **Conteo incorrecto:** inicia el cronómetro antes de mover el carro y evita reinicios o teletransportes en medio de la prueba.

**Referencias:** [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym) · [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros) · [Entorno ROS 2 Humble](https://github.com/widegonz/F1Tenth-Repository). El mapa Sepang proviene de los archivos de mapas suministrados para el proyecto; conviene verificar sus condiciones de redistribución. El controlador se diseñó para **simulación**, no está validado para un vehículo físico.
