# 🏎️ F1TENTH Sepang | Follow the Gap reactivo — Partes 1 y 2

[English](README.md) · **Español**  
**ROS 2 Humble · Ubuntu 22.04 · LiDAR · Dirección Ackermann**

Este repositorio reúne la **parte 1 (un vehículo con Follow the Gap)** y la **parte 2 (cinco obstáculos y adelantamiento entre dos vehículos)** en Sepang, **sin una trayectoria global predefinida**. Incluye un **cronómetro de 10 vueltas**, el mapa de Sepang y un script de configuración. El controlador decide hacia dónde conducir usando el LiDAR; la odometría permite supervisar el movimiento del vehículo.

**Contenido:** [Parte 1 — controlador original](#-cómo-funciona-follow-the-gap) · [Parte 2 — competencia con V3.9](#-parte-2--cinco-obstáculos-estáticos-y-un-oponente-dinámico)

## 🎥 Demostración de 10 vueltas

[![Ver la demostración de 10 vueltas en Sepang](https://img.youtube.com/vi/9m-QhL65yLw/hqdefault.jpg)](https://youtu.be/9m-QhL65yLw)

**[▶ Ver en YouTube](https://youtu.be/9m-QhL65yLw)** — Haz clic en la miniatura para abrir el video.

### 🎬 Video original sin edición — 10 vueltas (Parte 1)

[![Ver el video original sin edición de las 10 vueltas](https://img.youtube.com/vi/5wyF8lTBg24/hqdefault.jpg)](https://youtu.be/5wyF8lTBg24)

**[▶ Ver grabación original sin edición en YouTube](https://youtu.be/5wyF8lTBg24)** — Haz clic en la miniatura para abrir la grabación original completa.

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

---

# 🏁 Parte 2 — Cinco obstáculos estáticos y un oponente dinámico

**Controlador definitivo de competencia: [Follow the Gap V3.9](controllers/gap_follow_sepang_v39.py).**
Los dos vehículos ejecutan **el mismo algoritmo Python**: únicamente cambian los tópicos ROS asociados y el valor configurado de velocidad máxima. En esta sección se documenta la versión final, no las pruebas intermedias.

## 🎥 Parte 2 — Video de la competencia

[![Ver el video de la parte 2 en F1TENTH Sepang](https://img.youtube.com/vi/_IGRx_2CJ-s/hqdefault.jpg)](https://youtu.be/_IGRx_2CJ-s)

**[▶ Ver el video de la parte 2 en YouTube](https://youtu.be/_IGRx_2CJ-s)** — Haz clic en la miniatura para abrir el video.

### 🎬 Video original sin edición — Parte 2 (Google Drive)

[![Ver el video original sin edición de la parte 2 en Google Drive](https://drive.google.com/thumbnail?id=1qDRfRShxvAR081CvCkMgt6b2JzihsOHV&sz=w640)](https://drive.google.com/file/d/1qDRfRShxvAR081CvCkMgt6b2JzihsOHV/view?usp=sharing)

**[▶ Ver grabación completa y original sin edición de la parte 2 en Google Drive](https://drive.google.com/file/d/1qDRfRShxvAR081CvCkMgt6b2JzihsOHV/view?usp=sharing)** — Haz clic en la miniatura o en el enlace para abrir el video.

*Si el video no se puede abrir, el propietario debe configurar el archivo en Google Drive como **Cualquier persona con el enlace → Lector**.*

## 🎯 Pruebas y resultados observados

| Prueba | Configuración | Resultado reportado |
|---|---|---|
| Evasión de obstáculos | Sepang con cinco obstáculos dibujados en el mapa de ocupación | 10 vueltas consecutivas sin colisión |
| Oponente dinámico | Mismo mapa con cinco obstáculos y dos carros autónomos; oponente inicialmente ~5 m adelante | 10 vueltas para cada vehículo y **un adelantamiento exitoso**, sin colisión |

## 📊 Parte 2 — Resultados de la carrera final (22 de noviembre de 2026)

Ambos vehículos ejecutaron **Follow the Gap V3.9** en Sepang con cinco obstáculos estáticos. El vehículo principal comenzó aproximadamente 5 m detrás del oponente y realizó un adelantamiento observado durante la carrera.

| Parámetro | Vehículo principal (EGO) | Vehículo oponente (OPP) |
|---|---:|---:|
| Controlador reactivo | Follow the Gap V3.9 | Follow the Gap V3.9 |
| Velocidad máxima configurada | 12 m/s | 6 m/s |
| **Vueltas completadas** | **10 / 10 ✅** | **10 / 10 ✅** |
| **Mejor tiempo por vuelta** | **81.24 s 🥇** | **93.43 s 🥈** |
| Número de la vuelta más rápida | 6 | 6 |
| **Tiempo total (10 vueltas)** | **847.96 s** | **944.78 s** |

El vehículo principal completó sus diez vueltas **96.82 s antes** que el oponente, según la diferencia de tiempos totales. Los resultados se verificaron directamente con los **archivos CSV originales de las 10 vueltas**, que ya están disponibles en este repositorio:

- **Principal (EGO):** [`results/sepang_lap_results_ego_20261122_160532.csv`](results/sepang_lap_results_ego_20261122_160532.csv) — mejor vuelta **81.239 s** (vuelta 6), total **847.960 s**.
- **Oponente (OPP):** [`results/sepang_lap_results_opp_20261122_160552.csv`](results/sepang_lap_results_opp_20261122_160552.csv) — mejor vuelta **93.431 s** (vuelta 6), total **944.780 s**.

La tabla anterior redondea los valores originales a dos decimales para facilitar la lectura.

## 🧠 Enfoque reactivo: Follow the Gap V3.9

El controlador toma decisiones locales a partir de sus sensores; no sigue una trayectoria global predefinida:

1. **Preprocesamiento LiDAR:** `preprocess_lidar()` utiliza los 180° frontales del tópico `/scan`, maneja las lecturas inválidas, limita el alcance y emplea un mínimo local de tres rayos para conservar los bordes de los obstáculos.
2. **Detección de gaps:** `choose_gap()` identifica regiones consecutivas libres dentro de ±65°. La anticipación depende de la velocidad: `lookahead_base + lookahead_gain × actual_speed`, con máximo de 12 m. Una burbuja local desaconseja dirigirse hacia obstáculos frontales sobresalientes.
3. **Dirección y verificación de pasillos:** `refine_steering()` compara otras direcciones si la trayectoria inicial es demasiado corta. `path_clearance()` estima el espacio libre sobre un arco de Ackermann y revisa centro y bordes laterales del vehículo.
4. **Velocidad adaptativa:** el comando se limita por velocidad máxima, dirección, aceleración lateral y distancia libre de la trayectoria. También se limita la rapidez de aceleración y frenado. **Configurar 12 m/s no implica alcanzar continuamente esa velocidad real.**
5. **Adelantamiento reactivo:** `opp_callback()` recibe la odometría del oponente. `_relative()` estima su posición relativa; `_overtake()` compara corredores candidatos a ambos lados y activa `OVERTAKE` únicamente si encuentra una posibilidad de paso. `_pass_path()` evalúa trayectorias laterales y estima la posición futura del rival. `_filtered()` separa aproximadamente sus retornos LiDAR de los obstáculos estáticos, manteniendo una verificación dinámica independiente. Una vez que el rival queda atrás, regresa al modo `NORMAL`.
6. **Publicación:** `scan_callback()` envía `AckermannDriveStamped` al tópico de conducción correspondiente.

**Sensores utilizados:** la evasión de paredes y obstáculos se basa en LiDAR y odometría propia. Para reconocer al rival y planificar el adelantamiento, **también se utiliza su odometría publicada por el simulador**. Por tanto, no es un rastreador de oponentes exclusivamente basado en LiDAR. Las verificaciones de corredor son aproximaciones geométricas, no garantías formales de ausencia de choque.

### Organización del código y tópicos ROS

| Archivo | Función |
|---|---|
| [`controllers/gap_follow_sepang_v39.py`](controllers/gap_follow_sepang_v39.py) | Controlador final de la parte 2: navegación reactiva, evasión, velocidad adaptativa y adelantamiento |
| [`timing/lap_timer_sepang_dual.py`](timing/lap_timer_sepang_dual.py) | Contador de 10 vueltas para cada carro: `vehicle:=ego` o `vehicle:=opp` |
| [`scripts/configure_sepang_part2.py`](scripts/configure_sepang_part2.py) | Configura el mapa, `num_agent: 2` y las posiciones iniciales |
| [`maps/Sepang_5obs_map.yaml`](maps/Sepang_5obs_map.yaml) | Metadatos del mapa; se refiere a `Sepang_5obs_map.png` |
| [`maps/Sepang_5obs_map.png`](maps/Sepang_5obs_map.png) | Mapa definitivo de Sepang con los cinco obstáculos estáticos, incluido en el repositorio |
| `results/` | Evidencias opcionales (archivos CSV, capturas y enlaces a videos) |
| [`controllers/gap_follow_sepang_v31_en.py`](controllers/gap_follow_sepang_v31_en.py) | Controlador original de la parte 1, conservado |
| [`timing/lap_timer_sepang.py`](timing/lap_timer_sepang.py) | Contador original de la parte 1, conservado |

Cada carro dispone de tópicos independientes:

| Señal | Robot principal | Oponente |
|---|---|---|
| LiDAR | `/scan` | `/opp_scan` |
| Odometría propia | `/ego_racecar/odom` | `/opp_racecar/odom` |
| Odometría del otro robot | `/ego_racecar/opp_odom` | `/opp_racecar/opp_odom` |
| Comandos de movimiento | `/drive` | `/opp_drive` |

## ⚙️ Preparación de la parte 2

Primero sigue los requisitos de instalación de ROS 2 Humble y F1Tenth indicados en la **parte 1** de este README. El simulador sigue siendo una **dependencia externa**, aquí ubicada en `~/F1Tenth-Repository`.

**Mapa definitivo incluido:** el archivo [`Sepang_5obs_map.png`](maps/Sepang_5obs_map.png), editado en GIMP con los cinco obstáculos, y su [`Sepang_5obs_map.yaml`](maps/Sepang_5obs_map.yaml) ya están disponibles en este repositorio. El YAML conserva la resolución original (0.05944 m/píxel) y el mismo origen.

Para configurar el simulador desde los archivos del repositorio, ejecuta:

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 scripts/configure_sepang_part2.py --workspace ~/F1Tenth-Repository

cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
~~~

El nuevo script respalda `sim.yaml` y establece:

~~~yaml
map_path: '/home/wilson/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_5obs_map'
map_img_ext: '.png'
num_agent: 2
sx: -14.14
sy: -1.14
stheta: 0.08
sx1: -9.16
sy1: -0.74
stheta1: 0.08
~~~

La ruta anterior es un **ejemplo** del equipo del autor; el script utiliza la ruta `--workspace` recibida para otros equipos. Los centros iniciales quedan separados aproximadamente 5 m según su rumbo; **verifica en RViz que ambos estén dentro de la pista y lejos de los obstáculos**. No ejecutes `configure_sepang.py` de la parte 1 en esta prueba: volvería a establecer un solo vehículo.

## 🚀 Ejecución completa (cinco terminales)

**Terminal 1 — Simulador:**

~~~bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
~~~

Ejecuta **ambos contadores antes de los controladores**, con los carros detenidos.

**Terminal 2 — Cronómetro del principal:**

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
python3 timing/lap_timer_sepang_dual.py --ros-args -r __node:=lap_timer_ego -p vehicle:=ego
~~~

**Terminal 3 — Cronómetro del oponente:**

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
python3 timing/lap_timer_sepang_dual.py --ros-args -r __node:=lap_timer_opponent -p vehicle:=opp
~~~

**Terminal 4 — Oponente, V3.9 a máximo 6 m/s:**

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
python3 controllers/gap_follow_sepang_v39.py --ros-args \
  -r __node:=controller_opponent \
  -r /scan:=/opp_scan \
  -r /ego_racecar/odom:=/opp_racecar/odom \
  -r /ego_racecar/opp_odom:=/opp_racecar/opp_odom \
  -r /drive:=/opp_drive \
  -p max_speed:=6.0 -p accel_limit:=5.0
~~~

**Terminal 5 — Principal, el mismo V3.9 a máximo 12 m/s:**

~~~bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
python3 controllers/gap_follow_sepang_v39.py --ros-args \
  -r __node:=controller_ego \
  -p max_speed:=12.0 -p accel_limit:=5.0
~~~

Con `num_agent: 2`, esta versión del simulador comienza a avanzar físicamente **cuando ambos tópicos de conducción reciben comandos**. Por ello, el segundo carro puede permanecer quieto hasta que se ejecute el controlador principal. No ejecutes dos controladores sobre el mismo tópico `/drive`.

### Prueba independiente de cinco obstáculos

Para evaluar únicamente la primera prueba de la parte 2, conserva `Sepang_5obs_map`, cambia `num_agent: 1` en `sim.yaml`, recompila si corresponde y reinicia. Ejecuta un único V3.9 con los tópicos del vehículo principal y un cronómetro de diez vueltas. Restablece `num_agent: 2` para la competencia con oponente.

## ⏱️ Contadores, evidencias y problemas frecuentes


### ¿Cómo está estructurado `lap_timer_sepang_dual.py`?

El [contador dual](timing/lap_timer_sepang_dual.py) es un **nodo ROS 2 independiente**. Se ejecuta dos veces con el mismo archivo: `vehicle:=ego` lee `/ego_racecar/odom` y `vehicle:=opp` lee `/opp_racecar/odom`. No envía órdenes de velocidad ni dirección.

| Función principal | Responsabilidad |
|---|---|
| `__init__()` | Selecciona el vehículo y su tópico de odometría, inicializa los contadores y crea un CSV independiente. |
| `yaw_from_quaternion()` | Convierte la orientación del vehículo en un ángulo de rumbo (yaw). |
| `odom_callback()` | Lee la posición, inicia el cronómetro al detectar movimiento, valida cada vuelta y registra tiempo por vuelta, mejor tiempo y total hasta completar 10. |
| `destroy_node()` | Cierra correctamente el archivo CSV. |
| `main()` | Inicializa el nodo ROS 2 y mantiene activa la recepción de odometría. |

Para evitar vueltas falsas, comprueba que el carro se aleje de la salida, recorra una distancia y tiempo mínimos y regrese con orientación compatible. **Este contador no detecta colisiones.**


- `lap_timer_sepang_dual.py` aplica los **mismos criterios de 10 vueltas** del cronómetro original: alejarse 5 m para rearmar, regresar a menos de 2 m, completar al menos 20 s y 80 m estimados, y conservar una orientación compatible. Comienza a medir cuando detecta un desplazamiento inicial de 0.4 m.
- Cada terminal genera un CSV en el directorio de ejecución: `sepang_lap_results_ego_YYYYMMDD_HHMMSS.csv` y `sepang_lap_results_opp_YYYYMMDD_HHMMSS.csv`. Contienen vuelta, duración, mejor tiempo, tiempo total y distancia estimada.
- La telemetría muestra `ActualSpeed`, `CommandSpeed`, `TargetSpeed`, `PathClear`, `Lookahead`, `Steering`, coordenadas y avisos `PASS START`, `PASS WAIT`, `PASS ABORT` y `PASS COMPLETE`.
- **El mensaje `PASS COMPLETE` no demuestra por sí solo un adelantamiento seguro.** Es importante grabar RViz para confirmar que ambos cambian de posición sin contacto. Los contadores no identifican colisiones ni detienen los carros.
- **Si los vehículos no avanzan**, verifica `num_agent: 2`, los dos nodos de control y los tópicos `/drive` y `/opp_drive`. Utiliza `ros2 topic list`.
- **Si aparece el mapa incorrecto**, comprueba `map_path`, la correspondencia de los nombres PNG/YAML, la copia del mapa y el reinicio del simulador.
- **Si se cuentan mal las vueltas**, reinicia simulador y contadores con los dos vehículos quietos; evita teletransportarlos durante el cronometraje.

**Alcance:** proyecto validado en simulación F1Tenth; no constituye un controlador probado en un vehículo físico. La demostración original de la parte 1 se mantiene arriba.
