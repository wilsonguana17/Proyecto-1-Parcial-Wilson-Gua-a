# F1TENTH Sepang — Follow the Gap reactivo (V3.1)

[English](README.md) | **Español**

> ROS 2 Humble · Ubuntu 22.04 · Navegación reactiva con LiDAR · Dirección Ackermann

Este repositorio contiene un controlador reactivo **Follow the Gap** para el circuito **Sepang** en el simulador F1TENTH, un **contador independiente de 10 vueltas**, los archivos del mapa y un script de configuración. Este documento es un **tutorial para principiantes** que explica cómo instalar los componentes, configurar el mapa y ejecutar una prueba desde una computadora Ubuntu.

**Resultado observado en la simulación del autor:** 10 vueltas completadas, **777.40 s en total**, **77.47 s en la mejor vuelta**, sin colisiones **observadas por el operador**. Es el resultado de una prueba, no una garantía de rendimiento en otras computadoras o ejecuciones. El contador no detecta impactos.

## 1. ¿Qué es el control reactivo Follow the Gap?

El controlador **no consulta un mapa ni sigue una trayectoria global predefinida** para decidir cómo conducir. En cada actualización del LiDAR:

1. Selecciona las mediciones frontales (aproximadamente ±90°) y suaviza las distancias.
2. Coloca una **burbuja de seguridad** sobre el obstáculo cercano seleccionado.
3. Localiza regiones consecutivas de espacio libre (**gaps**) y escoge la más amplia.
4. Combina el punto más lejano del gap con su centro para seleccionar una dirección.
5. Suaviza el giro y calcula límites de velocidad según dirección, aceleración lateral y distancia frontal.
6. Si el vehículo parece avanzar estable y el LiDAR confirma un pasillo local, permite una **mejora frontal limitada**.
7. Publica velocidad y dirección en `/drive`.

**Flujo ROS 2:** `/scan` (`sensor_msgs/LaserScan`) + `/ego_racecar/odom` (`nav_msgs/Odometry`) → controlador → `/drive` (`ackermann_msgs/AckermannDriveStamped`). El cronómetro solo recibe odometría, no controla el vehículo.

## 2. Estructura del proyecto

```text
Proyecto-1-Parcial-Wilson-Gua-a/
├── README.md                           # Tutorial completo en inglés
├── README_ES.md                        # Tutorial completo en español (este)
├── controllers/
│   └── gap_follow_sepang_v31_en.py     # Controlador y telemetría en inglés
├── timing/
│   └── lap_timer_sepang.py             # Contador y CSV en inglés
├── maps/
│   ├── Sepang_map.png                  # Imagen del mapa de ocupación
│   └── Sepang_map.yaml                 # Metadatos del mapa
├── scripts/
│   └── configure_sepang.py             # Copia mapa y configura sim.yaml
├── results/
│   └── summary.csv                     # Resumen de pruebas realizadas
└── .gitignore
```

**El simulador es una dependencia externa**; no se copian los archivos del simulador a este proyecto.

## 3. Requisitos

- **Ubuntu 22.04 LTS**, con interfaz gráfica disponible para RViz.
- **ROS 2 Humble Desktop**, instalado desde la [documentación oficial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html).
- Python 3, `git`, `pip`, `numpy`, `colcon`, `rosdep` y F1TENTH Gym.
- Paquetes ROS 2 para `ackermann_msgs`, servidor de mapas Nav2, gestor de ciclo de vida, Xacro y publicación del estado del robot.

> **Nota de compatibilidad:** los programas se ejecutaron en un entorno Ubuntu + ROS Humble ya configurado. Los pasos para una instalación completamente nueva se basan en la documentación del simulador, pero **todavía no se han verificado en una segunda computadora limpia**. Algunas versiones de dependencias podrían requerir ajustes.

> **Acceso al repositorio:** mientras sea **privado**, únicamente el propietario y los colaboradores autorizados podrán clonarlo. Para compartirlo con el profesor, cambia la visibilidad a **Public** o concédele acceso.

## 4. Instalación desde cero en otra computadora Ubuntu

### 4.1 Instalar ROS 2 Humble

Sigue la [guía de instalación de ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), selecciona la instalación **Desktop** y comprueba:

```bash
source /opt/ros/humble/setup.bash
ros2 --help
```

### 4.2 Instalar herramientas y paquetes ROS 2

```bash
sudo apt update
sudo apt install -y git unzip python3-pip python3-numpy python3-colcon-common-extensions python3-rosdep2
sudo apt install -y ros-humble-ackermann-msgs ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager ros-humble-xacro ros-humble-robot-state-publisher
```

Si nunca inicializaste `rosdep`, ejecuta **una sola vez** `sudo rosdep init`. Luego:

```bash
rosdep update
```

### 4.3 Instalar el motor F1TENTH Gym

```bash
cd ~
git clone https://github.com/f1tenth/f1tenth_gym.git
cd ~/f1tenth_gym
python3 -m pip install -e .
```

Se sigue el enfoque de instalación descrito en [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym). Si aparecen conflictos entre versiones de Python, Gym y ROS, hay que resolverlos sin desinstalar indiscriminadamente los paquetes de ROS.

### 4.4 Descargar el simulador y nuestro repositorio

El repositorio público [F1Tenth-Repository](https://github.com/widegonz/F1Tenth-Repository) contiene un entorno ROS 2 con instrucciones para Humble:

```bash
cd ~
git clone https://github.com/widegonz/F1Tenth-Repository.git
```

Descarga **este proyecto** reemplazando `wilsonguana17` por el usuario real de GitHub que lo publique:

```bash
cd ~
git clone https://github.com/wilsonguana17/Proyecto-1-Parcial-Wilson-Gua-a.git
```

> No copies literalmente `wilsonguana17`: es un ejemplo que cambiaremos cuando el repositorio esté creado.

### 4.5 Configurar Sepang automáticamente

```bash
python3 ~/Proyecto-1-Parcial-Wilson-Gua-a/scripts/configure_sepang.py \
  --workspace ~/F1Tenth-Repository
```

Este script verifica que exista el simulador, copia `Sepang_map.png` y `Sepang_map.yaml` a la carpeta de mapas, crea un **respaldo** de `src/f1tenth_gym_ros/config/sim.yaml` y modifica solamente:

```yaml
map_path: '<HOME>/F1Tenth-Repository/src/f1tenth_gym_ros/maps/Sepang_map'
map_img_ext: '.png'
num_agent: 1
sx: -14.14
sy: -1.14
stheta: 0.08
```

`<HOME>` se reemplaza automáticamente por la ruta personal del usuario. Este fragmento contiene **solo las claves modificadas**, no todo el archivo `sim.yaml`.

### 4.6 Instalar dependencias y compilar el simulador

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
rosdep install -i --from-paths src --rosdistro humble -y
colcon build --symlink-install
source install/setup.bash
```

Si la compilación falla, revisa el error concreto. Algunas revisiones antiguas de F1TENTH Gym necesitan versiones compatibles de bibliotecas Python.

## 5. Ejecución de Sepang (tres terminales)

**Terminal 1 — simulador RViz:**

```bash
cd ~/F1Tenth-Repository
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

Comprueba que se abra **Sepang** y que el vehículo esté detenido dentro de la pista.

**Terminal 2 — iniciar el contador ANTES de mover el vehículo:**

```bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 timing/lap_timer_sepang.py
```

**Terminal 3 — ejecutar la configuración exitosa de la V3.1:**

```bash
source /opt/ros/humble/setup.bash
source ~/F1Tenth-Repository/install/setup.bash
cd ~/Proyecto-1-Parcial-Wilson-Gua-a
python3 controllers/gap_follow_sepang_v31_en.py --ros-args \
  -p max_speed:=12.0 -p front_boost_max:=0.8
```

Para detener el controlador presiona **Ctrl+C**. El contador registra hasta 10 vueltas y genera `sepang_lap_results_YYYYMMDD_HHMMSS.csv` en el **directorio desde donde se ejecutó**. El contador **no detiene automáticamente** el vehículo. No ejecutes dos controladores al mismo tiempo.

**Si ya tienes el entorno probado** en `~/FollowTheGap-F1Tenth`, puedes configurarlo usando `--workspace ~/FollowTheGap-F1Tenth`, compilar y cargar ese entorno, y ejecutar los dos scripts de este repositorio. No es necesario descargar otro simulador.

## 6. Explicación de las partes esenciales del código

El controlador está en `controllers/gap_follow_sepang_v31_en.py`:

| Función | Para qué sirve |
|---|---|
| `__init__` | Crea suscripciones y publicador ROS, declara parámetros e inicializa estados. |
| `odom_callback` | Lee velocidad, velocidad angular y X/Y, y comprueba la actualización de la odometría. |
| `preprocess_lidar` | Selecciona LiDAR frontal, trata valores inválidos, limita y suaviza lecturas. |
| `choose_gap` | Genera la burbuja de seguridad, encuentra el mayor gap y escoge la dirección. |
| `front_distance_v2` | Calcula el percentil 20 de distancias válidas del sector frontal ±12°. |
| `corridor_clearance` | Estima distancia libre de un pasillo local de anchura finita. |
| `confidence_component` | Convierte dirección y velocidad angular en confianza para acelerar en rectas. |
| `scan_callback` | Aplica límites de dirección/velocidad, publica `/drive` y registra telemetría en inglés. |

**Dirección:** si el rayo seleccionado tiene índice `i`, entonces `δ_propuesta = angle_min + i × angle_increment`. Se limita a ±0.4189 rad y se suaviza: `δ = 0.65 δ_propuesta + 0.35 δ_anterior`.

**Límites de velocidad** (en m/s):

```text
v_FTG      = max(1, vmax × (1 - (abs(direccion) / direccion_max)^1.8))
v_curva    = sqrt(a_lat_max × distancia_ejes / max(abs(tan(direccion)), 1e-5))
v_frenteV2 = sqrt(2 × decel_frenado × max(0, distancia_frontal - margen))
v_objetivo = min(vmax, v_FTG, v_curva, v_frenteV3_1)
```

Los valores incluyen `distancia_ejes=0.33 m`, `a_lat_max=8 m/s²`, `brake_accel=5 m/s²`, `front_margin=1 m`, `accel_limit=3 m/s²` y `decel_limit=12 m/s²`. Son **valores de configuración**, no garantías físicas verificadas.

**Mejora V3.1:** si la dirección y el `YawRate` sugieren una recta estable y la odometría está actualizada, se calcula un pasillo de distancia local. Una ganancia no negativa de hasta `front_boost_max × confidence` puede aumentar el límite frontal de la V2. Por sí sola, esta mejora nunca reduce el límite anterior. El objetivo final sigue cumpliendo las demás restricciones.

El contador (`timing/lap_timer_sepang.py`) registra la posición inicial en su primera odometría, inicia al detectar un desplazamiento de 0.4 m y cuenta vueltas cuando se vuelve cerca de la salida **después** de alejarse lo suficiente, cumplir distancia/tiempo mínimos y tener orientación compatible. Estima la distancia sumando incrementos de posición X/Y. Es un contador por proximidad, no una línea de meta geométrica exacta.

## 7. Telemetría y resultados CSV

| Etiqueta en inglés | Significado en español |
|---|---|
| `ActualSpeed` | Magnitud de velocidad real calculada de la odometría (m/s). |
| `CommandSpeed` | Velocidad enviada por `/drive` (m/s). |
| `TargetSpeed` | Velocidad objetivo tras las restricciones (m/s). |
| `CornerLimit` | Velocidad máxima calculada para una curva (m/s). |
| `FrontDistanceV2` | Estimación de distancia frontal de la V2 (m). |
| `CorridorDistance` | Distancia estimada del pasillo local (m); `--` si no se calcula. |
| `FrontSpeedLimit` | Límite frontal calculado por la V3.1 (m/s). |
| `V2Target` | Objetivo que habría elegido la lógica de la V2 (m/s). |
| `SpeedGain` | Aumento del objetivo respecto a la V2 (m/s). |
| `Confidence` | Confianza de recta entre 0 y 1. |
| `Mode` | `STRAIGHT`, `BLENDED` o `V2`. |
| `LimitedBy` | Restricción activa (`MAX`, `FTG`, `CORNER`, `FRONT`). |
| `SteeringAngle` | Ángulo de dirección ordenado (grados). |
| `YawRate` | Velocidad angular medida (grados/s). |
| `X`, `Y` | Posición X/Y en el mapa (m). |

Los tiempos de mensajes ROS son generados por ROS 2. El CSV contiene `lap`, `lap_time_s`, `best_lap_s`, `elapsed_time_s`, `lap_distance_m`.

## 8. Resultados medidos y limitaciones

| Controlador | Máximo configurado | Vueltas | Tiempo total | Mejor vuelta |
|---|---:|---:|---:|---:|
| V1 | 6.0 m/s | 10/10 | 851.57 s | 84.89 s |
| V2 | 8.5 m/s | 10/10 | 781.06 s | 77.64 s |
| V2 | 12.0 m/s | 10/10 | 780.42 s | 77.82 s |
| **V3.1** | **12.0 m/s** | **10/10** | **777.40 s** | **77.47 s** |

`results/summary.csv` conserva los valores reportados. **Sin colisiones** significa ninguna observada durante esas pruebas, **no** detección automática de choques ni demostración estadística de confiabilidad. El tiempo se mide con un reloj monotónico de ejecución, no necesariamente equivalente al tiempo de simulación. Conviene repetir ensayos independientes antes de realizar comparaciones concluyentes. La velocidad máxima es un tope del comando y puede no alcanzarse por otras restricciones.

## 9. Solución de problemas

- **`python3: can't open file`** — ejecuta desde la raíz de este repositorio o utiliza la ruta completa dentro de `controllers/` o `timing/`.
- **Aparece el mapa anterior** — cierra el simulador, vuelve a configurar Sepang, recompila y carga el workspace; comprueba `ros2 param get /bridge map_path`.
- **No se mueve el vehículo** — revisa `/scan`, `/ego_racecar/odom`, `/drive` mediante `ros2 topic list` y asegúrate de que no haya otro controlador publicando.
- **Falta `rclpy` o `ackermann_msgs`** — carga ROS 2, instala los paquetes y carga el workspace compilado.
- **Error de `f110_gym` o `f110-v0`** — verifica que F1TENTH Gym esté instalado en el entorno Python utilizado por ROS.
- **Cuenta mal las vueltas** — inicia el cronómetro con el auto detenido en el punto de salida y no teletransportes ni reinicies el simulador a media carrera.

## 10. Fuentes, atribución y alcance

- Simulación y puente ROS: [F1TENTH Gym](https://github.com/f1tenth/f1tenth_gym), [F1TENTH Gym ROS](https://github.com/f1tenth/f1tenth_gym_ros) y la [guía de entorno ROS 2 Humble](https://github.com/widegonz/F1Tenth-Repository). El código del simulador **no está redistribuido** aquí.
- Los archivos `Sepang_map` provienen del ZIP `Mapas-F1Tenth-v2.zip` aportado por el autor. **Se debe verificar su licencia/procedencia antes de publicar el repositorio**.
- Este controlador es para **simulación**, no se ha validado como sistema de seguridad para un vehículo físico.
- Antes de publicar, el propietario debe decidir la licencia de sus archivos originales y comprobar las condiciones de materiales de terceros.
