# websim F1TENTH

Simulador para los labs de F1TENTH con interfaz en el navegador. Habla **exactamente los mismos topics que `f1tenth_gym_ros`**, así que tus nodos (safety node, wall follow, gap follow…) funcionan sin cambiar una línea:

| Topic | Tipo | Sentido |
|---|---|---|
| `/scan` | `sensor_msgs/LaserScan` (1080 haces, 270°, 40 Hz, frame `ego_racecar/laser`) | sale |
| `/ego_racecar/odom` | `nav_msgs/Odometry` (100 Hz, `map` → `ego_racecar/base_link`) | sale |
| `/map` | `nav_msgs/OccupancyGrid` (transient local, incluye lo que dibujes) | sale |
| TF | `map → ego_racecar/base_link → ego_racecar/laser` | sale |
| `/websim/collision` | `std_msgs/Bool` | sale |
| `/drive` | `ackermann_msgs/AckermannDriveStamped` | entra |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` (el "2D Pose Estimate" de RViz) | entra |
| `/emergency_brake` | `std_msgs/Bool` (freno con prioridad absoluta; añadido por websim, no existe en el oficial) | entra |

La física es el mismo modelo *single-track* de `f1tenth_gym` (mismos parámetros, mismo controlador de velocidad y dirección, paso de 10 ms), así que lo que ajustes aquí se comporta igual en el oficial.

## Qué aporta frente al oficial

- **Nada de X11, xhost ni noVNC.** Abres `http://localhost:8080` y ya.
- **Teclado integrado** (WASD) y tres modos de conducción: solo ROS, solo teclado, y *Teclado + ROS*, donde cualquier mensaje en `/drive` manda sobre el teclado durante 0,25 s. Es el montaje ideal para el lab 2: conduces tú hacia la pared y tu nodo frena.
- **Freno de emergencia por ROS.** Un `std_msgs/Bool` en `/emergency_brake` fuerza velocidad 0 por encima de `/drive` y del teclado, igual que la tecla `X`. Dos nodos que publican en `/drive` se pisan (gana el último mensaje, sin más), así que un AEB que compita por ese topic pierde en cuanto el conductor publique más rápido; por este canal no hay carrera. Se libera solo al segundo de dejar de publicarlo, para que el coche no quede bloqueado si matas el nodo.
- **Quién conduce, a la vista**: el panel indica si el último mando vino de ROS, del teclado o del freno, y qué nodos publican en `/drive` y a cuántos Hz.
- **Gráficas en vivo** de velocidad real frente a mandada, y de **cualquier topic tuyo** numérico (`Float64`, `Float32`, `Int*`, `Bool`, `AckermannDriveStamped`, `Odometry`, `Twist`). Publica tu iTTC mínimo en, por ejemplo, `/debug/ittc` y lo verás junto a la velocidad.
- **Pausa, paso a paso y cámara lenta** (×0,1 a ×2) para ver qué pasa en el instante del frenado.
- **Colocar el coche** con el ratón (clic y arrastrar para orientarlo), **dibujar y borrar paredes** en caliente (el LiDAR y las colisiones las ven al momento), **medir** distancias.
- Colisiones detectadas con la huella real del coche, contador, aviso y reinicio automático opcional.
- Cronómetro de vuelta (la salida es la línea a cuadros).

## Instalación

### Opción A — en este workspace (ya está integrado)

El paquete vive en `src/f1tenth_websim` del workspace `Roboracer`, que el
`docker-compose.yml` de la raíz monta en `/f1tenth_ws` y cuyo puerto 8080 ya
está publicado. Solo hay que compilar:

```bash
docker compose up -d
docker compose exec f1tenth_dev bash
# dentro del contenedor:
cd /f1tenth_ws
colcon build --packages-select f1tenth_websim
source install/setup.bash
ros2 launch f1tenth_websim websim_launch.py
```

Abre `http://localhost:8080`.

> Si la imagen es anterior a la integración, reconstrúyela una vez
> (`docker compose build`) para que incluya `python3-scipy` y `python3-pil`.

No lances `gym_bridge_launch.py` a la vez: los dos publicarían en `/scan`. Como
`config/websim.yaml` apunta a `/f1tenth_ws/src/f1tenth_gym_ros/maps`,
**Levine y Spielberg aparecen en el selector** junto a los mapas de websim.

### Opción B — contenedor propio (independiente del workspace)

```bash
cd src/f1tenth_websim
docker compose -f docker/docker-compose.yml up --build
```

Usa la red del host (`network_mode: host`, `ipc: host`). Para que tus nodos lo vean desde tu devcontainer, ese contenedor también debe usar `network_mode: host` e `ipc: host`, y el mismo `ROS_DOMAIN_ID`.

### Opción C — sin ROS, para probar la interfaz

```bash
pip install numpy scipy pyyaml pillow
python3 -m f1tenth_websim.main --no-ros
```

## Uso

| Tecla | Acción |
|---|---|
| `W` `A` `S` `D` o flechas | conducir (en modos *Teclado* y *Teclado + ROS*) |
| `X` | freno de emergencia (fuerza velocidad 0 por encima de todo) |
| `M` | cambiar quién conduce |
| `Espacio` | pausa / reanudar |
| `.` | avanzar 50 ms estando en pausa |
| `R` | volver a la salida |
| `F` | seguir al coche |
| `L` | LiDAR: rayos, puntos, oculto |
| `T` | estela |
| `1`…`5` | herramientas: mover, coche, pared, borrar, medir |

Rueda del ratón para el zoom; arrastrar con el botón derecho mueve la vista con cualquier herramienta.

La estela se colorea por velocidad: turquesa lento, amarillo medio, rojo rápido.

### Flujo recomendado para el lab 2 (AEB)

1. Mapa `pasillo_aeb`, modo **Teclado + ROS**.
2. Lanza tu `safety_node` y comprueba en el panel ROS que aparece como publicador de `/drive`.
3. Publica desde tu nodo el iTTC mínimo en un `std_msgs/Float64` (por ejemplo `/debug/ittc`) y añádelo en *Tus topics*.
4. Acelera con `W` hacia el fondo. Si tu nodo frena, verás el cambio de *Al volante* a ROS, la velocidad mandada caer a 0 y el iTTC cruzar tu umbral.
5. Para analizar el momento exacto: cámara lenta ×0,25 o pausa y avance con `.`.

Si en vez de conducir tú lanzas otro nodo (o un `ros2 topic pub`) que publique en
`/drive`, publica el freno en `/emergency_brake`: compitiendo por `/drive` el que
publique más rápido gana, y el coche acaba contra la pared.

Ten en cuenta además que un criterio de iTTC puro no frena a baja velocidad: se
estabiliza en `distancia = velocidad × umbral`, así que al reducir la velocidad el
coche se va arrimando hasta tocar. Hace falta también una distancia mínima.

## Mapas propios

Cualquier par `.yaml` + `.png` en formato `map_server`. Déjalos en
`src/f1tenth_websim/maps_extra` (ya está en `extra_map_dirs`), añade otra carpeta
en `extra_map_dirs` de `config/websim.yaml`, o pásala con `--maps /ruta`. Opcionalmente, en el `.yaml`:

```yaml
websim_spawn: [x, y, yaw]          # posición de salida
websim_description: "Texto que aparece al pasar el ratón por el selector"
```

Los mapas incluidos se regeneran con `python3 tools/gen_maps.py maps`.

## Diferencias con `f1tenth_gym_ros`

- Añade `/emergency_brake`, que el oficial no tiene. Un nodo que solo publique en `/drive` sigue funcionando igual.
- Un solo coche (sin oponente `/opp_racecar`).
- Al chocar, el coche se detiene y puede salir marcha atrás.
- No publica el modelo URDF del coche. RViz sigue funcionando con `/map`, `/scan` y TF si lo quieres.
- Las marcas de tiempo son de reloj real (igual que el oficial; no hay `/clock`).
