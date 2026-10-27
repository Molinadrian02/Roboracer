# Roboracer

Workspace ROS 2 Humble para los labs de F1TENTH / Roboracer. Todo corre dentro de
un único contenedor (`docker-compose.yml`), que monta este repositorio en
`/f1tenth_ws`.

## Paquetes

| Paquete | Qué es |
|---|---|
| `src/f1tenth_websim` | **Simulador con interfaz en el navegador** (sustituye al oficial). Mismos topics que `f1tenth_gym_ros`, sin X11 ni noVNC. Ver su [README](src/f1tenth_websim/README.md). |
| `src/f1tenth_gym_ros` | Simulador oficial de F1TENTH (se mantiene como referencia y por sus mapas). |
| `src/lab1_pkg` | Lab 1: `talker` / `relay` (Python y C++). |
| `src/lab2_pkg` | Lab 2: `safety_node` (AEB). Ojo: el paquete ROS se llama `safety_node`, no `lab2_pkg`. |

## Puesta en marcha

```bash
docker compose up -d
docker compose exec f1tenth_dev bash
```

Dentro del contenedor:

```bash
cd /f1tenth_ws
colcon build
source install/setup.bash
```

### Simulador web (recomendado)

```bash
ros2 launch f1tenth_websim websim_launch.py
```

Abre `http://localhost:8080` en el navegador del host. El puerto ya está
publicado en `docker-compose.yml`.

### Simulador oficial (alternativa, necesita X11)

```bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

No lances los dos a la vez: ambos publican en `/scan`.

### Nodos de los labs

```bash
ros2 launch lab1_pkg lab1_launch.py      # lab 1
ros2 run safety_node safety_node.py      # lab 2 (AEB)
```
