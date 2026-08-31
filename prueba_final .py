import numpy as np

from configuracion_robot import (
    GIRO_DIBUJO_GRADOS,
    ORIGEN_DIBUJO_MM,
    VELOCIDAD_PRUEBAS,
)
from control_robot import (
    conectar_robot,
    ejecutar_trayectoria,
    mover_y_verificar,
    trayectoria_local_a_robot,
)
from fase1_trayectorias import trayectoria_circulo


# T vive en el marco local del dibujo y usa metros.
T = trayectoria_circulo(
    centro=(0.0, 0.0),
    radio=0.005,
    z=0.0,
    numero_puntos=36,
)

# Los comandos de pymycobot viven en el marco de la base y usan milímetros.
puntos_robot = trayectoria_local_a_robot(
    T,
    ORIGEN_DIBUJO_MM,
    GIRO_DIBUJO_GRADOS,
)

print("Origen local del dibujo:", ORIGEN_DIBUJO_MM)
print("Primeros cinco puntos que recibirá el robot:")
for punto in puntos_robot[:5]:
    print(np.round(punto, 2).tolist())

mc = conectar_robot()

try:
    input(
        "\nRetira las manos y verifica el recorrido. "
        "ENTER para ir al origen del dibujo..."
    )
    mover_y_verificar(
        mc,
        ORIGEN_DIBUJO_MM,
        VELOCIDAD_PRUEBAS,
    )
    print("Origen local establecido por software.")

    input("ENTER para ir al primer punto del círculo...")
    mover_y_verificar(
        mc,
        puntos_robot[0].tolist(),
        VELOCIDAD_PRUEBAS,
    )

    input("ENTER para trazar el círculo...")
    ejecutar_trayectoria(
        mc,
        puntos_robot[1:],
        VELOCIDAD_PRUEBAS,
    )
    print("Círculo terminado.")
except KeyboardInterrupt:
    mc.stop()
    print("Movimiento detenido.")
finally:
    mc.close()
