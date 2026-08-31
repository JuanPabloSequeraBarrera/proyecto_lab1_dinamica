import numpy as np

from pymycobot import MyPalletizer260
from pymycobot import utils

from fase1_trayectorias import trayectoria_circulo


puerto = utils.get_port_list()[0]

mc = MyPalletizer260(
    puerto,
    1000000
)

mc.power_on()


# posición actual del robot

centro_robot = mc.get_coords()

print("Centro robot:")
print(centro_robot)

x0, y0, z0, rx0 = centro_robot


# círculo matemático

T = trayectoria_circulo(
    centro=(0, 0),
    radio=0.005,
    z=0,
    numero_puntos=20
)


# convertir metros -> mm

puntos_robot = []

for x, y, z in T:

    punto = [

        x0 + 1000*x,
        y0 + 1000*y,
        z0 + 1000*z,
        rx0

    ]

    puntos_robot.append(punto)


print("\nPuntos que recibirá el robot:")

for punto in puntos_robot:

    print(np.round(punto, 2))


input("\nENTER para mover al inicio...")

try:

    mc.sync_send_coords(
        puntos_robot[0],
        10
    )

    input(
        "Está en el inicio. "
        "ENTER para comenzar círculo..."
    )

    for punto in puntos_robot:

        mc.sync_send_coords(
            punto,
            10
        )

except KeyboardInterrupt:

    mc.stop()

    print("Movimiento detenido.")