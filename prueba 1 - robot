from pymycobot import MyPalletizer260
from pymycobot import utils

puerto = utils.get_port_list()[0]

mc = MyPalletizer260(puerto, 1000000)

mc.power_on()

# User defines where the laser should go
puntos = [
    [150, 50, 100, 0],
    [160, 50, 100, 0],
    [160, 60, 100, 0],
    [150, 60, 100, 0],
    [150, 50, 100, 0]
]

print("Puntos que se enviarán al robot:")

for punto in puntos:
    print(punto)

input("\nPresiona ENTER para comenzar...")

try:

    for punto in puntos:

        mc.sync_send_coords(
            punto,
            10
        )

except KeyboardInterrupt:

    mc.stop()

    print("Movimiento detenido.")
