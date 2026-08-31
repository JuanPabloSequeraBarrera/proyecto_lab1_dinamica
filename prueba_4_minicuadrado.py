from pymycobot import MyPalletizer260
from pymycobot import utils

puerto = utils.get_port_list()[0]

mc = MyPalletizer260(puerto, 1000000)

mc.power_on()

centro = mc.get_coords()

print("Centro:")
print(centro)

x0, y0, z0, rx0 = centro

d = 5  # mm

puntos = [

    [x0,     y0,     z0, rx0],
    [x0 + d, y0,     z0, rx0],
    [x0 + d, y0 + d, z0, rx0],
    [x0,     y0 + d, z0, rx0],
    [x0,     y0,     z0, rx0]

]

print("\nTrayectoria:")
for p in puntos:
    print(p)

input("\nENTER para comenzar cuadrado de 5 mm...")

try:

    for punto in puntos:

        mc.sync_send_coords(
            punto,
            10
        )

except KeyboardInterrupt:

    mc.stop()

    print("Movimiento detenido.")