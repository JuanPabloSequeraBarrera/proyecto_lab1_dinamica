from pymycobot import MyPalletizer260
from pymycobot import utils

puerto = utils.get_port_list()[0]

mc = MyPalletizer260(puerto, 1000000)

mc.power_on()

actual = mc.get_coords()
actual2 = [0, 0, 0, 0]#MODIFICAR ANTES DE INCIAR!

print("Posición actual:")
print(actual)

if actual is None or len(actual) != 4:
    raise RuntimeError("No pude leer correctamente las coordenadas.")

x, y, z, rx = actual

objetivo = [
    x + 5,
    y,
    z,
    rx
]

print("\nEl robot intentará ir a:")
print(objetivo)

input("\nENTER para mover SOLO 5 mm en X...")

try:

    mc.sync_send_coords(
        objetivo,
        10
    )

except KeyboardInterrupt:

    mc.stop()
    print("Movimiento detenido.")

print("\nPosición final:")
print(mc.get_coords())

input("ENTER para regresar...")

mc.sync_send_coords(
    actual2,
    10
)
#SI SE QUIERE VOLVER AL PUNTO ORIGINAL
