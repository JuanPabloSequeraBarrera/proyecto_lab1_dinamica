from pymycobot import MyPalletizer260
from pymycobot import utils
import time
from control_robot import (
    conectar_robot,
)

print("Puertos disponibles:")

puertos = utils.get_port_list()

for i, puerto in enumerate(puertos):
    print(i, puerto)

if len(puertos) == 0:
    raise RuntimeError("No se encontró ningún puerto serial.")

puerto = puertos[0]

print(f"\nIntentando conectar a: {puerto}")

mc = MyPalletizer260(puerto, 115200)

print("Controlador:", mc.is_controller_connected())
print("Ángulos:", mc.get_angles())
print("Coordenadas:", mc.get_coords())

mc.release_all_servos()
try:
    while True:

        angulos = mc.get_angles()

        if angulos is not None and angulos != -1:
            print("Ángulos instantáneos:", angulos)
        else:
            print("No se pudieron leer los ángulos.")

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nLectura detenida.")