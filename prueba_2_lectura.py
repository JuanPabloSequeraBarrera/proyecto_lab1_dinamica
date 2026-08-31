from pymycobot import MyPalletizer260
from pymycobot import utils

puerto = utils.get_port_list()[0]

mc = MyPalletizer260(puerto, 115200)

mc.power_on()

coords = mc.get_coords()
angles = mc.get_angles()

print("POSICIÓN ACTUAL")
print("coords =", coords)
print("angles =", angles)

#HAY QUE ANOTAR LAS CORDENADAS PORQUE ESAS SERAN EL PUNTO DE REFERENCIA
