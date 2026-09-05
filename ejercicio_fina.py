import pandas as pd 
import numpy as np

from fase1_trayectorias import cinematica_inversa
from configuracion_robot import ORIGEN_DIBUJO_GRADOS
from control_robot import (
    conectar_robot,
    mover_angulos_y_verificar, 
    validar_angulos
)


datos = pd.read_csv("/Users/juansequerabarrera/Documents/uniandes/fifth semester/dinamica/lab1/pymycobot/trayectoria_19_Cruz_irregular_v2.csv")
x = datos["x_m"].to_numpy(dtype=float)
y = datos["y_m"].to_numpy(dtype=float)

x_home = 0.1876790733
y_home = -0.1266538283

x_local = x - x_home
y_local = y - y_home
z = np.zeros_like(x)
trayectoria = np.column_stack([x_local,y_local,z])

angulos = cinematica_inversa(trayectoria,l1=0.140,l2_x=0.178,l2_y=-0.005,home=ORIGEN_DIBUJO_GRADOS)
print(angulos)


