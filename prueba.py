import numpy as np
import sympy as sp 
from configuracion_robot import VELOCIDAD_PRUEBAS
from control_robot import (
    conectar_robot,
    ejecutar_trayectoria_angulos,
    mover_angulos_y_verificar,
    validar_angulos, cinematica_inversa
)
from fase1_trayectorias import pasd

def mover_desde_xy(mc, x, y, l1, l2, velocidad=30):

    angulos = cinematica_inversa(x, y, l1, l2)

    print("Ángulos calculados:", angulos)

    mover_angulos_y_verificar(
        mc,
        angulos,
        velocidad
    )

    return angulos
