import numpy as np
import sympy as sp 
import time
from configuracion_robot import VELOCIDAD_PRUEBAS
from control_robot import (
    conectar_robot,
    ejecutar_trayectoria_angulos,
    mover_angulos_y_verificar,
    obtener_estado
)
from fase1_trayectorias import cinematica_inversa,trayectoria_circulo

def mover_circulo(mc, centro, radio, z, l1, l2, numero_puntos=120, velocidad=30):

    # 1. Crear trayectoria del círculo
    T = trayectoria_circulo(
        centro,
        radio,
        z,
        numero_puntos
    )

    # 2. Convertir XYZ a ángulos
    angulos = cinematica_inversa(
        T,
        l1,
        l2
    )
    # print(angulos)
    


    # 3. Mandar los ángulos al robot
    ejecutar_trayectoria_angulos(
        mc,
        angulos,
        velocidad
    )

    return T, angulos

mc = conectar_robot()
l1 = 0.20
l2 = 0.20

centro = [0.0, 0.0]
radio = 0.05
z = 0.0

T, angulos = mover_circulo(
    mc,
    centro,
    radio,
    z,
    l1,
    l2
)