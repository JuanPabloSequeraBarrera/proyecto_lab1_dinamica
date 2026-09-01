import numpy as np
import sympy as sp 
from configuracion_robot import VELOCIDAD_PRUEBAS
from control_robot import (
    conectar_robot,
    ejecutar_trayectoria_angulos,
    mover_angulos_y_verificar,
    validar_angulos,
)
from fase1_trayectorias import pasd
