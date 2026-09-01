"""Mueve el robot al origen local elegido para las figuras."""

from configuracion_robot import ORIGEN_DIBUJO_MM, VELOCIDAD_PRUEBAS
from control_robot import conectar_robot, mover_y_verificar


mc = conectar_robot()

try:
    print("\nOrigen cartesiano del dibujo:", ORIGEN_DIBUJO_MM)
    input("Retira las manos. ENTER para mover el robot a este origen...")
    mover_y_verificar(mc, ORIGEN_DIBUJO_MM, VELOCIDAD_PRUEBAS)
    print("Origen local establecido por software.")
finally:
    mc.close()
