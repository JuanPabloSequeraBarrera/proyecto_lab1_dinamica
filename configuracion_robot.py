"""Parámetros pruebas físicas del myPalletizer 260."""
# No se fija el baudrate aquí: MyPalletizer260 usa 115200 por defecto y esa
# fue la configuración que sí permitió comunicarse con el robot.

# Pose cartesiana del origen local del dibujo en el marco de la base.
# Unidades: [mm, mm, mm, grados].
ORIGEN_DIBUJO_MM = [171.6, -5.5, 209.4, -7.03]
ORIGEN_DIBUJO_GRADOS = [-2.19, 4.13, 1.66, 0.52]
PUERTO_ROBOT = "/dev/cu.usbserial-5AE20106981"
# Giro del marco local del dibujo respecto a los ejes X-Y de la base.
GIRO_DIBUJO_GRADOS = 0

VELOCIDAD_PRUEBAS = 10
INTENTOS_LECTURA = 10
PAUSA_LECTURA_S = 0.25



