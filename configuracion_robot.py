"""Parámetros compartidos por las pruebas físicas del myPalletizer 260."""

# En el computador del laboratorio el robot respondió en este puerto.
PUERTO_ROBOT = "COM5"

# No se fija el baudrate aquí: MyPalletizer260 usa 115200 por defecto y esa
# fue la configuración que sí permitió comunicarse con el robot.

# Pose cartesiana del origen local del dibujo en el marco de la base.
# Unidades: [mm, mm, mm, grados].
ORIGEN_DIBUJO_MM = [29.2, 16.3, 169.2, -4.39]

# Giro del marco local del dibujo respecto a los ejes X-Y de la base.
GIRO_DIBUJO_GRADOS = 0.0

VELOCIDAD_PRUEBAS = 10
INTENTOS_LECTURA = 10
PAUSA_LECTURA_S = 0.25

