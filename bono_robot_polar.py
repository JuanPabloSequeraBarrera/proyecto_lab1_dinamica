import numpy as np
import serial
import time


# ============================================================
# PARAMETROS DEL MECANISMO
# ============================================================

l1 = 51.3 / 1000
l2 = 126.6 / 2000


# ============================================================
# CINEMATICA INVERSA
# ============================================================

def cin_inv(x, y, l1, l2):

    r = np.sqrt(x**2 + y**2)

    theta0 = np.arctan2(x, y)

    theta1 = 2*(
        np.arccos(
            (l1**2 - (r/2)**2 - l2**2) /
            (-2 * (r/2) * l2)
        )
    )

    return np.degrees(
        np.column_stack([theta0, theta1])
    )


# ============================================================
# LEER CSV
# ============================================================

datos = np.loadtxt(
    #"trayectoria_09_Cruz_irregular.csv",
    "trayectoria_15_Rosa_multipetalo_v2.csv",
    delimiter=",",
    skiprows=1,
    usecols=(3,4)
)

x = datos[:, 0]
y = datos[:, 1]


# ============================================================
# CENTRO DE LA FIGURA
# ============================================================

centro_x = (np.max(x) + np.min(x)) / 2
centro_y = (np.max(y) + np.min(y)) / 2


# ============================================================
# POSICION DE REFERENCIA
#
# Queremos:
#
# theta0 = 0°
# theta1 = 60°
#
# Esto corresponde a:
#
# x = 0
# y = 0.190385 m aproximadamente
# ============================================================

theta1_centro = np.radians(90)

a = (
    l2 * np.cos(theta1_centro / 2)
    + np.sqrt(
        l1**2 -
        l2**2 * np.sin(theta1_centro / 2)**2
    )
)

radio_centro = 2 * a


# ============================================================
# CENTRAR LA FIGURA EN LA POSICION DE REFERENCIA
# ============================================================

x = x - centro_x

y = y - centro_y + radio_centro


# ============================================================
# CINEMATICA INVERSA
# ============================================================

trayectoria = cin_inv(x, y, l1, l2)





# ============================================================
# COMPROBAR LIMITES
# ============================================================

if np.any(trayectoria[:, 0] < -60) or np.any(trayectoria[:, 0] > 60):

    print("ERROR: theta0 sale de los limites")
    print("Min theta0:", np.min(trayectoria[:, 0]))
    print("Max theta0:", np.max(trayectoria[:, 0]))

    raise ValueError("Trayectoria fuera de rango")


if np.any(trayectoria[:, 1] < 0) or np.any(trayectoria[:, 1] > 120):

    print("ERROR: theta1 sale de los limites")
    print("Min theta1:", np.min(trayectoria[:, 1]))
    print("Max theta1:", np.max(trayectoria[:, 1]))

    raise ValueError("Trayectoria fuera de rango")


# ============================================================
# MOSTRAR LIMITES DE LA TRAYECTORIA
# ============================================================


print("theta0 minimo:", np.min(trayectoria[:, 0]))
print("theta0 maximo:", np.max(trayectoria[:, 0]))
print("theta1 minimo:", np.min(trayectoria[:, 1]))
print("theta1 maximo:", np.max(trayectoria[:, 1]))



# ============================================================
# USAR TODOS LOS PUNTOS
# ============================================================

trayectoria_enviada = trayectoria


# ============================================================
# CONECTAR CON ARDUINO
# ============================================================

arduino = serial.Serial(
    "COM8",
    115200,
    timeout=2
)

time.sleep(2)


# ============================================================
# MANDAR CANTIDAD DE PUNTOS
# ============================================================

arduino.write(
    f"{len(trayectoria_enviada)}\n".encode()
)

time.sleep(0.5)


# ============================================================
# MANDAR TRAYECTORIA
# ============================================================

for theta0, theta1 in trayectoria_enviada:

    arduino.write(
        f"{theta0:.3f} {theta1:.3f}\n".encode()
    )

    # Esperar aproximadamente lo mismo que
    # tarda Arduino en procesar el punto
    time.sleep(0.33)


# ============================================================
# FINAL
# ============================================================

print("Trayectoria enviada")

time.sleep(2)

arduino.close()