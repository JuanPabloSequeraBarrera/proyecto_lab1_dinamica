import numpy as np

from configuracion_robot import (
    ORIGEN_DIBUJO_GRADOS,
    VELOCIDAD_PRUEBAS,
)

from control_robot import (
    conectar_robot,
    ejecutar_trayectoria_angulos,
    mover_angulos_y_verificar,
)

from fase1_trayectorias import (
    cinematica_inversa,
    trayectoria_circulo,
)


def mover_circulo(mc,centro,radio,numero_puntos=20,velocidad=10,):
    # Círculo local alrededor de la posición home.
    T = trayectoria_circulo(centro=centro,radio=radio,z=0.0,numero_puntos=numero_puntos,)

    angulos = cinematica_inversa(T,l1=0.140,l2_x=0.178,l2_y=-0.005,home=ORIGEN_DIBUJO_GRADOS,)

    print("\nPrimer punto:")
    print(np.round(angulos[0], 2))

    print("\nÁngulos mínimos:")
    print(np.round(np.min(angulos, axis=0), 2))


    print("\nÁngulos máximos:")
    print(np.round(np.max(angulos, axis=0), 2))

    input(
        "\nPresione ENTER para ir al primer punto..."
    )

    # Primero se llega y verifica el punto inicial.
    mover_angulos_y_verificar(
        mc,
        angulos[0].tolist(),
        velocidad,
        tolerancia_grados=2.0,
    )

    input(
        "\nPrimer punto alcanzado."
        "\nPresiona ENTER para comenzar el círculo..."
    )

    # El primer punto ya fue alcanzado.
    ejecutar_trayectoria_angulos(
        mc,
        angulos[1:],
        velocidad,
        tolerancia_grados=2.0,
        timeout_por_punto=10.0,
    )

    return T, angulos


mc = conectar_robot()

try:
    T, angulos = mover_circulo(
        mc,
        centro=(0.0, 0.0),
        radio=0.005,        # Primero prueba un círculo de 5 mm
        numero_puntos=20,   # No empieces con 120
        velocidad=VELOCIDAD_PRUEBAS,
    )

finally:
    try:
        mc.stop()
    finally:
        mc.close()