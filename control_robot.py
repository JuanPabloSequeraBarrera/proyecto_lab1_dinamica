"""Conexión y movimientos seguros para las pruebas del laboratorio.

El origen del dibujo es un marco de referencia definido por software. Este
módulo no recalibra los ceros mecánicos de los servos.
"""

from math import cos, isfinite, radians, sin
from time import sleep

import numpy as np

from configuracion_robot import (
    INTENTOS_LECTURA,
    PAUSA_LECTURA_S,
    PUERTO_ROBOT,
)


LIMITES_COORDENADAS = (
    (-260.0, 260.0),
    (-260.0, 260.0),
    (-15.0, 357.58),
    (-180.0, 180.0),
)


def _convertir_vector(valor, longitud):
    """Convierte una respuesta válida de la API en una lista de flotantes."""
    if not isinstance(valor, (list, tuple)) or len(valor) != longitud:
        return None

    try:
        vector = [float(elemento) for elemento in valor]
    except (TypeError, ValueError):
        return None

    if not all(isfinite(elemento) for elemento in vector):
        return None
    return vector


def leer_vector(mc, metodo, nombre, longitud=4):
    """Reintenta una lectura porque el controlador puede responder -1 o None."""
    ultima_respuesta = None
    ultimo_error = None

    for _ in range(INTENTOS_LECTURA):
        try:
            ultima_respuesta = metodo()
            vector = _convertir_vector(ultima_respuesta, longitud)
            if vector is not None:
                return vector
        except Exception as error:  # error serial transitorio
            ultimo_error = error
        sleep(PAUSA_LECTURA_S)

    detalle = f"Última respuesta: {ultima_respuesta!r}."
    if ultimo_error is not None:
        detalle += f" Último error: {ultimo_error}."
    raise RuntimeError(f"No fue posible leer {nombre}. {detalle}")


def leer_estado(mc, metodo, nombre, valor_correcto=1):
    """Lee repetidamente un estado escalar hasta obtener el valor esperado."""
    ultima_respuesta = None
    ultimo_error = None

    for _ in range(INTENTOS_LECTURA):
        try:
            ultima_respuesta = metodo()
            if ultima_respuesta == valor_correcto:
                return ultima_respuesta
        except Exception as error:  # error serial transitorio
            ultimo_error = error
        sleep(PAUSA_LECTURA_S)

    detalle = f"Última respuesta: {ultima_respuesta!r}."
    if ultimo_error is not None:
        detalle += f" Último error: {ultimo_error}."
    raise RuntimeError(f"Estado incorrecto al consultar {nombre}. {detalle}")


def conectar_robot(puerto=PUERTO_ROBOT):
    # La dependencia serial se carga solo al conectar hardware. Así, las
    # transformaciones matemáticas pueden probarse sin tener el robot presente.
    from pymycobot import MyPalletizer260

    print(f"Conectando con el robot en {puerto}")
    mc = MyPalletizer260(puerto)

    try:
        sleep(0.5)
        leer_estado(
            mc,
            mc.is_controller_connected,
            "la conexión con el controlador",
        )

        mc.power_on()
        sleep(2.0)

        # Garantiza que no haya quedado activado el modo de movimiento manual.
        mc.set_free_mode(0)
        sleep(0.5)

        leer_estado(mc, mc.is_power_on, "la alimentación")
        leer_estado(mc, mc.is_all_servo_enable, "los cuatro servos")

        angulos = leer_vector(mc, mc.get_angles, "los ángulos")
        coordenadas = leer_vector(mc, mc.get_coords, "las coordenadas")

        print("Controlador: conectado")
        print("Servos: habilitados")
        print("Ángulos:", angulos)
        print("Coordenadas:", coordenadas)
        return mc
    except Exception:
        mc.close()
        raise


def validar_coordenadas(coordenadas):
    """Valida longitud y límites publicados para [x, y, z, rx]."""
    vector = _convertir_vector(coordenadas, 4)
    if vector is None:
        raise ValueError("Las coordenadas deben ser cuatro números: [x, y, z, rx].")

    nombres = ("x", "y", "z", "rx")
    for nombre, valor, (minimo, maximo) in zip(
        nombres, vector, LIMITES_COORDENADAS
    ):
        if not minimo <= valor <= maximo:
            raise ValueError(
                f"{nombre}={valor} está fuera del intervalo "
                f"[{minimo}, {maximo}]."
            )
    return vector


def mover_y_verificar(mc, objetivo, velocidad, tolerancia_mm=5.0):
    """Mueve a una pose cartesiana y comprueba que el robot sí se acercó."""
    objetivo = validar_coordenadas(objetivo)
    print("Moviendo hacia:", [round(valor, 3) for valor in objetivo])

    mc.sync_send_coords(objetivo, velocidad, timeout=20)
    final = leer_vector(mc, mc.get_coords, "las coordenadas finales")

    error_xyz = max(abs(final[i] - objetivo[i]) for i in range(3))
    error_rx = abs(final[3] - objetivo[3])
    print("Pose alcanzada:", final)
    print(f"Error máximo XYZ: {error_xyz:.2f} mm; error rx: {error_rx:.2f}°")

    if error_xyz > tolerancia_mm or error_rx > 5.0:
        raise RuntimeError(
            "El robot terminó demasiado lejos del objetivo. "
            "No se continuará con la trayectoria."
        )
    return final


def trayectoria_local_a_robot(
    trayectoria_m,
    origen_robot,
    giro_grados=0.0,
):
    """Convierte T en metros del marco del dibujo a [x,y,z,rx] del robot.

    El origen del dibujo se expresa en el marco cartesiano de la base. La
    trayectoria local puede rotarse dentro del plano X-Y antes de trasladarse.
    """
    trayectoria = np.asarray(trayectoria_m, dtype=float)
    if trayectoria.ndim != 2 or trayectoria.shape[1] != 3:
        raise ValueError("T debe ser una matriz de tamaño N x 3 en metros.")
    if not np.all(np.isfinite(trayectoria)):
        raise ValueError("T contiene valores no numéricos o infinitos.")

    origen = np.asarray(validar_coordenadas(origen_robot), dtype=float)
    theta = radians(giro_grados)
    rotacion_xy = np.array(
        [
            [cos(theta), -sin(theta)],
            [sin(theta), cos(theta)],
        ]
    )

    desplazamientos_mm = trayectoria * 1000.0
    desplazamientos_mm[:, :2] = desplazamientos_mm[:, :2] @ rotacion_xy.T

    puntos = np.empty((len(trayectoria), 4), dtype=float)
    puntos[:, :3] = origen[:3] + desplazamientos_mm
    puntos[:, 3] = origen[3]

    for punto in puntos:
        validar_coordenadas(punto.tolist())
    return puntos


def ejecutar_trayectoria(mc, puntos_robot, velocidad):
    """Visita secuencialmente una trayectoria ya expresada en el robot."""
    puntos = np.asarray(puntos_robot, dtype=float)
    if puntos.ndim != 2 or puntos.shape[1] != 4:
        raise ValueError("La trayectoria del robot debe tener tamaño N x 4.")

    total = len(puntos)
    for indice, punto in enumerate(puntos, start=1):
        objetivo = validar_coordenadas(punto.tolist())
        mc.sync_send_coords(objetivo, velocidad, timeout=20)
        if indice == 1 or indice == total or indice % 10 == 0:
            print(f"Punto {indice}/{total}")
