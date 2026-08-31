#a jp le gusta el pene
"""Fase 1 del preinforme: crear y visualizar trayectorias planas.

En esta fase T es una matriz de N puntos. Cada fila contiene [x, y, z]
en metros. Como el laser dibuja sobre un plano, z permanece constante.

Esta NO es todavia la simulacion completa del robot: faltan la cinematica
directa, el Jacobiano, la cinematica inversa, Open3D y las colisiones.
"""

import numpy as np
import matplotlib.pyplot as plt


def trayectoria_circulo(centro, radio, z, numero_puntos=120):
    """Devuelve una trayectoria circular cerrada de forma (N, 3)."""
    angulo = np.linspace(0.0, 2.0 * np.pi, numero_puntos)
    x = centro[0] + radio * np.cos(angulo)
    y = centro[1] + radio * np.sin(angulo)
    return np.column_stack((x, y, np.full_like(x, z)))


def trayectoria_poligono(centro, radio, z, numero_lados, puntos_por_lado=20):
    """Devuelve un poligono regular cerrado: triangulo, cuadrado, hexagono, etc."""
    if numero_lados < 3:
        raise ValueError("Un poligono debe tener al menos 3 lados.")

    angulos = np.linspace(0.0, 2.0 * np.pi, numero_lados, endpoint=False)
    vertices_xy = np.column_stack((
        centro[0] + radio * np.cos(angulos),
        centro[1] + radio * np.sin(angulos),
    ))
    vertices_xy = np.vstack((vertices_xy, vertices_xy[0]))

    segmentos = []
    for inicio, final in zip(vertices_xy[:-1], vertices_xy[1:]):
        fraccion = np.linspace(0.0, 1.0, puntos_por_lado, endpoint=False)[:, None]
        segmentos.append(inicio + fraccion * (final - inicio))

    puntos_xy = np.vstack(segmentos)
    puntos_xy = np.vstack((puntos_xy, puntos_xy[0]))
    return np.column_stack((puntos_xy, np.full(len(puntos_xy), z)))


def simular_mypalletizer(L1, L2, L3, L4, T):
    """Valida y muestra las trayectorias T (circulo, estre).

    Esta primera version solo estudia la trayectoria cartesiana. La prueba
    distancia <= L1 + L2 + L3 + L4 es necesaria, pero aun no garantiza que
    exista una configuracion articular valida; eso se comprobara con la IK.
    """
    longitudes = np.asarray([L1, L2, L3, L4], dtype=float)
    if np.any(longitudes <= 0.0):
        raise ValueError("Todas las longitudes deben ser positivas.")

    T = np.asarray(T, dtype=float)
    if T.ndim != 2 or T.shape[1] not in (2, 3):
        raise ValueError("T debe tener forma (N, 2) o (N, 3).")
    if T.shape[1] == 2:
        T = np.column_stack((T, np.zeros(len(T))))

    distancia_origen = np.linalg.norm(T, axis=1)
    alcance_maximo_teorico = np.sum(longitudes)
    fuera_alcance = distancia_origen > alcance_maximo_teorico

    print(f"Numero de puntos: {len(T)}")
    print(f"Alcance maximo teorico: {alcance_maximo_teorico:.3f} m")
    print(f"Distancia maxima de T al origen: {distancia_origen.max():.3f} m")
    print(f"Puntos que fallan la prueba inicial: {fuera_alcance.sum()}")

    plt.figure(figsize=(7, 7))
    plt.plot(T[:, 0], T[:, 1], color="tab:red", linewidth=2, label="Trayectoria T")
    plt.scatter(T[0, 0], T[0, 1], color="tab:green", s=60, label="Inicio", zorder=3)

    if np.any(fuera_alcance):
        plt.scatter(T[fuera_alcance, 0], T[fuera_alcance, 1],
                    color="black", marker="x", label="Fuera de alcance teorico")

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trayectoria plana del laser")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    return T


if __name__ == "__main__":
    # Valores provisionales: reemplazarlos por las longitudes medidas del robot.
    L1, L2, L3, L4 = 0.06, 0.08, 0.08, 0.04

    centro = (0.14, 0.00)
    altura_plano = 0.06

    T_circulo = trayectoria_circulo(centro, 0.035, altura_plano)
    T_cuadrado = trayectoria_poligono(centro, 0.035, altura_plano, 4)
    T_triangulo = trayectoria_poligono(centro, 0.035, altura_plano, 3)
    T_hexagono = trayectoria_poligono(centro, 0.035, altura_plano, 6)
    #T_estrella = trayectoria_estrella(centro, 0.040, 0.018, altura_plano)

    # Cambia T_circulo por cualquiera de las otras trayectorias para probarla.
    simular_mypalletizer(L1, L2, L3, L4, T_hexagono)
