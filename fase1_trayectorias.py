
import numpy as np
import matplotlib.pyplot as plt
import sympy as sp
import sympy.physics.mechanics as me
from configuracion_robot import ORIGEN_DIBUJO_GRADOS

def trayectoria_poligono(centro, radio, z, numero_lados, puntos_por_lado=20): #el radio aquí es el radio de la circurferencia imaginaria de un poligono, o sea donde viven los vertices
    """Devuelve un poligono regular cerrado: triangulo, cuadrado, hexagono, etc."""
    if numero_lados < 3:
        return ("Un poligono debe tener al menos 3 lados.")

    angulos = np.linspace(0.0, 2.0 * np.pi, numero_lados, endpoint=False) #aquí como en la funcion anterior se crean angulos de 0 a 2pi equidistantes para los vertices del poligono 
    #el endpoint false es para que se tomen los valores desde o hasta 2pi sin incluirlo
    vertices_xy = np.column_stack((
        centro[0] + radio * np.cos(angulos), #como todo poligono tiene un circulo interno podemos usar la misma ecuacion de un circulo para definir esa posicion de n vertices 
        centro[1] + radio * np.sin(angulos), #esto es una tupla que usa otra vez column stack que une todo en columnas en funcion de cuantos valores de angulos(vertices) mande linspace
    ))
    vertices_xy = np.vstack((vertices_xy, vertices_xy[0]))
    #esta linea cierra el poligno, lo unico que hace es que apila todo lo que está en vertices_xy y le agrega el primer vertice para terminar la trayectoria

    segmentos = [] #aqí se van a guardar todos los lados del poligono que van de vertice a vertice
    
    #El vertices_xy[:-1] es pedirle a la lista de listas de vertices que te de todos los vertices menos el ultimo que se usa para cerrar el poligono
    #El vertices_xy[1:] te da todos los elementos desde la posicion 1 hasta la posicion final (o sea el segundo vertice)
    #Zip lo unico que hace es que empareja estos dos elementos y te da los lados del poligono 
    for inicio, final in zip(vertices_xy[:-1], vertices_xy[1:]): 
        #aqui en este for lo que se hace es que a partir de los puntos guardados en el zip se toma un punto de inicio A[a,b] y final B[c,d] y se opera en cada ciclo
        fraccion = np.linspace(0.0, 1.0, puntos_por_lado, endpoint=False)[:, None] #en esta linea la distancia de A hasta B se parte en pedazos equidistantes de puntos seleccionados y se excluye el punto final
        #esto es simplemente un operador matematico que hace cabeza menos cola, lo multiplica por la cantidad de puntos que están en la trayectoria y suma inicio 
        segmentos.append(inicio + fraccion * (final - inicio))
    
    puntos_xy = np.vstack(segmentos) #vstack solamente apila los datos verticalmente, es una sola matriz vertical[[x0,y0],[x1,y1]]
    puntos_xy = np.vstack((puntos_xy, puntos_xy[0])) #se agrega a los puntos el primer punto de la trayectoria al final para que cierre la figura
    return np.column_stack((puntos_xy, np.full(len(puntos_xy), z)))
    #este np.full crea tantos vectores de z valor como la longitud de puntos y lo apila para devolver un vector [[x0,y0,z],[x1,y1,z],...] 






def simular_mypalletizer(L1, L2, L3, L4, T): #las cuatro longitudes de brazo de robot y T la trayectoria generada por las anteriores funciones
    """Muestra la trayectoria T  que puede ser (circulo,cuadrado,triangulo, hexagono).
    """
    longitudes = np.asarray([L1, L2, L3, L4], dtype=float)
    if np.any(longitudes <= 0.0):
        raise ValueError("Todas las longitudes deben ser positivas.")


    plt.figure(figsize=(7, 7)) #Crea una figura 7*7 pulgadas
    plt.plot(T[:, 0], T[:, 1], color="tab:red", linewidth=2, label="Trayectoria T") #aquí tenemos del arreglo T separamaos x como columna 1 y columna 2 y
    plt.scatter(T[0, 0], T[0, 1], color="tab:green", s=60, label="Inicio", zorder=3) #ESTA LINEA UNICAMENTE SIRVE PARA UBICAR EL PUNTO INCICAL DE LA TRRATECTORIA
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Trayectoria plana del laser")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()

    plt.show()
    return T

def trayectoria_circulo(centro, radio, z, numero_puntos=120):
    
    angulo = np.linspace(0.0, 2.0 * np.pi, numero_puntos) #el operador linspace crea puntos equidistanes , en este caso le estoy diciendo (incie en 0, termine en pi, haga 120 puntos)
    x = centro[0] + radio * np.cos(angulo) #aqui se usa la ecuacion de un circulo(x o y = centro[cordanada x o y] + radio*coseno(angulo n))
    y = centro[1] + radio * np.sin(angulo)# esta es la componente y del centro 
    return np.column_stack((x, y, np.full_like(x, z)))
    #full like crea un vector de valores z con la misma dimension de los valores de x pero con su valor z, digamos, si x tiene 5 elementos z también tendrá 5-
    #solamente que tendrá el valor de z, en este caso 0 que es la altura del plano 
    #column stack simplemente une los tres valores que le mandas en cada columna y lo manda como una lista de listas que el robot debe visitar
    # el return sería algo como [[x0,y0,z(este sería el mismo valor de z siempre)],[x1,y1,z],[x2,y2,z],[]}
    #full like crea un vector de valores z con la misma dimension de los valores de x pero con su valor z, digamos, si x tiene 5 elementos z también tendrá 5-
    #solamente que tendrá el valor de z, en este caso 0 que es la altura del plano 
    #column stack simplemente une los tres valores que le mandas en cada columna y lo manda como una lista de listas que el robot debe visitar
    # el return sería algo como [[x0,y0,z(este sería el mismo valor de z siempre)],[x1,y1,z],[x2,y2,z],[]}





def cinematica_inversa(T,l1=0.140,l2_x=0.178,l2_y=-0.005,home=None):
    """
    Convierte desplazamientos locales [x, y, z] en ángulos 
    [J1, J2, J3, J4].
    T representa desplazamientos respecto a la posición home.
    """

    trayectoria = np.asarray(T, dtype=float)
    
    home = ORIGEN_DIBUJO_GRADOS

    home = np.asarray(home, dtype=float)

    l2 = np.hypot(l2_x, l2_y)
    beta = np.arctan2(l2_y, l2_x)

    # Ángulos de J2 y J3 en la posición home.
    j2_home = np.deg2rad(home[1])
    j3_home = np.deg2rad(home[2])

    # Conversión entre los ángulos del motor y el modelo 2R.
    theta1_home = j2_home - np.pi / 2.0
    theta2_home = j3_home + np.pi / 2.0 + beta

    # Posición cartesiana correspondiente al home.
    x_home = (l1 * np.cos(theta1_home)+ l2 * np.cos(theta1_home + theta2_home))

    y_home = (l1 * np.sin(theta1_home)+ l2 * np.sin(theta1_home + theta2_home))

    # T contiene desplazamientos alrededor del home.
    x = x_home + trayectoria[:, 0]
    y = y_home + trayectoria[:, 1]

    cos_theta2 = (x**2 + y**2 - l1**2 - l2**2) / (2.0 * l1 * l2)


    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    # Mantener la misma configuración de codo que el home.
    signo_codo = 1.0 if theta2_home >= 0.0 else -1.0
    theta2 = signo_codo * np.arccos(cos_theta2)

    theta1 = np.arctan2(y, x) - np.arctan2(l2 * np.sin(theta2),l1 + l2 * np.cos(theta2),)

    # Conversión del modelo matemático a ángulos del robot.
    j2 = np.rad2deg(theta1 + np.pi / 2.0)
    j3 = np.rad2deg(theta2 - np.pi / 2.0 - beta)

    # J1 y J4 permanecen en sus valores home.
    j1 = np.full_like(j2, home[0])
    j4 = np.full_like(j2, home[3])

    angulos = np.column_stack((j1, j2, j3, j4))

    return angulos




# def cinematica_inversa(T,l1,l2):
#     # Sacamos directamente las columnas x e y de la trayectoria
#     x = T[:, 0]
#     y = T[:, 1]
#     home = [-2.19, 4.13, 1.66, 0.52]
#     x = x - 0.0347
#     y = y + 0.0529

#     r = np.sqrt(x**2 + y**2)
#     alpha = np.arccos((-r**2 + l1**2 + l2**2) / (2 * l1 * l2))

#     q2 = -alpha + np.pi

#     phi = np.arcsin((l2 * np.sin(alpha)) / r)

#     theta = np.arctan2(y, x)
#     q1 = theta - phi

#     angulos = np.column_stack((np.rad2deg(np.zeros_like(q1) - home[0]),np.rad2deg(q1 - home[1]),np.rad2deg(q2 - home[2]),np.rad2deg(np.zeros_like(q1) - home[3])))
#     print(angulos)
#     return angulos

if __name__ == "__main__":

    centro = (0.0, 0.0)
    altura_plano = 0.0

    T_circulo = trayectoria_circulo(
        centro=centro,
        radio=1,
        z=altura_plano,
        numero_puntos=20,
    )

    angulos = cinematica_inversa(
        T_circulo,
        l1=0.140,
        l2_x=0.178,
        l2_y=-0.005,
        home=[-2.19, 4.13, 1.66, 0.52],
    )

    print("\nÁngulos calculados:")
    print(np.round(angulos, 2))

    print("\nMínimos:")
    print(np.round(np.min(angulos, axis=0), 2))

    print("\nMáximos:")
    print(np.round(np.max(angulos, axis=0), 2))
    #T_cuadrado = trayectoria_poligono(centro, 0.035, altura_plano, 4)
    #T_triangulo = trayectoria_poligono(centro, 0.035, altura_plano, 3)
    #T_hexagono = trayectoria_poligono(centro, 0.035, altura_plano, 6)
    #simular_mypalletizer(L1, L2, L3, L4, T_circulo)