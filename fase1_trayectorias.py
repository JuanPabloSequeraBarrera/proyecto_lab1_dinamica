"""Fase 1 del preinforme: crear y visualizar trayectorias planas.

En esta fase T es una matriz de N puntos. Cada fila contiene [x, y, z]
en metros. Como el laser dibuja sobre un plano, z permanece constante.

Esta NO es todavia la simulacion completa del robot: faltan la cinematica
directa, el Jacobiano, la cinematica inversa, Open3D y las colisiones.
"""

import numpy as np
import matplotlib.pyplot as plt
import sympy as sp
import sympy.physics.mechanics as me


def trayectoria_circulo(centro, radio, z, numero_puntos=120):
    """
    Esta función recibe 4 parametros, el centro que en todos los casos es (0,0) en xy, el radio que es que tan grande es el circulo, z que es una altura aproximada
    """
    
    angulo = np.linspace(0.0, 2.0 * np.pi, numero_puntos) #el operador linspace crea puntos equidistanes , en este caso le estoy diciendo (incie en 0, termine en pi, haga 120 puntos)
    x = centro[0] + radio * np.cos(angulo) #aqui se usa la ecuacion de un circulo(x o y = centro[cordanada x o y] + radio*coseno(angulo n))
    y = centro[1] + radio * np.sin(angulo)# esta es la componente y del centro 
    return np.column_stack((x, y, np.full_like(x, z)))
    #full like crea un vector de valores z con la misma dimension de los valores de x pero con su valor z, digamos, si x tiene 5 elementos z también tendrá 5-
    #solamente que tendrá el valor de z, en este caso 0 que es la altura del plano 
    #column stack simplemente une los tres valores que le mandas en cada columna y lo manda como una lista de listas que el robot debe visitar
    # el return sería algo como [[x0,y0,z(este sería el mismo valor de z siempre)],[x1,y1,z],[x2,y2,z],[]}

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


if __name__ == "__main__":
    #hay que cambiar esto por las longitudes del robot que tengamos
    L1, L2, L3, L4 = 0.06, 0.08, 0.08, 0.04

    centro = (0.00, 0.00)
    altura_plano = 0 #revisar mañana si cambiar

    T_circulo = trayectoria_circulo(centro, 0.035, altura_plano)
    T_cuadrado = trayectoria_poligono(centro, 0.035, altura_plano, 4)
    T_triangulo = trayectoria_poligono(centro, 0.035, altura_plano, 3)
    T_hexagono = trayectoria_poligono(centro, 0.035, altura_plano, 6)
    #T_estrella = trayectoria_estrella(centro, 0.040, 0.018, altura_plano)

    simular_mypalletizer(L1, L2, L3, L4, T_circulo)

# Cinematica directa
#Vairbales
q1, q2, q3, q4 = me.dynamicsymbols('q1 q2 q3 q4')

#Longitudes del robot
L1 = 0.115
L2 = 0.130
L3 = 0.130
L4 = 0.03412

#posicion en x del laser, q1:rotacion base, q2: angulo primer joint, q2+q3: angulo 2nd joint, q2+q3+q4: angulo last joint
x_laser = sp.cos(q1) * (
    L2 * sp.cos(q2)
    + L3 * sp.cos(q2 + q3)
    + L4 * sp.cos(q2 + q3 + q4)
)

#posicion en y del laser
y_laser = sp.sin(q1) * (
    L2 * sp.cos(q2)
    + L3 * sp.cos(q2 + q3)
    + L4 * sp.cos(q2 + q3 + q4)
)

#posicion en z del laser
z_laser = (
    L1
    + L2 * sp.sin(q2)
    + L3 * sp.sin(q2 + q3)
    + L4 * sp.sin(q2 + q3 + q4)
)

fk = sp.lambdify( #funcion convierte las ecuaciones simbolicas a una funcion numerica, returns las 3 coordenadas del laser
    (q1, q2, q3, q4),
    (x_laser, y_laser, z_laser),
    'numpy'
)

#recibe angulos calcula posicion
def cinematica_directa(q1_val, q2_val, q3_val, q4_val):

    x, y, z = fk( #pone los angulos en la ecuacion
        q1_val,
        q2_val,
        q3_val,
        q4_val
    )

    return np.array([x, y, z], dtype=float) #devuelve posicon del laser como vector



#Cinematica Inversa
def cinematica_inversa(x, y, z): #recibimos coordenadas and we output angulos para llegar a esa posicion
    L1, L2, L3, L4 = 0.115, 0.13, 0.13, 0.03412 # robot lengthss

    #rotacion base
    q1_sol = np.arctan2(y, x) #arctan2 encuentra angulo de la base a partir de coordenadas + las de en el cuadrante que es
    r = np.sqrt(x**2 + y**2) # distancia radial
    z_prima = z - L1 # elimina altura inicial para trabajar con la posicion del laser respecto primera articulacion

    L34 = L3 + L4 #assuming q4=0 (L4 y L3 alineados)

    cos_q3 = ( #cosine rule para encontrar el coseno del angulo q3
        r**2
        + z_prima**2
        - L2**2
        - L34**2
    ) / (2 * L2 * L34)

    cos_q3 = np.clip(cos_q3, -1.0, 1.0) # limitamos valor
    q3_sol = np.arccos(cos_q3) #q3 a partir del coseno calculado


    q2_sol = (
    np.arctan2(z_prima, r) # angulo horizontal hasta punto objetivo
    -
    np.arctan2( #angulo generado por segunda parte del brazo
        L34 * np.sin(q3_sol),
        L2 + L34 * np.cos(q3_sol)
    )
    )

    q4_sol = 0.0 #se fija cause we need 4 ecuaciones

    return np.array([ #devuelven los cuatros angulos (en rads)
        q1_sol,
        q2_sol,
        q3_sol,
        q4_sol
    ])
