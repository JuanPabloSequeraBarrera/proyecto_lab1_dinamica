"""Fase 1 del preinforme: crear y visualizar trayectorias planas.

En esta fase T es una matriz de N puntos. Cada fila contiene [x, y, z]
en metros. Como el laser dibuja sobre un plano, z permanece constante.

Esta NO es todavia la simulacion completa del robot: faltan la cinematica
directa, el Jacobiano, la cinematica inversa, Open3D y las colisiones.
"""

import numpy as np
import matplotlib.pyplot as plt


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

#Cinematica Directa:
#Empezamos con las variables:
q1, q2, q3, q4 = me.dynamicsymbols('q1 q2 q3 q4')
L1, L2, L3, L4 = 0.115, 0.13, 0.13, 0.03412

#Refrence frames
N = me.ReferenceFrame('N')
A = N.orientnew('A', 'Axis', (q1, N.z))
B = A.orientnew('B', 'Axis', (q2, A.y))
C = B.orientnew('C', 'Axis', (q3, B.y))
D = C.orientnew('D', 'Axis', (q4, C.y))

#Origen para que cada punto se pueda describir based on other points
O = me.Point('O')

#Describimos nuestros puntos:
J1 = O
J2 = J1.locatenew('J2', L1*N.z)
J3 = J2.locatenew('J3', L2*B.x)
J4 = J3.locatenew('J4',L3 * C.x)

Laser = J4.locatenew('Laser',L4 * D.x)

#laser con respecto a origen
r_laser = Laser.pos_from(O).express(N)
r_laser_vectors = [sp.simplify(r_laser.dot(N.x)), sp.simplify(r_laser.dot(N.y)), sp.simplify(r_laser.dot(N.z))]

fk = sp.lambdify(
    (q1, q2, q3, q4),
    (r_laser_vectors[0], r_laser_vectors[1], r_laser_vectors[2]),
)

def cinematica_directa(q1_val, q2_val, q3_val, q4_val):
    x, y, z = fk(
        q1_val,
        q2_val,
        q3_val,
        q4_val
    )

    return np.array([x, y, z], dtype=float)
