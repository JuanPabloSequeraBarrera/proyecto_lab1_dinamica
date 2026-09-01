

def mover_desde_xy(mc, x, y, l1, l2, velocidad=30):

    angulos = cinematica_inversa(x, y, l1, l2)

    print("Ángulos calculados:", angulos)

    mover_angulos_y_verificar(
        mc,
        angulos,
        velocidad
    )

    return angulos
