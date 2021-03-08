# Trabajo previo P3

### Que funciones/estrategias vais a aplicar para decidir la nueva v y w en función de los parámetros medidos en cada nueva imagen/frame? Cuales serán dichos parámetros?

- Al principio estrategia 1 (sin localizacion explicita), solo centrar el objetivo en el centro:
  - Calcular d (distancia centro vista -> centro objetivo)
    w = fw(d)
    (por ej, interpolacion lineal entre -wmax, +wmax en f. de d)

  - Calcular area A (area del blob):
    v = fv(A-a)
    (Donde a es el area objetivo, obtenida a partir de un par de fotos en la pos objetivo)


### Preparar pseudocódigo/esquemaen pythonpara estos dos métodos de la clase robot:

TODO
