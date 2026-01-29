# Ejercicio 1 – Rotaciones en el Espacio

## Enunciado

Un vector **P_A** es rotado primero **45° alrededor del eje Y_A** y posteriormente
**60° alrededor del eje X_A**.  
Determinar la matriz de rotación que realiza estas rotaciones en el orden indicado.

---

## Consideraciones

Las rotaciones se aplican de derecha a izquierda en la multiplicación matricial.
La primera rotación aplicada al vector es la que se coloca más a la derecha.

---

## Matriz de rotación sobre el eje Y_A (45°)

R_Y(45°) =

[ cos45   0   sin45  
  0       1     0  
 -sin45   0   cos45 ]

R_Y(45°) =

[ √2/2   0   √2/2  
  0      1     0  
 -√2/2   0   √2/2 ]

---

## Matriz de rotación sobre el eje X_A (60°)

R_X(60°) =

[ 1   0        0  
  0  cos60  -sin60  
  0  sin60   cos60 ]

R_X(60°) =

[ 1    0        0  
  0   1/2   -√3/2  
  0  √3/2    1/2 ]

---

## Matriz de rotación total

La matriz de rotación resultante se obtiene como:

R = R_X(60°) · R_Y(45°)

---

## Resultado final

R =

[ √2/2      0        √2/2  
  √6/4     1/2     -√6/4  
 -√3/2    √3/2      1/2 ]
