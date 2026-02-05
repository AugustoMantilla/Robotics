# Forward Kinematics

## Introduction
Robotics kinematics studies robot motion without considering the forces involved, focusing on the position and orientation of its links. Forward kinematics computes the end-effector position and orientation from the joint variables. The robot is modeled as a kinematic chain of links and joints, using reference frames to describe spatial relationships. The Denavit–Hartenberg convention standardizes this representation using four parameters, simplifying the kinematic analysis of robotic manipulators.

## Exercices

### Exercise 1
![Diagrama del sistema](../recursos/imgs/Exercise1_KinematicsOriginal.jpg)
![Diagrama del sistema](../recursos/imgs/Exercise1_Kinematics_Valeria.jpg)

### Exercise 2
![Diagrama del sistema](../recursos/imgs/Exercise2_Kinematics.jpg)
![Diagrama del sistema](../recursos/imgs/Ex2HH.jpg)
![Diagrama del sistema](../recursos/imgs/tablahh.jpg)
![Diagrama del sistema](../recursos/imgs/Matriz_HOS.jpg)
![Diagrama del sistema](../recursos/imgs/MatrizFinal_HOS.jpg)

### Exercise 3
![Diagrama del sistema](../recursos/imgs/Exercise3_Kinematics.jpg)
![Diagrama del sistema](../recursos/imgs/Ex3_Table_FBD.jpg)
![Diagrama del sistema](../recursos/imgs/Matriz1_AMM.jpg)
![Diagrama del sistema](../recursos/imgs/Matriz2_AMM.jpg)
![Diagrama del sistema](../recursos/imgs/MatrizFinal_AMM.jpg)


### Exercise 4
![Diagrama del sistema](../recursos/imgs/Exercise4_Kinematics.jpg)
![Diagrama del sistema](../recursos/imgs/Brianproblem.jpg)
![Diagrama del sistema](../recursos/imgs/Ex4_Table_Matriz.jpg)
![Diagrama del sistema](../recursos/imgs/Ex4_Matriz2.jpg)
![Diagrama del sistema](../recursos/imgs/Ex4_Matriz3.jpg)

### Final Transformation Matrix

```
T06 = [
 r11 r12 r13 px
 r21 r22 r23 py
 r31 r32 r33 pz
 0   0   0   1
]
```

---

### Rotation Terms

```
r11 = c1*c23*c4*s5 + c1*s23*c5 - s1*s4*s5

r12 = c6*(c1*c23*s4 + s1*c4)
      - s6*(c1*c23*c4*c5 - c1*s23*s5 - s1*s4*c5)

r13 = s6*(c1*c23*s4 + s1*c4)
      + c6*(c1*c23*c4*c5 - c1*s23*s5 - s1*s4*c5)

r21 = s1*c23*c4*s5 + s1*s23*c5 + c1*s4*s5

r22 = c6*(s1*c23*s4 - c1*c4)
      - s6*(s1*c23*c4*c5 - s1*s23*s5 + c1*s4*c5)

r23 = s6*(s1*c23*s4 - c1*c4)
      + c6*(s1*c23*c4*c5 - s1*s23*s5 + c1*s4*c5)

r31 = s23*c4*s5 - c23*c5

r32 = c6*s23*s4
      - s6*(s23*c4*c5 + c23*s5)

r33 = s6*s23*s4
      + c6*(s23*c4*c5 + c23*s5)
```

---

### Position Terms

```
px = c1*(8*c2 + 8*c23 - d4*s23) - d3*s1

py = s1*(8*c2 + 8*c23 - d4*s23) + d3*c1

pz = 13 + 8*s2 + 8*s23 + d4*c23 + d6
```

---

### Exercise 5
![Diagrama del sistema](../recursos/imgs/Exercise5_Kinematics.jpg)
![Diagrama del sistema](../recursos/imgs/Ex5_FBD.jpg)
![Diagrama del sistema](../recursos/imgs/Ex5_table.jpg)
![Diagrama del sistema](../recursos/imgs/Ex5_M1.jpg)
![Diagrama del sistema](../recursos/imgs/Ex5_M2.jpg)
![Diagrama del sistema](../recursos/imgs/Ex5_M3.jpg)
![Diagrama del sistema](../recursos/imgs/Ex5_M4.jpg)