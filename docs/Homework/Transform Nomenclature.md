## Exercise 1

A vector **P** is rotated about the axis **Ya** by an angle of 45°, and then rotated about the axis **Xa** by an angle of 60°. Give the rotation matrix that performs these rotations in the indicated order.

Solving for the rotations of the Ya axis we will have the following matrix:

<img src="../recursos/imgs/RotationYA_Homework_TransformNomenclature.jpeg" alt="Matrix Ya" width="420">

And for the Xa axis rotation we got this matrix:


![Diagrama del sistema](../recursos/imgs/RotationXA_Homework_TransformNomenclature.jpeg)


Since the rotation around the Ya​ axis occurs first, the total rotation matrix is given by:
![Diagrama del sistema](../recursos/imgs/MultiplicationRXYaRXa.jpeg)

Carrying out the matrix multiplication, we obtain:
![Diagrama del sistema](../recursos/imgs/MultiplicationMatrix.jpeg)


## Exercise 2

Reference frame B is rotated with respect to reference frame A by an angle of 30° about the axis Xa. The translation of frame B with respect to frame A is given by [5 10 0] formulate the homogeneous transformation matrix

The rotation of frame B with respect to frame A about the XA axis is expressed as:

![Diagrama del sistema](../recursos/imgs/RotationXA_Homework_TransformNomenclatureEx2.jpeg)

The translation of frame B relative to frame A is given by:



![Diagrama del sistema](../recursos/imgs/pab.jpeg)

The homogeneous transformation matrix TAB is constructed as:
C

Substituting the corresponding terms:
![Diagrama del sistema](../recursos/imgs/tabgrande.jpeg)

## Exercise 3
![Diagrama del sistema](../recursos/imgs/imageEx3.jpeg)

- From the given image, obtain:
    * The value of ABT
    * The value of ACT

From the diagram, the origin of frame B is displaced from the origin of frame A by a distance of 3 units along the Xa axis. Therefore, the translation vector is:


![Diagrama del sistema](../recursos/imgs/pabEx3.jpeg)

Looking at the image the orientation of the axes corresponds to a rotation of 180◦ about the Za axis:


![Diagrama del sistema](../recursos/imgs/rza.jpeg)

So, the homogeneous transformation matrix Tab is this:


![Diagrama del sistema](../recursos/imgs/tabcompleta.jpeg)

The origin of frame C is obtained by moving 3 units along the XA axis and 2 units along the ZA axis:


![Diagrama del sistema](../recursos/imgs/pac.jpeg)

From the diagram, frame C is rotated by 30◦ about the ZA axis with respect to frame A. The rotation matrix is given by:


![Diagrama del sistema](../recursos/imgs/rac.jpeg)

So, the homogeneous transformation matrix Tab is this:
![Diagrama del sistema](../recursos/imgs/tac.jpeg)