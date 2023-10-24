# Aus optimize.py
Link: https://github.com/pumablattlaus/redundancy_res_collision/blob/main/optimization_algo/scripts/optimize.py
## UR_Vel induced by Mir
Berechne j_p (Zeile 590)
```python
r0E = getVector_eef(q) + self.ur.ur_base_pos[i]
j_p = getJacobianPlatformWithEEF(r0E.x, r0E.y) # zu MiRbase mit x',w_z
```


```python
v_ur_nullspace = j_p @ vel_mir # in MiRbase Coordinates
ur_vel = inv_rotM @ self.v_command - v_ur_nullspace # in MiRbase Coordinates (v_command is in worldCoordinates)
```


```python
def getJacobianPlatformWithEEF(rx, ry):
    """get Jacobian of platform with coordinates x,y,theta. Transforms x_p', y_p', theta_p' to cartesian coordinates of EE
    
    Args:
        rx (float): x coordinate of EE relativ to mur_base_link
        ry (float): y coordinate of EE relativ to mur_base_link

    Returns:
        Jacobian (np.array((6,3)))
    """
    J = np.zeros((6, 3))
    J[0,0]=1
    J[1,1]=1
    J[0,2]=-ry
    J[1,2]=rx
    J[5,2]=1
    return J
```