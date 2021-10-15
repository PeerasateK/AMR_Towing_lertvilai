from scipy.spatial.transform import Rotation as R
import numpy as np

homo_cart_p4 = np.array([[0, 1, 0,-0.5],
                        [1, 0, 0,-0.4],
                        [0, 0, 1,0],
                        [0,0,0,1]])

a = [homo_cart_p4[:3][0][3],homo_cart_p4[:3][1][3]]
print(a)
