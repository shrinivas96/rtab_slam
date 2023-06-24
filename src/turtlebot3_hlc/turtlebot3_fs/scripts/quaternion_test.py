import numpy as np
from tf.transformations import quaternion_from_euler
  
if __name__ == '__main__':
    for i in np.linspace(0, 3*np.pi, 30):
        q = quaternion_from_euler(0, 0, i)
        print("The quaternion representation for %s is \t\t %s %s %s %s." % (i*180/np.pi, q[0], q[1], q[2], q[3]))