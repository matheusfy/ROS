import numpy as np
v1 = np.ones((3,1))
v2 = np.zeros((3,1))
v_stack = np.vstack((v1,v2))
print(v_stack.shape[1])