import matplotlib.pyplot as plt
import numpy as np
 
vectors = np.array(([2, 0], [3, 2]))
vector_addition = vectors[0] + vectors[1]
vectors = np.append(vectors, vector_addition[None,:], axis=0)
 
tail = [0, 0]
fig, ax = plt.subplots(1)
ax.quiver(*tail,
           vectors[:, 0],
           vectors[:, 1],
           scale=1,
           scale_units='xy',
           angles = 'xy',
           color=['g', 'r', 'k'])
 
ax.set_xlim((-1, vectors[:,0].max()+1))
ax.set_ylim((-1, vectors[:,1].max()+1))
plt.show()