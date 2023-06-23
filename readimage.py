from PIL import Image,ImageOps
import numpy as np
import matplotlib.pyplot as plt

#img = Image.open('cspace.png')
#img = ImageOps.grayscale(img)
#np_img = np.array(img)
#np_img = ~np_img
#np_img[np_img > 0] = 1
#plt.set_cmap('binary')
#plt.imshow(np_img)

#np.save('scpace.npy', np_img)

grid = np.load('/home/anhbanhieu/catkin_ws/scpace.npy')
plt.imshow(grid)
plt.tight_layout()
plt.show()

