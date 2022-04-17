import os
import cv2
import numpy as np
from matplotlib import pyplot as plt
import matplotlib.image as mpimg
from sklearn.cluster import KMeans

path = os.getcwd() + '/filtered_depth.jpg'
raw_image = cv2.imread(os.getcwd() + '/raw_photo.jpg')
img = mpimg.imread(path)
plt.imshow(img)
plt.show()
red = np.array(raw_image[:,:,0]).flatten()
green = np.array(raw_image[:,:,1]).flatten()
blue = np.array(raw_image[:,:,2]).flatten()

R = np.reshape(red, (len(red), 1))
G = np.reshape(green, (len(green), 1))
B = np.reshape(blue, (len(blue), 1))

N = np.array(img).flatten()
N = np.reshape(N, (len(N), 1))

N = np.hstack((N, R, G, B))
print(N.shape)
I = np.array(img)
nrows, ncols = I.shape[0], I.shape[1]
X2D, Y2D = np.meshgrid(np.arange(0, ncols, 1), np.arange(0, nrows, 1))
out = np.column_stack((Y2D.ravel(),X2D.ravel()))
fullStack = np.column_stack((out, N))

kmeans_model = KMeans(n_clusters=2, random_state=1).fit(N)
labels= kmeans_model.labels_

output = np.reshape(labels, (nrows, ncols))
output[output != output[int(nrows/2), int(ncols/2)]] = 0
output = cv2.medianBlur(output.astype(np.uint8), 3)
temp = out[labels == output[int(nrows/2), int(ncols/2)]]
plt.imshow(output)
plt.show()
print(temp.shape)
((center_x, center_y), (dim_x, dim_y), angle) = cv2.minAreaRect(np.column_stack((temp[:,1], temp[:,0])))
rect = ((center_x, center_y), (dim_x, dim_y), angle)
print(center_x, center_y)
box = np.int0(cv2.boxPoints(rect))
cv2.drawContours(output, [box], 0, (36,255,12), 3) # OR

print(box)
# cv2.imwrite("bounded.jpg", box)
plt.imshow(output)
plt.show()
# print(temp[:,0])
# plt.scatter(temp[:,1], temp[:,0])
# temp = out[labels==0]
# plt.scatter(temp[:,1], temp[:,0], c='r')
# temp = out[labels==2]
# plt.scatter(temp[:,1], temp[:,0], c='g')
# # plt.imshow()
# plt.show()


