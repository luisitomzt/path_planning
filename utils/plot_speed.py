import pickle
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
	with open("velocities.txt", "rb") as fp:
		data = pickle.load(fp)
		speed = 50
	for d in data:
		#print d
		print speed, " mean ",np.mean(d),"std ",np.std(d)
		speed+=50
		plt.plot(d)

plt.ylabel('Velocities')
plt.xlabel('Samples')
plt.show()

