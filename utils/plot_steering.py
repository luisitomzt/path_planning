import pickle
import numpy as np
import matplotlib.pyplot as plt


if __name__ == '__main__':
	with open("steering_radius.txt", "rb") as fp:
		data = pickle.load(fp)
		plt.plot(data)

		print np.mean(data)
		print np.std(data)



plt.ylabel('Velocities')
plt.ylim(2,4)
plt.xlabel('Samples')
plt.show()