from random import randint, random


if __name__ == '__main__':

	with open('poses.txt', 'w') as f:

		for i in range(10):
			x = randint(0, 3)
			y = randint(0, 3)
			theta = 3.14*random()

			s = '{}\t{}\t{}\n'.format(x, y, theta)

			f.write(s)