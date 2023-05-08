import yaml
import numpy as np
import cv2
import pdb

class Map(object):

	def __init__(self, map_file):

		#Loading map deets
		print("Loading map from " + map_file )
		with open(map_file + '.yaml', 'r') as stream:
			try:
				map_data = yaml.load(stream)
			except yaml.YAMLError as exc:
				print(exc)

		self.res = map_data['resolution']
		self.origin = np.array(map_data['origin'])
		print("Map details loaded!")

		#Loading map image
		img = cv2.imread(map_file + '.pgm', cv2.IMREAD_GRAYSCALE)

		## changing values of the img file to create a map file
		self.map = np.ones_like(img)
		self.map[img == 0] = 0       # occupied cells
		self.map[img == 254 ] = 1
		self.map[img == 255] = 1     # free cells
		self.map[img == 205] = 2     # unknown cells

		print("Map image loaded!")

	# functiion to convert the pixel coordinate to world coordinate using resolution and origin
	def pixel_to_world(self,x_pix, y_pix):

		converted_x_pix = y_pix
		converted_y_pix = self.map.shape[0] - 1 - x_pix
		x_world = converted_x_pix * self.res + self.origin[0]
		y_world = converted_y_pix * self.res + self.origin[1]
		return x_world, y_world

	## function to convert the world coordinate to pixel coordinate using resolution and origin
	def world_to_pixel(self, x_world, y_world):
		
		x_pix = int((x_world - self.origin[0]) / self.res)
		y_pix = int((y_world - self.origin[1]) / self.res)
		return x_pix, y_pix


	def is_collided(self, xi):
		x_world = xi[0]
		y_world = xi[1]
		x_pix, y_pix = self.world_to_pixel(x_world, y_world)
		# pdb.set_trace()
		if self.map[ self.map.shape[0] - 1 -y_pix ,x_pix] == 0: # note : pixel indexing assumes (y,x) in our case
			# print("Collision detected!")
			# print("x_world: ", x_world, "y_world: ", y_world)
			# print("final pix : ", self.map.shape[0] - 1 -y_pix ,"  ,  ",x_pix)
			# print("x pix ", x_pix, " , y pix ", y_pix)
			return True
		else:
			# print("No collision!")
			# print("x_world: ", x_world, "y_world: ", y_world)
			# print("final pix : ", self.map.shape[0] - 1 -y_pix ,"  ,  ",x_pix)
			return False
