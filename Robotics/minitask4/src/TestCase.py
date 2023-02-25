import unittest
from gridmap import *

class TestCase(unittest.TestCase):
	
	def test_to_grid(self):
		gridMap = GridMap([0,8], [0,8], 1)

		# Equal Test
		result = gridMap.to_grid(5, 5, 20,20,0, 0, 1)
		self.assertEqual(result, (5,5))
		result = gridMap.to_grid(5, 5, 10,10,0, 0, 0.5)
		self.assertEqual(result, (9,9))
		result = gridMap.to_grid(0, 0, 5, 5, -5, -5, 1)
		self.assertEqual(result, (4,4))
		result = gridMap.to_grid(3, 0, 10, 10, -8, -2, 1)
		self.assertEqual(result, (9,2)) 
		result = gridMap.to_grid(9, -2, 20, 20, 5, -3, 0.1)
		self.assertEqual(result, (19,10)) 
		
		# Not Equal Test
		result = gridMap.to_grid(3, 3, 20, 20, 0, 0, 1)
		self.assertNotEqual(result, (4,5)) # Should be 3,3
		result = gridMap.to_grid(4, -7, 10, 10, 1, -7, 0.2)
		self.assertNotEqual(result, (10,1)) # Should be 10,0


	def test_to_world(self):
		gridMap = GridMap([0,8], [0,8], 1)

		# Equal Test
		result = gridMap.to_world(9, 9, [0,10], [0,10],-5, -5, 1)
		self.assertEqual(result, (4,4))
	
		result = gridMap.to_world(2, 10, [0,20], [0,20],4, 2, 0.5)
		self.assertEqual(result, (5,7)) 
		result = gridMap.to_world(7, 2, [0,10], [0,10],-2, 10, 2)
		self.assertEqual(result, (10,10)) 
		result = gridMap.to_world(4, 1, [0,20], [0,20],-1, -4, 1)
		self.assertEqual(result, (3,0))  

		# Not Equal Test
		result = gridMap.to_world(9, 9,[0,20], [0,20], -5, -5, 1)
		self.assertNotEqual(result, (4.5,4.5)) # Should be 4,4
		result = gridMap.to_world(2, 15,[0,10], [0,10], -2, -3, 0.5)
		self.assertNotEqual(result, (-2,4.5)) # Should be 0, 4.5

		result = gridMap.to_world(12, 15, [0,20], [0,20], 1, -2, 0.2)
		self.assertEqual(result, (3.40000,1.0))  # Assertion not able to show decimal correctly
	
if __name__ == '__main__':
    unittest.main()
