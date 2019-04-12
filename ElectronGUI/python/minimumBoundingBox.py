def minimum_bounding_box(xcoords, ycoords):

  min_x = 100000 # start with something much higher than expected min
  min_y = 100000
  max_x = -100000 # start with something much lower than expected max
  max_y = -100000

  for item in xcoords:
    if item < min_x:
      min_x = item

    if item > max_x:
      max_x = item

  for item in ycoords:
    if item < min_y:
      min_y = item

    if item > max_y:
      max_y = item

  return [min_x, max_x, min_y, max_y]
