# Configuration file for roomba in Stage 3.2.2


define roomba position
(
# localization_origin [ 3.0 0.0 0 0]

 size [0.33 0.33 0.1]

  obstacle_return 1           	# Can hit things.
  laser_return 1                 # reflects laser beams
#  ranger_return 1             	# reflects sonar beams
#  blobfinder_return 1         	# Seen by blobfinders  



  # this block approximates the circular shape of a Roomba
  block
  ( 
    points 16
    point[0] [ 0.225 0.000 ]
    point[1] [ 0.208 0.086 ]
    point[2] [ 0.159 0.159 ]
    point[3] [ 0.086 0.208 ]
    point[4] [ 0.000 0.225 ]
    point[5] [ -0.086 0.208 ]
    point[6] [ -0.159 0.159 ]
    point[7] [ -0.208 0.086 ]
    point[8] [ -0.225 0.000 ]
    point[9] [ -0.208 -0.086 ]
    point[10] [ -0.159 -0.159 ]
    point[11] [ -0.086 -0.208 ]
    point[12] [ -0.000 -0.225 ]
    point[13] [ 0.086 -0.208 ]
    point[14] [ 0.159 -0.159 ]
    point[15] [ 0.208 -0.086 ]
    z [0.184 0.4]
  )
  # this bumper array VERY crudely approximates the Roomba's bumpers
#  bumper( bcount 2  
#	  blength 0.33
#          bpose[0] [0.12  0.12  45]
#          bpose[1] [0.12 -0.12 -45]
#        )

  color "gray50"
)


define create roomba( color "gray90" )



