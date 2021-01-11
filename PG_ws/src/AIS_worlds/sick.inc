define sicksensor sensor
(
# factory settings for LMS200

   	pose [0.09 0.0 0 0]
	size [ 0.156 0.155 0.19 ]
    	color "blue"  
   	samples 540			#number of laser samples per scan
    	range_max 30.0		#max range, scanner will not detect objects beyond this range (m)
    	fov 270			#field of view of the scanner (raidans)
  	resolution 1		#only report every nth range and interpolate the ones inbetween
)
define sickcommon ranger
(
color "yellow"
size [ 0.156 0.155 0.19 ] # dimensions from LMS200 data sheet
)
define sickbase sickcommon
(
block( points 4
point[0] [0 0]
point[1] [0 1]
point[2] [1 1]
point[3] [1 0]
z [0 0.21]
)
)
# extends sicklaser to add nice-looking but relatively expensive geometry
define fancysickbase sickcommon
(
	# bottom
	block(
		points 4
		point[0] [ -0.02 -0.077 ]
		point[1] [ 0.078 -0.077 ]
		point[2] [ 0.078 0.077 ]
		point[3] [ -0.02 0.077 ]
		z [0 0.151 ]
	)
	# laser bit
	block( 
		points 10
		point[0] [ -0.02 -0.05 ]
		point[1] [ 0.03 -0.05 ]
		point[2] [ 0.045 -0.045 ]
		point[3] [ 0.055 -0.035 ]
		point[4] [ 0.06 -0.02 ]
		point[5] [ 0.06 0.02 ]
		point[6] [ 0.055 0.035 ]
		point[7] [ 0.045 0.045 ]
		point[8] [ 0.03 0.05 ]
		point[9] [ -0.02 0.05 ]
		z [0.15 0.21 ]
	)
)
define sicklaser sickbase ( sicksensor() )
define fancysicklaser fancysickbase ( sicksensor() )
