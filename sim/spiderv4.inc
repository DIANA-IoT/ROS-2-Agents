# spider.inc
# model for the robot "Spider" from Hexbug

# spider's sonar OLD model....
#define spider_sonar ranger
#(
#	scount 8 # the number of transducers
#
#
#	# define the pose of each transducer [xpos ypos heading]
#	spose[0] [ 0.02 0.04 60]
#	spose[1] [ 0.02 0.04 45]
#	spose[2] [ 0.025 0.02 30]
#	spose[3] [ 0.025 0.02 15 ]
#	spose[4] [ 0.04 0 0 ] 
#	spose[5] [ 0.025 -0.02 -15 ]
#	spose[6] [ 0.025 -0.02 -30]
#	spose[7] [ 0.02 -0.04 -45]
#	# spose[8] [ 0.02 -0.04 -60]
#		
#	# define the field of view of each transducer [range_min range_max view_angle]
#	sview [0 0.25 5]
#
#	# define the size of each transducer [xsize ysize] in meters
#	ssize [0.02 0.02]
#)

# ranger model from Joaquin's configuration files...
define topurg ranger
(
  sensor( 		
	pose [ 0 0 0 0 ]
    size [ 0.07 0.07 0.05 ]	
    range [ 0.0  3.0]
    fov 360
    samples 360
  )

  model
  (
    # generic model properties
    pose [ 0.0 0.0 -0.2 0.0 ]
    size [ 0.07 0.07 0.05 ]  
    color "blue"
  )
)

# spider's body
define spider position
(
	# actual size
	size [0.2 0.2 0.20]


	# body
	block
	(
		points 5
		point[0] [ 1 -2 ]
		point[1] [ 2 0 ]
		point[2] [1 2 ]
		point[3] [ -2 2 ]
		point[4] [ -2 -2 ]
		z [ 0 0.5 ]
	)

	# positional things
	mass 0.5 		# In Kg
	drive "diff" 		# Differential steering model
	gui_nose 1 		# For knowing which way it's facing
  	ranger_return 1       # reflects sonar beams

	sonar_return 1
	laser_return 1
	obstacle_return 1	# can hit things
  	localization "gps"    	
  	
    #localization_origin [0 0 0 0] 	# Start odometry at (0, 0, 0) JMCG: why is this commented out?

	# sensors attached to spider
	#spider_sonar( pose [0 0 0 0] )
	topurg(pose [ 0.15 0 0 0 ]) #Instead of the sonar, now we have this for future use.
	color "green"
)

