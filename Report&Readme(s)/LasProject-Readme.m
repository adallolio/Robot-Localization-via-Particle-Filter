%%LAS PROJECT - ALBERTO DALLOLIO

%% README FILE

%% Folder Contents:
% 
% - Robot.m  			  contains all the methods and properties of the 											  Robot class object (plotting, odometry, etc)
% - Map.m 			  defines the map environment features 									  						(dimension,landmarks) and the methods for plotting					    					  the map
% - Path.m 			  defines the driver: the driver is responsible for 										  driving the Robot along a path to be specified. In 										  Path the points are randomly (or not) chosen and 											  the controls for the robot are properly generated
% - BearingSensor.m 	  creates the bearing sensor with its features. 											  Computes the bearings to landmarks and their 						 						  locations
% - tb_optparse.m 	  is a matlab toolbox options parser
% - transl.m 			  creates a translational matrix
% - rt2tr.m 			  converts rotation+transl into a homog. transform
% - hom_tr_z.m 		  computes a homogeneous transformation matrix from a 					  						rotation around z-axis
% - getPiAngle.m 		  transforms angles in [-pi,pi]
% - draw_poly.m 		  draws polygons to represent the differential drive robot
% - compute_numsteps.m  computes the timesteps necessary to run the simulation 
% 					  for plotting purpose
% - compute_class.m 	  function required by the sensor to compute the landmark color
% - angdiff.m 		  computes the difference between 2 angles
