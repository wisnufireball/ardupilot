using X plane:
	cd to ArduPlane/RBOTIX_RAVEN_C

	$ sim_vehicle.py -j4 -f xplane

using JSBSIM

	//cd to ArduPlane/RBOTIX_RAVEN_C
	$ cd cygdrive/e/ardupilot/ArduPlane/RBOTIX_RAVEN_C/
	
	//run E:\ardupilot\Tools\autotest\fg_plane_view.bat on windows

	//run simulation
	$ sim_vehicle.py -j4 -L sinabung

	//check output port
	MANUAL> output

	//connect mission planner to sitl via udp, use port from output

	setup wind
	param set SIM_WIND_DIR 180
	param set SIM_WIND_SPD 10

	to quit:
	disconnect mp
	close flight gear
	close mavproxy



