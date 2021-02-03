

The crcl Yaml subbranch defines various ROS parameters used by the CRCL operation. Information on CRCL can be found here: https://www.nist.gov/el/intelligent-systems-division-73500/canonical-robot-command-language-crcl 

	crcl:
		# default length units of measure (METER|MM)
		length_units: METER
		
		# default angle units of measure (RADIAN |DEGREE)
		angle_units: RADIAN
		
		# ignore stop – temporary kludge
		StopIgnore: 1
		
		# the IP of the host CRCL server
		Ip: 127.0.0.1
		
		# The port the host CRCL server listens to
		Port: 64444
		
		# If auto status, time between status publishes
		PublishStatusPeriod: 0.05
		
		# auto status (1) automatically reports status
		AutoStatus: 1
		
		# flywheel handles all CRCL commands until done
		flywhee: 0
		
		# ????
		processAllCrclMessages: 0

	# debug flags used by CRCL streaming 
	Debug: 
		
		# general diagnostics
		General: 1
		
		# echo status message reporting
		StatusMsg: 0
		
		# echo command message receipt
		CommandMsg: 0
		
		# echo XMl CRCL socket characters
		XML: 0









