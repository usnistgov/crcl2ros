## <a name="Q:_How_do_I_verify_the_URDF_is_valid?"></a>Q: How do I verify the URDF is valid?


A: Make sure the command "checkurdf" is installed. 


	>~/ros/moveittest/src/fanuc_lrmate200id_support/urdf$ checkurdf FanucLRMate200iD.urdf  
	No command 'checkurdf' found, did you mean: 
	 Command 'check_urdf' from package 'liburdfdom-tools' (universe) 
	checkurdf: command not found 
	>~/ros/moveittest/src/fanuc_lrmate200id_support/urdf$ rosrun urdfdom check_urdf FanucLRMate200iD.urdf  
	[rospack] Error: package 'urdfdom' not found 
	 
	sudo apt-get install liburdfdom-tools  

Then you use " check_urdf" to verify the urdf is valid, or roslaunch will barf. Below is a working fanuc urdf:



	> check_urdf lrmate200id.urdf 
	robot name is: fanuc_lrmate200id 
	---------- Successfully Parsed XML --------------- root Link: world has 1 child(ren) 
	     child(1):  fanuc_base_link 
	         child(1):  fanuc_link_1 
	             child(1):  fanuc_link_2 
	                 child(1):  fanuc_link_3 
	                     child(1):  fanuc_link_4 
	                         child(1):  fanuc_link_5 
	                             child(1):  fanuc_link_6 
	                                 child(1):  fanuc_finger_1 
	                                 child(2):  fanuc_finger_2 







