# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.mpc.Debug:
/Users/student/Udacity/sdcn2/L19\ P5\ Model\ Predictive\ Control\ Project/MyProject/CarND-MPC-Project-P10/xbuild/Debug/mpc:
	/bin/rm -f /Users/student/Udacity/sdcn2/L19\ P5\ Model\ Predictive\ Control\ Project/MyProject/CarND-MPC-Project-P10/xbuild/Debug/mpc


PostBuild.mpc.Release:
/Users/student/Udacity/sdcn2/L19\ P5\ Model\ Predictive\ Control\ Project/MyProject/CarND-MPC-Project-P10/xbuild/Release/mpc:
	/bin/rm -f /Users/student/Udacity/sdcn2/L19\ P5\ Model\ Predictive\ Control\ Project/MyProject/CarND-MPC-Project-P10/xbuild/Release/mpc


PostBuild.mpc.MinSizeRel:
/Users/student/Udacity/sdcn2/L19\ P5\ Model\ Predictive\ Control\ Project/MyProject/CarND-MPC-Project-P10/xbuild/MinSizeRel/mpc:
	/bin/rm -f /Users/student/Udacity/sdcn2/L19\ P5\ Model\ Predictive\ Control\ Project/MyProject/CarND-MPC-Project-P10/xbuild/MinSizeRel/mpc


PostBuild.mpc.RelWithDebInfo:
/Users/student/Udacity/sdcn2/L19\ P5\ Model\ Predictive\ Control\ Project/MyProject/CarND-MPC-Project-P10/xbuild/RelWithDebInfo/mpc:
	/bin/rm -f /Users/student/Udacity/sdcn2/L19\ P5\ Model\ Predictive\ Control\ Project/MyProject/CarND-MPC-Project-P10/xbuild/RelWithDebInfo/mpc




# For each target create a dummy ruleso the target does not have to exist
