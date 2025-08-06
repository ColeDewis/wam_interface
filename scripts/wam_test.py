#!/usr/bin/env python3
import numpy as np
import rospy
from wam_interface.wam import WAM

if __name__ == "__main__":
    rospy.init_node("wam_test_node", anonymous=False)
    rospy.loginfo("Initalizing WAM")
    wam = WAM()
    # home angles: 
    # [0.0012783173232380342, -2.0322394678728495, 0.03128990575628289, 3.068046796926165, 0.019135224261253875, -0.08444801450834355, -0.1943932786791449]
    
    # reach angles: 
    reach = np.array([0.008692557798018632, 0.3442632989449545, -0.03439152906740131, 2.350058567040802, 0.08255030631714483, 0.2555580364147625, -0.25552647149843205])
    
    # pose at reach: 
    # pose: 
    #   position: 
    #     x: 0.43642363949380314
    #     y: -0.005884460846148774
    #     z: 0.054414529444438
    #   orientation: 
    #     x: -0.07710936736060474
    #     y: 0.9923763225047953
    #     z: -0.005058206140148401
    #     w: -0.09600934614791198
    
    # joint lims: 
    # J1 [2.65, -2.65]
    # J2 [-2.02, 2.02]
    # J3 [-2.72, 2.78]
    # J4 [3.08, -0.80]
    # J5 [-4.74, 1.23]
    # J6 [-1.525, 1.48]
    # J7 [-3.026, 2.93]
    
    zero = np.zeros(7)
    # # wam.send_joint_velocities(zero)

    # for i in range(10):
    #     # small = np.array([0, 0, 0, 0, -0.0, 0.1, 0.0])
    #     small = np.array([0.07, 0.07, 0.0, 0.0, 0.0, 0.0])
    #     # wam.send_joint_velocities(small)
    #     wam.send_twist_topic(small)
    #     rospy.sleep(0.2)
    #     rospy.loginfo(f"{i} - {wam.joints}")
        
    # wam.send_joint_velocities(zero)
    # wam.send_pose(0.5, 0.0, 0.0, [0.0, 1.0, 0.0, 0.0])
    wam.go_home()
    # wam.send_joint_angles(np.zeros(7))
    # wam.send_joint_angles(reach)
    
    rospy.sleep(0.5)

    rospy.loginfo(f"Done. Joint angles: {wam.joints}")
