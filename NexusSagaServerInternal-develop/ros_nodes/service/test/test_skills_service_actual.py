import rospy
import time

while True:
    print("Reset...")
    rospy.set_param("/skill_name_input", f"{str(time.time())},None,None")
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")
    rospy.set_param("/pick_lock_on", False)
    rospy.set_param("/is_replanning", f"{str(time.time())},False")

    # Give instruction
    rospy.set_param("/llm_planner", f"{str(time.time())},None")
    instruction = "None"
    while instruction == "None":
        raw_instruction = rospy.get_param(
            "/llm_planner", f"{str(time.time())},None"
        )
        raw_instruction_list = raw_instruction.split(",")

        if len(raw_instruction_list) < 2:
            print(
                f"ERROR on receiving instruction via ros param. Acceptable format `timestamp,instruction_string. Received - {raw_instruction}"
            )
            instruction = "None"
        elif len(raw_instruction) == 2:
            instruction = raw_instruction_list[1]
        else:
            instruction = ",".join(raw_instruction_list[1:])

    ######################
    ##### Navigation #####
    ######################
    print("navigation_highlight...")
    rospy.set_param("/is_replanning", f"{str(time.time())},False")
    rospy.set_param("/skill_name_input", f"{str(time.time())},nav,sink")
    rospy.set_param("nav_target_xyz", "-0.8,0.5,2.9|")
    time.sleep(20)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},nav,True,Execution suc!!")
    time.sleep(0.5)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")

    ######################
    ##### Open/Close #####
    ######################
    # print("open skills...")
    # rospy.set_param("/skill_name_input", f"{str(time.time())},Open,drawer")
    # rospy.set_param("multi_class_object_target", "drawer handle")
    # breakpoint()
    # rospy.set_param("/pick_lock_on", True)
    # breakpoint()
    # time.sleep(20)
    # rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},open,True,Execution suc!!")
    # time.sleep(0.5)
    # rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")

    ################
    ##### Pick #####
    ################
    print("target_object_detection...")
    rospy.set_param("/is_replanning", f"{str(time.time())},True")
    time.sleep(10)
    rospy.set_param("/skill_name_input", f"{str(time.time())},pick,cup")
    time.sleep(20)
    print("found_object...")
    rospy.set_param("/pick_lock_on", True)
    time.sleep(20)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},pick,True,Execution suc!!")
    time.sleep(0.5)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")

    ######################
    ##### Navigation #####
    ######################
    print("navigation_highlight...")
    rospy.set_param("/skill_name_input", f"{str(time.time())},nav,dining table")
    rospy.set_param("nav_target_xyz", "1.3,0.3,1.5|")
    time.sleep(20)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},nav,True,Execution suc!!")
    time.sleep(0.5)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")

    #################
    ##### Place #####
    #################
    print("location_detection...")
    rospy.set_param("/skill_name_input", f"{str(time.time())},place,cup")
    rospy.set_param("place_target_xyz", "1.0,0.0,0.0|")
    rospy.set_param("robot_target_ee_rpy", "0.0,0.0,0.0")
    time.sleep(20)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},place,True,Execution suc!!")
    time.sleep(0.5)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")

    ######################
    ##### Navigation #####
    ######################
    print("navigation_highlight...")
    rospy.set_param("/skill_name_input", f"{str(time.time())},nav,dining table")
    rospy.set_param("nav_target_xyz", "1.3,0.3,1.5|")
    time.sleep(20)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},nav,True,Execution suc!!")
    time.sleep(0.5)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")

    ################
    ##### Pick #####
    ################
    print("target_object_detection...")
    rospy.set_param("/is_replanning", f"{str(time.time())},False")
    time.sleep(10)
    rospy.set_param("/skill_name_input", f"{str(time.time())},pick,cup")
    time.sleep(20)

    print("found_object...")
    rospy.set_param("/pick_lock_on", True)
    time.sleep(20)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},pick,True,Execution suc!!")
    time.sleep(0.5)
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")
    
    print("Done, reset...")
    rospy.set_param("/skill_name_input", f"{str(time.time())},None,None")
    rospy.set_param("/skill_name_suc_msg", f"{str(time.time())},None,None,None")
    rospy.set_param("/pick_lock_on", False)
    rospy.set_param("/llm_planner", f"{str(time.time())},None")