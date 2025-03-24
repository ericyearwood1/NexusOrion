import rospy
import time

LAST_TIMESTAMP = -1



def print_and_set_param (param, val):
    curr_time = float(val.split(',')[0])

    global LAST_TIMESTAMP
    if LAST_TIMESTAMP== -1 or (curr_time - LAST_TIMESTAMP > 15) or (curr_time - LAST_TIMESTAMP < 0):
        LAST_TIMESTAMP = curr_time

    time.sleep(curr_time - LAST_TIMESTAMP)
    print(f" {param}  ->  {val}")
    rospy.set_param(param,val)
    LAST_TIMESTAMP = curr_time

def clear_params():
    print_and_set_param("/skill_name_input", "0,None,None")
    print_and_set_param("/skill_name_suc_msg","0,None,None,None")
    print_and_set_param("/llm_planner", "0,None")

def test():

    clear_params()
    param = "/skill_name_input"
    param2 = "/skill_name_suc_msg"
    param3 = "llm_planner"

    print_and_set_param(param3, "1723652929.824241,HelloWorld")
    val = "1723652930.824241,FindAgentActionTool[],None"
    print_and_set_param(param,val)
    val2 = "1723652930.824241,FindAgentActionTool[],True,The agent is walking"
    print_and_set_param(param2,val2)

    val = "1723652931.824241,FindObjectTool[],16_cup_on_table_or_counter"
    print_and_set_param(param,val)
    val2 = 	"1723652931.824241,FindObjectTool[],True,16_cup_on_table_pur_counter is 1/37 meters away from the agent in lab."
    print_and_set_param(param2,val2)

    val = "1723652932.824241,Navigate[],16_cup_on_table_pur_counte"
    print_and_set_param(param,val)
    val2 = "1723652939.824241,Navigate[],True,Successfully reached the target pose by default"
    print_and_set_param(param2,val2)

    val = "1723652940.824241,Pick[],cup"
    print_and_set_param(param,val)
    val2 = 	"1723652950.824241,Pick[],False,Node with name cup not present in the graph. Use the appropriate tool to get a valid name."
    print_and_set_param(param2,val2)

    val = "1723652951.824241,FindObjectTool[],cup"
    print_and_set_param(param,val)
    val2 = "1723652951.824241,FindObjectTool[],True,16 cup on table or counter is 1.07 meters away from the agent in lab. - 59 cup is in/on 81 blue table or cabinet and 1.22 meters away from the agent in lab. - 53 white cup is 2.90 meters away from the agent in"
    print_and_set_param(param2,val2)

    val = "1723652952.824241,Pick[],59_cup"
    print_and_set_param(param,val)
    val2 = "1723653000.824241,Pick[],True,Successfully picked the target object" 
    print_and_set_param(param2,val2)

    val = "1723653001.824241,FindReceptacleTool[],83_person_in_chair"
    print_and_set_param(param,val)
    val2 = "1723653001.824241,FindReceptacleTool[],True,83_person_in_char in lab"
    print_and_set_param(param2,val2)

    val = "1723653002.824241,Navigate[],83_person_in_chair"
    print_and_set_param(param,val)
    val2 = "1723653010.824241,Navigate[],True,Successfully reached the target pose by default"
    print_and_set_param(param2,val2)

    val = "1723653011.824241,Place[],83_person_in_chair"
    print_and_set_param(param,val)
    val2 = "1723653015.824241,Place[],True,Final Thought: Exit!"
    print_and_set_param(param2,val2)


    print_and_set_param(param3, "1723653016.824241,None")

    time.sleep(1)
    clear_params()

if __name__ == "__main__":
    # test()
    clear_params()