def pretty_parse_object_or_furniture_str(str_name:str):
    target_str = "Default Name"
    str_list = str_name.split('_')
    if len(str_list)==1:
        target_str = str_list[0]
    else:
        # Check if first element of target_str_list is a number
        target_str = ' '.join(str_list[1:]) if str_list[0].isdigit() else ' '.join(str_list)

    return target_str

def remap_od_object_labels_to_ui_object_labels(od_object_label:str):
    od_label_to_ui_label_dict = {
        "pineapple plush toy": "pineapple toy",
        "pink donut plush toy": "donut toy",
        "avocado plush toy": "avocado toy",
        "cup": "cup",
        "bottle": "bottle",
        "can": "can",
    }
    
    return od_label_to_ui_label_dict.get(od_object_label, od_object_label)
