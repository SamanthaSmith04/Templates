"""
    Gets the base link and end effector link from the URDF.xacro file
    Parameters:
        file_path - the path to the URDF.xacro file
    Returns:
        base - the base link
        eef - the end effector link
"""
def get_links_from_urdf(file_path):
    #Convert XACRO to URDF
    processed_xacro = xacro.process_file(file_path)
    out = "/home/samubuntu/AA_DEVEL/ws_arp/src/augmented-reality-painting/arp_motion_plan/scripts/out.urdf"
    with open(out, 'w') as urdf_file:
        urdf_file.write(processed_xacro.toxml())
        urdf_file.close()
    urdf_from_file = urdf.URDF.from_xml_file(out)

    #get base link
    base = urdf_from_file.links[0] 
    base = base.name

    #get EEF link
    eef = urdf_from_file.links[-1] #gets last link in the list which should be the endeffector link
    eef = eef.name
    print(base) 
    print("==========")
    print(eef)
    return base, eef
