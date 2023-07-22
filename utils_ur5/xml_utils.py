
import xml.etree.ElementTree as ET

def parse_xml_file(file_path):
    try:
        tree = ET.parse(file_path)
        return tree.getroot()
    except ET.ParseError as e:
        print(f"Error parsing XML file: {e}")
        return None

def find_element(root, element_name):
    elements = root.findall(".//{}".format(element_name))
    if elements:
        return elements[0]
    return None

def modify_element(element, new_text):
    element.text = new_text

def get_xml_content(root):
    return ET.tostring(root, encoding="utf-8", method="xml").decode()


def save_xml_file(root, file_path):
    try:
        tree = ET.ElementTree(root)
        tree.write(file_path, encoding="utf-8", xml_declaration=True)
        print("XML file saved successfully.")
    except Exception as e:
        print(f"Error saving XML file: {e}")

# Test
if __name__ == "__main__":
    xml_path = '/home/toto/ros_ws/src/UR5BlokVision/locosim/ros_impedance_controller/worlds/models/X1-Y1-Z2/mesh_X1-Y1-Z2.sdf'
    root = parse_xml_file(xml_path)
    element = find_element(root, 'static')
    modify_element(element, '1')
