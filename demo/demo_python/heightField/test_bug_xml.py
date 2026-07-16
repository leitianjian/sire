
import xml.etree.ElementTree as ET

tree = ET.parse('sire_ball_free_fall.xml')
root = tree.getroot()
parts = root.find('Model').find('PartPool')
for part in parts:
    print(part.attrib)

