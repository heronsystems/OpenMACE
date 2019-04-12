import io
import lxml.etree as ET
#using lxml instead of xml preserved the comments

# Parse arguments:


#adding the encoding when the file is opened and written is needed to avoid a charmap error
with io.open('../Ground_Template.xml', encoding="utf8") as f:
  tree = ET.parse(f)
  root = tree.getroot()


  for elem in root.getiterator():
    try:
      elem.text = elem.text.replace('GUI_HOST_ADDRESS', guiHostAddress)
      elem.text = elem.text.replace('ETHERNET_PORT_NUMBER', ethernetPortNumber)
      elem.text = elem.text.replace('LISTEN_PORT_NUMBER', listenPortNumber)
    except AttributeError:
      pass

#tree.write('output.xml', encoding="utf8")
# Adding the xml_declaration and method helped keep the header info at the top of the file.
#filename = 'Ground'.format(vehicleID)
tree.write('GroundInstance.xml', xml_declaration=True, method='xml', encoding="utf8")
