
#!/usr/bin/env python
import os
import subprocess
import sys
from sys import argv
import argparse
import io
import lxml.etree as ET
#using lxml instead of xml preserved the comments

def generateVehicleXML(id, elPort, sitlPort, relativeLogDir):
    mace_root_dir = os.environ['MACE_ROOT']
    file_prefix = '/MaceSetup_Configs/SimEnvironment/'
    with io.open(mace_root_dir + file_prefix + 'Vehicle_Template.xml', encoding="utf8") as f:
        tree = ET.parse(f)
        root = tree.getroot()

	root.set('MaceInstance', str(id))
        for elem in root.getiterator():
            try:
                elem.text = elem.text.replace('ETHERNET_PORT_NUMBER', str(elPort))
                elem.text = elem.text.replace('LISTEN_PORT_NUMBER', str(sitlPort))
            except AttributeError:
                pass



    #tree.write('output.xml', encoding="utf8")
    # Adding the xml_declaration and method helped keep the header info at the top of the file.
    #current_dir = os.getcwd()
    filename = '/Vehicle_{}.xml'.format(id)
    relative_filepath = relativeLogDir + filename
    #print(relative_filepath)
    tree.write(mace_root_dir + relative_filepath, xml_declaration=True, method='xml', encoding="utf8")
    return relative_filepath


#print(generateVehicleXML(1, 10, 100))

# Parse command line arguments:
#parser = argparse.ArgumentParser()
#parser.add_argument('id', type=str, help='Vehicle ID')
#parser.add_argument('elPort', type=str, help='TCP port number for External Link comms over Ethernet')
#parser.add_argument('sitlPort', type=str, help='UDP port number for vehicle to listen on for the ArduPilot SITL instance')
# parser.add_argument('fp', type=str, help='Relative file path to the launch script JSON configuration file. (e.g.: ./Vehicle_Template.xml')
#args = parser.parse_args()

# Call function:
#print(generateVehicleXML(args.id, args.elPort, args.sitlPort))


# #!/usr/bin/env python
# def get_value(*args):
#     return "Hello World " + ":".join(map(str, args))

# def main(argv):
#     print(get_value(*argv[1:]))

# if __name__ == "__main__":
#     import sys
#     main(sys.argv)
