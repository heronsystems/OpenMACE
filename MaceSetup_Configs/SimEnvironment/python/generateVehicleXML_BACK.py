
#!/usr/bin/env python
from os import system
import subprocess
import sys
from sys import argv
import argparse
import lxml.etree as ET
#using lxml instead of xml preserved the comments

def generateVehicleXML(id, elPort, sitlPort):
    # Parse command line arguments:
    parser = argparse.ArgumentParser()
    parser.add_argument('id', type=str, help='Vehicle ID')
    parser.add_argument('elPort', type=str, help='TCP port number for External Link comms over Ethernet')
    parser.add_argument('sitlPort', type=str, help='UDP port number for vehicle to listen on for the ArduPilot SITL instance')
    # parser.add_argument('fp', type=str, help='Relative file path to the launch script JSON configuration file. (e.g.: ./Vehicle_Template.xml')
    args = parser.parse_args()
    with open('Vehicle_Template.xml', encoding="utf8") as f:
        tree = ET.parse(f)
        root = tree.getroot()

        for elem in root.getiterator():
            try:
                elem.text = elem.text.replace('ETHERNET_PORT_NUMBER', args.elPort)
                elem.text = elem.text.replace('LISTEN_PORT_NUMBER', args.sitlPort)
            except AttributeError:
                pass



    #tree.write('output.xml', encoding="utf8")
    # Adding the xml_declaration and method helped keep the header info at the top of the file.
    filename = 'Vehicle_{}'.format(args.id)
    tree.write(filename, xml_declaration=True, method='xml', encoding="utf8")
    return filename
