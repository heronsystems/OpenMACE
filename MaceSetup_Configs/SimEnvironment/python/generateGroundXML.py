
#!/usr/bin/env python
import os
import subprocess
import sys
from sys import argv
import argparse
import io
import lxml.etree as ET
#using lxml instead of xml preserved the comments

def generateGroundXML(guiHostAddress, elPort, relativeLogDir):
    mace_root_dir = os.environ['MACE_ROOT']
    file_prefix = '/MaceSetup_Configs/SimEnvironment/'
    with io.open(mace_root_dir + file_prefix + 'Ground_Template.xml', encoding="utf8") as f:
        tree = ET.parse(f)
        root = tree.getroot()

        for elem in root.getiterator():
            try:
                elem.text = elem.text.replace('GUI_HOST_ADDRESS', str(guiHostAddress))
                elem.text = elem.text.replace('ETHERNET_PORT_NUMBER', str(elPort))
            except AttributeError:
                pass



    #tree.write('output.xml', encoding="utf8")
    # Adding the xml_declaration and method helped keep the header info at the top of the file.
    #current_dir = os.getcwd()
    filename = '/GroundInstance.xml'
    relative_filepath = relativeLogDir + filename
    #print(relative_filepath)
    tree.write(mace_root_dir + relative_filepath, xml_declaration=True, method='xml', encoding="utf8")
    return relative_filepath
