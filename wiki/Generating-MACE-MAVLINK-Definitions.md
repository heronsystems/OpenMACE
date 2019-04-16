The tool used to generate the appropriate MACE MAVLINK definition files was a custom ported python script based on a development tool from the [Ardupilot](https://github.com/ArduPilot/ardupilot) code base. The port was migrated to the MACE framework and is housed in the mavlink_cpp/MACE subdirectory of this code base. 

# Setup 

## Prerequisites
The only prerequisites are that python is installed (see [HERE](https://www.python.org/downloads/) for appropriate OS download and installation steps) and check that you can execute python scripts from the command line, within a shell, or as a executable file. Current checking has been done with Python 2.0. 

Additionally, you need to install the pymavlink-mace package from this repository. It is located in (see [HERE](https://www.python.org/downloads/). This can be installed from the command line by navigating to the source directory and

```
python setup.py install
```
Once installation is complete, ensure that python can see the installed package. 

## Configuring MACE MAVLINK XML Schema 
Everything within the MAVLINK definitions is contained to individualized xml files governed by a schema. All subsequent definition files should be linked appropriately as contained at the top of the `mace_common.xml` file. An example is shown below modifying the original `mace_common.xml` and linking to an additional xml definition file called `your_custom_defition_file.xml`

```
<?xml version="1.0"?>
<mavlink>
  <include>common.xml</include>
  <include>mission.xml</include>
  <include>boundary.xml</include>
  <include>your_custom_defition_file.xml</include>
```
It is the responsibility of the developer to be diligent about code reuse and already existing definitions. If there are modifications necessary to any of the previous definitions, please update the appropriate definition rather than creating a new definition. As listed above, each included file in the definition is built in order. It should be recognized that there are a limited number of msg ID slots available per the MAVLINK definition. Each subsequent file added should allow room in this ID definition per the previous file before starting their ID definitions. Example being `mission.xml` contains definitions for messsage IDS 100->120. Leaving some room for potential follow on development, `boundary.xml`begins its definition at 130 and progresses. A developer should create a new xml file to encompass the communications they are trying to carry between MACE instances. 
 
# Establishing a new MACE MAVLINK definition file 
To define a custom enumeration, a developer may do so by simply performing the following in their XML file

## Defining an Enumeration
To define a custom enumeration, a developer may do so by simply performing the following in their XML file

```
    <enum name="MAV_MISSION_TYPE">
      <description>Type of mission items being requested/sent in mission protocol.</description>
      <entry value="0" name="MAV_MISSION_TYPE_AUTO">
        <description>Items are mission commands for main auto mission.</description>
      </entry>
      ....
    </enum>
```

## Defining a MESSAGE
To define a custom message, a developer may do so by simply performing the following in their XML file

    <message id="MSG_ID_NUM" name="MSG_NAME">
      <description>Custom description of the message</description>
      <field type="uint8_t" name="target_system">COMPONENT_DESCRIPTION</field>
    </message>

Available Parameter Datatypes Include: 
Specifies the datatype of a MAVLink parameter.
1. 8,16,32,64 bit unsigned integers
2. 8,16,32,64 bit signed integers
3. 32,64 bit floating
**It is important that the user minimize the parameter definition wherever possible to only encapsulate the expected data to be sent. This enables the transmission when serialized to have higher bandwidth and greater throughput from the modules **

# Generating the MACE MAVLINK definition
## Mavenerate (GUI) {#mavgenerate}

**mavgenerate.py** is GUI code generator for MAVLink, written in Python.

> **Note** *Mavgenerate* provides a GUI front end to the [mavgen](#mavgen) command line code generator, and supports the same [options](#mavgen_options).

The GUI can be launched from anywhere using Python's `-m` argument:

```sh
python -m mavgenerate
```

![mavgenerate UI](../../assets/mavgen/mavlink_generator.png)

Generator Steps:
1. Choose the target XML file (typically in [mace_mavlink_generator/mavlink/mace_definitions](https://github.com/MACE/tree/master/mavlink_cpp/)).

   > **Note** If using a custom dialect, first copy it into the above directory (if the dialect is dependent on **common.xml** it must be located in the same directory).
1. Choose an output directory (e.g. **mavlink/include**).
1. Select the target output programming language.
1. Select the target MAVLink protocol version (ideally 2.0)
   > **Caution** Generation will fail if the protocol is not [supported](../README.md#supported_languages) by the selected programming language.
1. Optionally check *Validate* and/or  *Validate Units* (if checked validates XML specifications).
1. Click **Generate** to create the source files.


## Mavgen (Command Line) {#mavgen}

**mavgen.py** is a command-line tool for generating MAVLink libraries for different programming languages. 
After the `mavlink` directory has been added to the `PYTHONPATH`, it can be run by executing from the command line. 

> **Tip** This is the backend used by [mavgenerate](#mavgenerate). The documentation below explains all the options for both tools. 

For example, to generate *MAVLink 2* C libraries for a dialect named **your_custom_dialect.xml**.
```sh
python -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/your_custom_dialect.xml
```

> **Note** The syntax for for generating Python modules is the same, except that the `--output` specifies a *filename* rather than a directory.
  <!-- https://github.com/ArduPilot/pymavlink/issues/203 -->

<span id="mavgen_options"></span>
The full syntax and options can be output by running *mavgen* with the `-h` flag (reproduced below):
```
usage: mavgen.py [-h] [-o OUTPUT]
                 [--lang {C,CS,JavaScript,Python,WLua,ObjC,Swift,Java,C++11}]
                 [--wire-protocol {0.9,1.0,2.0}] [--no-validate]
                 [--error-limit ERROR_LIMIT] [--strict-units]
                 XML [XML ...]

This tool generate implementations from MAVLink message definitions

positional arguments:
  XML                   MAVLink definitions

optional arguments:
  -h, --help            show this help message and exit
  -o OUTPUT, --output OUTPUT
                        output directory.
  --lang {C,CS,JavaScript,Python,WLua,ObjC,Swift,Java,C++11}
                        language of generated code [default: Python]
  --wire-protocol {0.9,1.0,2.0}
                        MAVLink protocol version. [default: 1.0]
  --no-validate         Do not perform XML validation. Can speed up code
                        generation if XML files are known to be correct.
  --error-limit ERROR_LIMIT
                        maximum number of validation errors to display
  --strict-units        Perform validation of units attributes.
```
