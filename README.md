**NodeOven**

NodeOven is a node, used at the MakerSpace Leiden, to control a Ceramic oven. This repository has the following subdirs:

- **Drawings:**

Schematic of the wiring between the different modules used

- **KiCad\_files:**

The design of the PCB for the Node/Controller. De PCB in this design is not meant to be used for a real PCB. It is used as guideline for a handmade PCB

- **Photos:**

Some photos of the Node made.

- **PlatformIO\_Files:**

The source code for the Node and the display used. For the development of this software PlatformIO is used, an Extension on Visual Studio Code.

For measuring the oven temperature different types of thermocouples can be used. The configuration of the correct one is done in:

/PlatformIO\_Files/NodeOven/platformio.ini

See for possible types to be used, the comment in this file for THERMOCOUPLE\_TYPE=