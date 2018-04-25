# NetTableToSocket

This project contains a small java application that can be used
to pull data from the WPI Network Tables and write it to a socket.

A single configuration file can be used to define what network
table, what socket(host/port/protocol), and the content of the
message sent to the socket. The message_config.xml file is an
example of this configuration file.

The NetTableToSocket.jar file contains the NetTableToSocket
classes that are generated when compiling this project. This
jar file can be built using this project. It is also included
so you do not need a java development environment to use it.

The WPINetTable.jar file contains the components that interface
with the WPI NetworkTables. This is a stripped down version of
the WPI SmartDashboard.jar with everything removed except the
Network Table related classes. This jar file SHOULD ONLY BE USED
FOR TESTING when a WPI provided SmartDashboard.jar file is not 
readily available.

To use the application, modify a copy of the configuration file to
set your network interface and to define your message with items
from your SmartDashboard. Then start the application using your
SmartDashboard.jar file [or the provided WPINetTable.jar file].

## Running the Application

From the command line go to the <project>/dist directory and enter:

`java -cp <path_to_wpi_tools>/SmartDashboard.jar;NetTableToSocket.jar HoloFirst.NetTableToSocket message_config.xml`

or

`java -cp WPINetTable.jar;NetTableToSocket.jar HoloFirst.NetTableToSocket message_config.xml`

**Note:** *On non-window machines the class-path separator is ':'*

## Building the Jar with Eclipse

Open the project in eclipse. If the project is not already in your 
Eclipse environment, you can select File -> Import ... ->
General -> Existing Projects into Workspace, and Browse to
the directory that contains this project. Depending on the 
settings in your Eclipse environment the project may build 
automatically. You can build on demand by right-clicking on the 
Build.xml file (in the Eclipse Package Explorer) and select 
Run As -> Ant Build. The jar file and all files required to run 
will be placed in the "dist" directory.

The Build.xml file can be modified to include the SmartDashboard.jar
file in the class-path so it is easier to start the NetTableToSocket
application. If the Build.xml file is modified, you can run with:

`java -jar NetTableToSocket.jar message_config.xml`
