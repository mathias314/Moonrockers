# Microcontroller Code  
This is the code for the Arduino microcontroller used for the low-level device interface.  

This portion of the project was developed in PlatformIO running in VS Code.  

## Project Structure  
- **src** - Main source file(s) for the project.  
- **lib** - Non-public libraries specific to this project. The `main` library in this folder is the main library for the project and serves as a dumping ground for everything it doesn't make sense to split out.  
- **include** - Folder for `.h` files associated with `src` files (currently not used in this project).  
- **examples** - Example programs which isolate aspects of functionality for debugging and development.  
- **test** - Contains the (somewhat limited) unit testing for this project.     

In addition to these folders, there are two PIO configuration files. 
 - **platformio.ini** - The primary configuration file for the project. This defines the **list of included libraries**, the default baud rate for the serial monitor, and the **board for which the code will be compiled**.  
  - **examples.ini** - Contains some extra configs for example programs. Each example has its own target env defined in the file.   

## Notes on PIO  
The code in the project can be uploaded by simply running `pio run -t upload`.  This will download all dependancies, compile the project, automatically detect the COM port of the device, and upload.  

Some additional PIO commands...
 - **Getting a device list**: `pio device list`
 - **Opening a serial monitor**: `pio device monitor`
 - **Running an example program**: `pio run -t upload -c examples.ini -e <example_name>`  
 - **Running tests**: `pio test -f <test_name>`  
