# Dalhousie Microtransat Autonomous SailBoat Team (DalMAST)
We are a team of Dalhousie students designing and building a sustainable, autonomous sailboat that will sail itself from Nova Scotia to Ireland as part of international competition. Aside from our project, we try to promote the Ocean Engineering and Technology Industry by hosting events and competitions for students.

From 2015 to the fall of 2018, the sailboat team was managed by Dalhousie faculty, including the previous Dean of Engineering, Dr. Leon. The most recent iteration of the vessel, the Sea Leon, was launched in the summer of 2018, with great success. It travelled for more than 3700km over 76 days before it ceased to transmit its location. In 2018, the responsibilities of the sailboat were transferred over to Dalhousie engineering students.

### Websites: 
Our website: [dalmast.squarespace.com](https://dalmast.squarespace.com), The competition: [microtransat.org](https://www.microtransat.org/)

### Current Software Team
As of Feb 2022:

Ethan Johnston - Team Lead  
Yiming Zhang - Team Member  
Ope Adelasoye - Team Member  
Grant Sutherland - Team Member  
Mathew Cockburn - Actually on electrical team but really likes to code so we are borrowing them.

### Credits

Alex Whidden - Conversion to FreeRTOS - REPORT  
Blake Meech  - REPORT  
Anthony Chalmers  
Serge Toutsenko  
Julia Sarty  
Thomas Gwynne-Timothy  - REPORT  
Jean Francois Bousquet  

Let Ethan know if there is anyone else.

## Technical Details and Background

Wiring, devices, mechanical background

## Using a RTOS (FreeRTOS)

How this works and how to add tasks and how the kernal works.

## Uploading Code

1. Plug in development board  
2. Open Microchip Studio (Start Page should recognize SAM D20)  
4. Open project (New_RTOS_Sailboat.atsln)  
5. Open Putty (Speed: 57600 | Connection type: Serial)  
6. Open Device Manager to make sure Virtual COM Port is equivalent to 'Serial line'  
7. 'Open'
8. Move *Back to Microchip Studio*   
9. Build -> Build Solution  
11. Run  

## Uploading Waypoints:

Creating KML waypoints file from Google Earth:
1. Click the placemark icon on the bottom left and then select a place on the map.
2. Select new project on the window that shows up and add in the project title, then click ok.
3. Fill out the name and description of the placemark in the window that shows up. This is auto saved.
4. Repeat the above steps for any additional placemarks you want to add, but instead of new project, save to your
   existing project.
5. Click the menu option (3 lines) on the top left and then select your project by right-clicking.
6. Click the 3 dots on the top right of the new window, and select export as KML file. This is the waypoints file.




Uploading to EEPROM using Processing KML Uploader script
1. [Download Processing](https://processing.org/download)
2. Place .kml in the same directory as the processing script. Rename the .kml file to OUTPUT.kml.  
3. Connect computer to the main board (big boy) and the XBee V3 USB radio transmitter.
4. Upload the most up to date software on main board. (If it is already on there, dont worry about it)
5. Open Putty. Enter the above port for the main board and the serial number as 57600, then proceed. This will allow you to verify
   that the waypoints are being uploaded properly.
6. Open the KML_TO_WAYPOINT_UPLOADER.pde processing script.
7. Change the port name at the top to the port name you obtained from the device manager.
8. Ensure that in the function parseKML, the loadStrings function is reading the file OUTPUT.kml.
9. Run the processing script with the play button.
10. That's it. You can see the functioning in the Putty window.

## Function Map Diagram

## Navigation

## Radio Message Decoding Table

## Using Test Functions

## GUI setup instructions 
In order to run the GUI that communicates with the radio module, **Processing** must be downloaded,
which you can do [here.](https://processing.org/download/)

After installing, import the G4P library under Sketch > Import Library > Add Library > G4P

To run, open the file of type Proccessing Source Code

![Legend](GUI_Legend.png "GUI Legend")

## Useful Links and Resources

[OLD Sailboat Overview](Sailboat_Docs/presentation/sailboat_overview.pptx)  
[Thomas Report (Good for Radio Codes)](Sailboat_Docs/report/master.pdf)  
[dalmast.squarespace.com](https://dalmast.squarespace.com)  
