import processing.core.*; 
import processing.xml.*; 

import processing.serial.*; 
import controlP5.*; 

import java.applet.*; 
import java.awt.Dimension; 
import java.awt.Frame; 
import java.awt.event.MouseEvent; 
import java.awt.event.KeyEvent; 
import java.awt.event.FocusEvent; 
import java.awt.Image; 
import java.io.*; 
import java.net.*; 
import java.text.*; 
import java.util.*; 
import java.util.zip.*; 
import java.util.regex.*; 

public class PIDController extends PApplet {

  

ControlP5 controlP5;

Textfield P;
Textfield I;
Textfield D;
Textfield targetAngle;

String stringP = "";
String stringI = "";
String stringD = "";
String stringTargetAngle = "";

PFont f;

boolean useDropDownLists = true; // Set if you want to use the dropdownlist or not
byte defaultComPort = 0;
int defaultBaudrate = 115200;

//Dropdown lists
DropdownList COMports; // Define the variable ports as a Dropdownlist.
Serial xbee; // Define the variable port as a Serial object.
int portNumber = -1; // The dropdown list will return a float value, which we will connvert into an int. We will use this int for that.

DropdownList baudrate;
int selectedBaudrate = -1; // Used to indicate which baudrate has been selected
String[] baudrates = {
  "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200" // these are the supported baudrates by a xbee module
};
boolean connectedSerial;
boolean aborted;

public void setup()
{
  controlP5 = new ControlP5(this);
  size(337, 230);

  f = loadFont("EuphemiaUCAS-Bold-30.vlw");
  textFont(f, 30);
  
  P = controlP5.addTextfield("P", 10, 65, 35, 20);
  P.setFocus(true);
  I = controlP5.addTextfield("I", 50, 65, 35, 20);
  D = controlP5.addTextfield("D", 90, 65, 35, 20);
  targetAngle = controlP5.addTextfield("TargetAngle", 130, 65, 35, 20);

  P.setAutoClear(false);
  I.setAutoClear(false);
  D.setAutoClear(false);
  targetAngle.setAutoClear(false);

  P.clear();
  I.clear();
  D.clear();
  targetAngle.clear();

  controlP5.addButton("Submit", 0, 202, 65, 60, 20);
  controlP5.addButton("Clear", 0, 267, 65, 60, 20);

  controlP5.addButton("Abort", 0, 10, 200, 40, 20);
  controlP5.addButton("Continue", 0, 55, 200, 50, 20);
  controlP5.addButton("StoreValues", 0, 267, 200, 60, 20);

  //println(xbee.list()); // Used for debugging
  if (useDropDownLists)
  {
    /* Drop down lists */
    COMports = controlP5.addDropdownList("COMPort", 10, 20, 150, 200); // Make a dropdown list with all comports
    customize(COMports); // Setup the dropdownlist by using a function

    baudrate = controlP5.addDropdownList("Baudrate", 165, 20, 55, 200); // Make a dropdown with all the available baudrates   
    customize(baudrate); // Setup the dropdownlist by using a function

    controlP5.addButton("Connect", 0, 225, 3, 45, 15);
    controlP5.addButton("Disconnect", 0, 275, 3, 52, 15);
  }
  else // if useDropDownLists is false, it will connect automatically at startup
  {
    xbee = new Serial(this, Serial.list()[defaultComPort], defaultBaudrate);
    xbee.bufferUntil('\n');
    connectedSerial = true;
    xbee.write("G;"); // Go
  }
}

public void draw() 
{
  background(0);

  fill(0, 102, 153);
  textSize(30); 
  textAlign(CENTER); 
  text("Set PID Values:", width/2, 55);
  text("Current PID Values:", width/2, 150);

  fill(255, 255, 255);
  textSize(10);  
  textAlign(LEFT);
  text("P: " + stringP + " I: " + stringI +  " D: " + stringD + " TargetAngle: " + stringTargetAngle, 10, 175); // Motor
}
public void Abort(int theValue)
{
  if (connectedSerial) 
  {
    xbee.write("A;");
    aborted = true;
  }
  else
    println("Establish a serial connection first!");
}
public void Continue(int theValue)
{
  if (connectedSerial) 
  {
    xbee.write("C;");
    aborted = false;
  }
  else
    println("Establish a serial connection first!");
}
public void Submit(int theValue) 
{
  if (connectedSerial)
  {    
    println("PID values: " + P.getText() + " " + I.getText() + " " + D.getText() +  " TargetAnlge: " + targetAngle.getText());
    String output1 = "T," + targetAngle.getText() + ';';
    String output2 = "P," + P.getText() + ';';
    String output3 = "I," + I.getText() + ';';
    String output4 = "D," + D.getText() + ';';

    xbee.write(output1);
    delay(10);
    xbee.write(output2);
    delay(10);
    xbee.write(output3);
    delay(10);
    xbee.write(output4);
    delay(10);    
    xbee.write("G;"); // Send values back to application
    delay(10);    
  }
  else
    println("Establish a serial connection first!");
}
public void Clear(int theValue) 
{
  P.clear();
  I.clear();
  D.clear();
  targetAngle.clear();
}
public void StoreValues(int theValue)
{
  //Don't set the text if the string is empty or it will crash
  if (stringP != null)
    P.setText(stringP);
  if (stringI != null)
    I.setText(stringI);
  if (stringD != null)
    D.setText(stringD);
  if (stringTargetAngle != null)
    targetAngle.setText(stringTargetAngle);
}
public void serialEvent(Serial xbee)
{
  String[] input = trim(split(xbee.readString(), ','));
  for (int i = 0; i<input.length;i++)
    println("Number: " + i + " Input: " + input[i]); // For debugging

  if (input[0].equals("Initialized"))
    println("Started");
  else if(input[0].equals("Processing"))
  {    
    if (input[1].length() > 6)
      stringP = input[1].substring(0, 6);
    else
      stringP = input[1];
    if (input[2].length() > 6)
      stringI = input[2].substring(0, 6);
    else
      stringI = input[2];
    if (input[3].length() > 6)
      stringD = input[3].substring(0, 6);
    else
      stringD = input[3];
    if (input[4].length() > 6)
      stringTargetAngle = input[4].substring(0, 6);
    else
      stringTargetAngle = input[4];
  }
  else
    println(input[0]);
  xbee.clear();  // Empty the buffer
}
public void keyPressed() 
{
  if (key == TAB)//'\t'
  {
    if (P.isFocus())
    {
      P.setFocus(false);
      I.setFocus(true);
    }
    else if (I.isFocus())
    {
      I.setFocus(false);
      D.setFocus(true);
    }
    else if (D.isFocus())
    {
      D.setFocus(false);
      targetAngle.setFocus(true);
    }
    else if (targetAngle.isFocus())
    {
      targetAngle.setFocus(false);
      P.setFocus(true);
    }    
    else
      P.setFocus(true);
  }
  else if (key == ENTER) // '\n'
    Submit(0);
  else if (key == ESC)
  {
    if (aborted)
      Continue(0);
    else
      Abort(0);      
    key = 0; // Disable Processing from quiting when pressing ESC
  }
  else if (key == CODED)
  {
    if (keyCode == UP)
      StoreValues(0);
  }
}
public void customize(DropdownList ddl) 
{
  ddl.setBackgroundColor(color(200));//Set the background color of the line between values
  ddl.setItemHeight(20);//Set the height of each item when the list is opened.
  ddl.setBarHeight(15);//Set the height of the bar itself.

  ddl.captionLabel().style().marginTop = 3;//Set the top margin of the lable.  
  ddl.captionLabel().style().marginLeft = 3;//Set the left margin of the lable.  
  ddl.valueLabel().style().marginTop = 3;//Set the top margin of the value selected.

  if (ddl.name() == "Baudrate")
  {
    ddl.captionLabel().set("Baudrate");
    for (int i=0; i<baudrates.length; i++)
      ddl.addItem(baudrates[i], i); // give each item a value
  }
  else if (ddl.name() == "COMPort")
  {
    ddl.captionLabel().set("Select COM port");//Set the lable of the bar when nothing is selected. 
    //Now well add the ports to the list, we use a for loop for that.
    for (int i=0; i<xbee.list().length; i++)    
      ddl.addItem(xbee.list()[i], i);//This is the line doing the actual adding of items, we use the current loop we are in to determin what place in the char array to access and what item number to add it as.
  }
  ddl.setColorBackground(color(60));
  ddl.setColorActive(color(255, 128));
}
public void controlEvent(ControlEvent theEvent) {
  if (theEvent.isGroup()) 
  {
    if (theEvent.group().name() == "COMPort")     
      portNumber = PApplet.parseInt(theEvent.group().value());//Since the list returns a float, we need to convert it to an int. For that we us the int() function.    
    else if (theEvent.group().name() == "Baudrate")    
      selectedBaudrate = PApplet.parseInt(theEvent.group().value());
  }
}
public void Connect(int theValue)
{     
  if (selectedBaudrate != -1 && portNumber != -1 && !connectedSerial)//Check if com port and baudrate is set and if there is not already a connection established
  {
    println("ConnectSerial");    
    xbee = new Serial(this, Serial.list()[portNumber], Integer.parseInt(baudrates[selectedBaudrate]));
    connectedSerial = true;
    xbee.bufferUntil('\n');
    xbee.write("G;"); // Go
  }
  else if (portNumber == -1)
    println("Select COM Port first!");
  else if (selectedBaudrate == -1)
    println("Select baudrate first!");
  else if (connectedSerial)
    println("Already connected to a port!");
}
public void Disconnect(int theValue)
{
  if (connectedSerial)//Check if there is a connection established
  {
    println("DisconnectSerial");
    xbee.stop();
    xbee.clear(); // Empty the buffer
    connectedSerial = false;
  }
  else
    println("Couldn't disconnect");
}

  static public void main(String args[]) {
    PApplet.main(new String[] { "--bgcolor=#FFFFFF", "PIDController" });
  }
}
