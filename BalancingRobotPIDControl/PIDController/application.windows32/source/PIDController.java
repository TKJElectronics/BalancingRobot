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

/* Motor */
Textfield P;
Textfield I;
Textfield D;
Textfield targetAngle;

String stringP = "";
String stringI = "";
String stringD = "";
String stringTargetAngle = "";

/* Encoders */
Textfield wheelP;
//Textfield wheelI;
Textfield wheelD;
Textfield targetPosition;
//Textfield leftTargetPosition;
//Textfield rightTargetPosition;

String stringWheelP = "";
//String stringWheelI = "";
String stringWheelD = "";
String stringTargetPosition = "";
//String stringRightTargetPosition = "";

PFont f;

boolean useDropDownLists = true; // Set if you want to use the dropdownlist or not
byte defaultComPort = 0;
int defaultBaudrate = 115200;

//Dropdown lists
DropdownList COMports; // Define the variable ports as a Dropdownlist.
Serial XBee; // Define the variable port as a Serial object.
int portNumber = -1; // The dropdown list will return a float value, which we will connvert into an int. we will use this int for that.

DropdownList baudrate;
int selectedBaudrate = -1;//Used to indicate that no baudrate has been selected
String[] baudrates = {
  "9600", "115200"
};//Avaible baudrates - add more if you need it
boolean connectedSerial;

boolean aborted;

public void setup()
{
  controlP5 = new ControlP5(this);
  size(337, 330);

  f = loadFont("EuphemiaUCAS-Bold-30.vlw");
  textFont(f, 30);

  /* Motors */
  P = controlP5.addTextfield("P", 10, 65, 40, 20);
  P.setFocus(true);
  I = controlP5.addTextfield("I", 55, 65, 40, 20);
  D = controlP5.addTextfield("D", 100, 65, 40, 20);
  targetAngle = controlP5.addTextfield("TargetAngle", 145, 65, 40, 20);

  P.setAutoClear(false);
  I.setAutoClear(false);
  D.setAutoClear(false);
  targetAngle.setAutoClear(false);

  P.clear();
  I.clear();
  D.clear();
  targetAngle.clear();

  /* Left Encoder */
  wheelP = controlP5.addTextfield("wheelP", 10, 100, 40, 20);
  //wheelI = controlP5.addTextfield("wheelI", 55, 100, 40, 20);
  wheelD = controlP5.addTextfield("wheelD", 55, 100, 40, 20);
  targetPosition = controlP5.addTextfield("TargetPosition", 10, 135, 40, 20);
  //rightTargetPosition = controlP5.addTextfield("rightTarget", 100, 135, 40, 20);

  wheelP.setAutoClear(false);
  //wheelI.setAutoClear(false);
  wheelD.setAutoClear(false);
  targetPosition.setAutoClear(false);
  //rightTargetPosition.setAutoClear(false);

  wheelP.clear();
  //wheelI.clear();
  wheelD.clear();
  targetPosition.clear();
  //rightTargetPosition.clear();

  controlP5.addButton("Submit", 0, 197, 135, 60, 20);
  controlP5.addButton("Clear", 0, 267, 135, 60, 20);

  controlP5.addButton("Abort", 0, 10, 300, 40, 20);
  controlP5.addButton("Continue", 0, 55, 300, 50, 20);
  controlP5.addButton("StoreValues", 0, 267, 300, 60, 20);

  //println(XBee.list()); // Used for debugging
  if (useDropDownLists)
  {
    //Drop down lists
    //Make a dropdown list calle ports. Lets explain the values: ("name", left margin, top margin, width, height (84 here since the boxes have a height of 20, and theres 1 px between each item so 4 items (or scroll bar).
    COMports = controlP5.addDropdownList("COMPort", 10, 20, 150, 200);
    //Setup the dropdownlist by using a function. This is more pratical if you have several list that needs the same settings.
    customize(COMports);

    baudrate = controlP5.addDropdownList("Baudrate", 165, 20, 55, 200);
    customize(baudrate);

    controlP5.addButton("Connect", 0, 225, 3, 45, 15);
    controlP5.addButton("Disconnect", 0, 275, 3, 52, 15);
  }
  else
  {
    XBee = new Serial(this, Serial.list()[defaultComPort], defaultBaudrate);
    XBee.bufferUntil('\n');
    connectedSerial = true;
    XBee.write("G"); // Go
  }
}

public void draw() 
{
  background(0);

  fill(0, 102, 153);
  textSize(30); 
  textAlign(CENTER); 
  text("Set PID Values:", width/2, 55);
  text("Current PID Values:", width/2, 210);

  fill(255, 255, 255);
  textSize(10);  
  textAlign(LEFT);
  text("P: " + stringP + " I: " + stringI +  " D: " + stringD + " TargetAngle: " + stringTargetAngle, 10, 235); // Motor
  text("WheelP: " + stringWheelP + /*" WheelI: " + stringWheelI +*/ " WheelD: " + stringWheelD, 10, 255); //Encoders
  text("TargetPosition: " + stringTargetPosition /*stringLeftTargetPosition  + " RightTargetPosition: " + stringRightTargetPosition*/, 10, 275);
}
public void Abort(int theValue)
{
  XBee.write("A");
  aborted = true;
}
public void Continue(int theValue)
{
  XBee.write("C");
  aborted = false;
}
public void Submit(int theValue) 
{
  if (connectedSerial)
  {    
    /* Motors */
    println("PID values: " + P.getText() + " " + I.getText() + " " + D.getText() +  " TargetAnlge: " + targetAngle.getText());
    String output1 = "1," + P.getText() + ',' + I.getText() + ',' + D.getText() + '\0';
    String output2 = "2," + targetAngle.getText() + '\0';
    /* Encoders */
    println("PID Wheels Values: " + wheelP.getText() + " " /*+ wheelI.getText() + " "*/ + wheelD.getText() +  " targetPosition: " + targetPosition.getText());//" leftTargetPosition: " + leftTargetPosition.getText() +  " rightTargetPosition: " + rightTargetPosition.getText());
    String output3 = "3," + wheelP.getText() + ','/* + wheelI.getText()*/ + wheelD.getText() + '\0';
    String output4 = "4," + targetPosition.getText() + '\0';//+ leftTargetPosition.getText() + ',' + rightTargetPosition.getText() + '\0';

    XBee.write(output1);
    delay(100);
    XBee.write(output2);
    delay(100);
    XBee.write(output3);
    delay(100);
    XBee.write(output4);
    delay(100);
  }
  else
    println("Establish a serial connection first!");
}
public void Clear(int theValue) 
{
  /* Motors */
  P.clear();
  I.clear();
  D.clear();
  targetAngle.clear();
  /* Encoders */
  wheelP.clear();
  //wheelI.clear();
  wheelD.clear();
  targetPosition.clear();
  //rightTargetPosition.clear();
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

  /* Encoders */
  if (stringWheelP != null)
    wheelP.setText(stringWheelP);
  //  if (stringWheelI != null)
  //  wheelI.setText(stringWheelI);
  if (stringWheelD != null)
    wheelD.setText(stringWheelD);
  if (stringTargetPosition != null)
    targetPosition.setText(stringTargetPosition);  
  //  if (stringRightTargetPosition != null)
  //  rightTargetPosition.setText(stringRightTargetPosition);
}
public void serialEvent(Serial XBee)
{
  String[] input = trim(split(XBee.readString(), ','));
  for (int i = 0; i<input.length;i++)
    println("Number: " + i + " Input: " + input[i]); // For debugging

  if (input[0].equals("Initialized"))
    println("Started");
  else
  {
    /* Motor */
    if (input[0].length() > 6)
      stringP = input[0].substring(0, 6);
    else
      stringP = input[0];
    if (input[1].length() > 6)
      stringI = input[1].substring(0, 6);
    else
      stringI = input[1];
    if (input[2].length() > 6)
      stringD = input[2].substring(0, 6);
    else
      stringD = input[2];
    if (input[3].length() > 6)
      stringTargetAngle = input[3].substring(0, 6);
    else
      stringTargetAngle = input[3];
    /* Encoders */
    if (input[4].length() > 6)
      stringWheelP = input[4].substring(0, 6);
    else
      stringWheelP = input[4];
    /*
    if (input[5].length() > 5)
     stringWheelI = input[5].substring(0, 5);
     else
     stringWheelI = input[5];
     */
    if (input[5].length() > 6)
      stringWheelD = input[5].substring(0, 6);
    else
      stringWheelD = input[5];
    if (input[6].length() > 6)
      stringTargetPosition = input[6].substring(0, 6);
    else
      stringTargetPosition = input[6];
    /*
    if (input[7].length() > 6)
     stringRightTargetPosition = input[7].substring(0, 6);
     else
     stringRightTargetPosition = input[7];
     */
  }
  XBee.clear();  // Empty the buffer
}
public void keyPressed() 
{
  if (key == TAB)//'\t'
  { 
    /* Motor */
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
      wheelP.setFocus(true);
    }
    /* EncoderS */
    else if (wheelP.isFocus())
    {
      wheelP.setFocus(false);
      //      wheelI.setFocus(true);
      wheelD.setFocus(true);
    }
    /*    else if (wheelI.isFocus())
     {
     wheelI.setFocus(false);
     wheelD.setFocus(true);
     }*/
    else if (wheelD.isFocus())
    {
      wheelD.setFocus(false);
      targetPosition.setFocus(true);
    }
    else if (targetPosition.isFocus())
    {
      targetPosition.setFocus(false);
      //     rightTargetPosition.setFocus(true);
      P.setFocus(true);
    }
    /*    
     else if (rightTargetPosition.isFocus())
     {
     rightTargetPosition.setFocus(false);
     P.setFocus(true);
     }    
     */
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
    if (keyCode == 16)
      StoreValues(0);
    if (connectedSerial)
    {
      if (keyCode == LEFT)
        XBee.write("L");
      if (keyCode == UP)
        XBee.write("U");
      if (keyCode == DOWN)
        XBee.write("D");
      if (keyCode == RIGHT)
        XBee.write("R");
    }
    else
      println("Establish a serial connection first!");
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
    for (int i=0; i<XBee.list().length; i++)    
      ddl.addItem(XBee.list()[i], i);//This is the line doing the actual adding of items, we use the current loop we are in to determin what place in the char array to access and what item number to add it as.
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
    XBee = new Serial(this, Serial.list()[portNumber], Integer.parseInt(baudrates[selectedBaudrate]));
    connectedSerial = true;
    XBee.bufferUntil('\n');
    XBee.write("G"); // Go
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
    XBee.stop();
    XBee.clear(); // Empty the buffer
    connectedSerial = false;
  }
  else
    println("Couldn't disconnect");
}
  static public void main(String args[]) {
    PApplet.main(new String[] { "--bgcolor=#FFFFFF", "PIDController" });
  }
}
