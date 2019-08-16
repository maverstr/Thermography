//import the required libraries
import processing.serial.*;

Serial mySerial;
Table table;
String filename;



void setup()
{
  //set mySerial to listen on COM port 7 at 500000 baud
  mySerial = new Serial(this, "COM7", 500000);
  mySerial.clear();

  table = new Table();
  //add a column header "Data" for the collected data
  table.addColumn("Data");


  size(800, 800);
  fill(255, 0, 0);
  noStroke();

  //smooth();
}

void draw()
{
  int[] hist = new int[256];

  if (mySerial.available() > 0)
  {
    //set the value recieved as a String
    //String value = mySerial.readString();
    //String value = mySerial.readStringUntil(10); //Line feeding int
    String value = mySerial.readStringUntil(64); //"@"
    //check to make sure there is a value
    if (value != null)
    {
      //add a new row for each value
      value = value.substring(0, value.length()-1); //removes the last char of string
      int[] array = int(split(value, " "));
      for (int i = 0; i < 24; i++) {
        for (int x = 0; x < 32; x++) {
          try {          
            fill(array[32*i +x]);
            rect(i*20, x*20, 20, 20);
            hist[array[32*i+x]]++;
          }
          catch(Exception e) {
            println("error" + e);
          }
        }
      }
      int histMax = max(hist);
  fill(153);
  rect(500,0,300,800);
      stroke(255,0,0);

      // Draw half of the histogram (skip every second value)
      for (int i = 500; i < 800; i=i+1) {
        // Map i (from 0..img.width) to a location in the histogram (0..255)
        int which = int(map(i, 500, 800, 0, 255));
        // Convert the histogram value to a location between 
        // the bottom and the top of the picture
        int y = int(map(hist[which], 0, histMax, 600, 0));
        line(i, 0, i, y);
      }
    }
  }
}



void keyPressed()
{
  exit();
}
