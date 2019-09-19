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
  //background(0);

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
      TableRow newRow = table.addRow();
      //place the new row and value under the "Data" column
      newRow.setString("Data", value.substring(0, value.length()-1)); //removes the last char of string
      int[] array = int(split(value, " "));
      for (int i = 0; i < 24; i++) {
        for (int x = 0; x < 32; x++) {
          try {          
            fill((array[32*i +x] -65420)*2);
            rect(i*20, x*20, 20, 20);
          }
          catch(Exception e) {
            println("error");
          }
        }
      }
    }
  }
}



void keyPressed()
{
  //variables used for the filename timestamp
  int d = day();
  int m = month();
  int h = hour();
  int min = minute();
  int s = second();
  //variable as string under the data folder set as (mm-dd--hh-min-s.csv)
  filename = "data/" + str(m) + "-" + str(d) + "--" + str(h) + "-" + str(min) + "-" + str(s) + ".csv";
  //save as a table in csv format(data/table - data folder name table)
  saveTable(table, filename);
  exit();
}
