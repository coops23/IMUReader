
import processing.serial.*;
Serial myPort;

class StopWatchTimer {
  int startTime = 0, stopTime = 0;
  boolean running = false;  
    
    void start() {
        startTime = millis();
        running = true;
    }
    void stop() {
        stopTime = millis();
        running = false;
    }
    void clear() {
       startTime = 0; 
       stopTime = 0;
       running = false;
    }
    int getElapsedTime() {
        int elapsed;
        if (running) {
             elapsed = (millis() - startTime);
        }
        else {
            elapsed = (stopTime - startTime);
        }
        return elapsed;
    }
    int second() {
      return (getElapsedTime() / 1000) % 60;
    }
    int minute() {
      return (getElapsedTime() / (1000*60)) % 60;
    }
    int hour() {
      return (getElapsedTime() / (1000*60*60)) % 24;
    }
    
    boolean Running() {
      return running;
    }   
}

StopWatchTimer sw = new StopWatchTimer();

float G_TO_MS3_CONVERSION = 9806.65f;
float LSB_TO_G = 8192;

float qs = 0.0;
float qx = 0.0;
float qy = 0.0;
float qz = 0.0;
float delta_t = 0.0;
float[] ypr = {0,0,0,0};
float[] a = {0,0,0,0};
float[] v = {0,0,0,0};
float[] s = {0,0,0,0};
float[] vPrevious = {0,0,0,0};
float[] sPrevious = {0,0,0,0};

void setup()
{
  size(600, 500, P3D);

  myPort = new Serial(this, "COM3", 115200);                    // Windows
  //myPort = new Serial(this, "/dev/tty.usbmodem1965331", 115200);             // Linux
  //myPort = new Serial(this, "/dev/cu.usbmodem1217321", 9600);  // Mac

  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
  sw.start();
}

void draw()
{
  serialEvent();  // read and parse incoming serial message
  background(255); // set background to white
  lights();

  translate(width/2, height/2); // set position to centre

  pushMatrix(); // begin object

  float c1 = cos(radians(ypr[2]));
  float s1 = sin(radians(ypr[2]));
  float c2 = cos(radians(ypr[1]));
  float s2 = sin(radians(ypr[1]));
  float c3 = cos(radians(ypr[0]));
  float s3 = sin(radians(ypr[0]));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  drawArduino();

  popMatrix(); // end of object

  // Print values to console
  print(ypr[2]);
  print("\t");
  print(ypr[1]);
  print("\t");
  print(ypr[0]);
  print("\t");
  print(s[0]);
  print("\t");
  print(s[1]);
  print("\t");
  print(s[2]);
  print("\t");
  print(delta_t/1000);
  println();
  
  if(sw.getElapsedTime() > 10000 && sw.Running())
  {
     int i =0;
     for(i = 0; i< 3;i++)
     {
       vPrevious[i] = 0;
       sPrevious[i] = 0;
     }
     sw.clear();
     sw.start();
  }
}

void serialEvent()
{
  int newLine = 13; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    if (message != null) {
      String[] list = split(trim(message), " ");
      if (list.length >= 12 && list[0].equals("Info:")) {
        ypr[0] = float(list[1]);
        ypr[1] = float(list[2]);
        ypr[2] = float(list[3]); 
        a[0] = (float(list[4]) / LSB_TO_G) * G_TO_MS3_CONVERSION;
        a[1] = (float(list[5]) / LSB_TO_G) * G_TO_MS3_CONVERSION;
        a[2] = (float(list[6]) / LSB_TO_G) * G_TO_MS3_CONVERSION;
        qs = float(list[7]);
        qx = float(list[8]);
        qy = float(list[9]);
        qz = float(list[10]);
        delta_t = float(list[11]);
      }
    }
  } while (message != null);
}

void drawArduino()
{
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 100, 0);
  fill(0, 100, 0); 
  translate(100, 0, 0);
  box(200, 10, 10);
  
  stroke(100,0,0);
  fill(100, 0, 0); 
  
  translate(-100, 0,100);
  box(10, 10, 200);
  
  stroke(0,0,100); 
  fill(0,0,100);

  translate(0, -100, -100); // set position to edge of Arduino box
  box(10, 200, 10);
  
  stroke(0,100,100); 
  fill(0,100,100);
  
  translate(0, 100, 0);
  
  Deadreckoning(delta_t/1000);
  translate(s[1],-s[2],s[0]);
  box(30,30,30);
} //<>// //<>//

void Deadreckoning(float delta_t)
{
  int i = 0;
  
  for(i = 0; i < 3; i++)
  {   
     v[i] = (a[i] * delta_t)+ vPrevious[i];
     s[i] = (0.5 * a[i] * delta_t * delta_t) + (vPrevious[i] * delta_t) + sPrevious[i];
     vPrevious[i] = v[i];
     sPrevious[i] = s[i];
  }
}