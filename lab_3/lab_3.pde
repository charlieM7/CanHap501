//Code based off of https://medium.com/@leekenjen/canhap-501-winter-2023-lab-3-communicate-something-dc0899f05e


/* library imports *****************************************************************************************************/ 
import processing.serial.*;
import static java.util.concurrent.TimeUnit.*;
import java.util.concurrent.*;
/* end library imports *************************************************************************************************/  


/* scheduler definition ************************************************************************************************/ 
private final ScheduledExecutorService scheduler      = Executors.newScheduledThreadPool(1);
/* end scheduler definition ********************************************************************************************/ 



/* device block definitions ********************************************************************************************/
Board             haplyBoard;
Device            widgetOne;
Mechanisms        pantograph;

byte              widgetOneID                         = 5;
int               CW                                  = 0;
int               CCW                                 = 1;
boolean           renderingForce                     = false;


int               hardwareVersion                     = 2;
/* end device block definition *****************************************************************************************/


/* framerate definition ************************************************************************************************/
long              baseFrameRate                       = 120;
/* end framerate definition ********************************************************************************************/ 

/* elements definition *************************************************************************************************/

/* Screen and world setup parameters */
float             pixelsPerMeter                      = 400.0;
float             radsPerDegree                       = 0.01745;

/* pantagraph link parameters in meters */
float             l                                   = 0.07;
float             L                                   = 0.09;

/* Forces and radial params */
PVector           actingForce                         = new PVector(0, 0);

float             min_x                               = -0.1;
float             max_x                               = 0.1;
float             min_y                               = 0.022;
float             max_y                               = 0.15;
PVector           center                              = new PVector(min_x + (max_x - min_x)/2, min_y + (max_y - min_y)/2);          

/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* device graphical position */
PVector           deviceOrigin                        = new PVector(0, 0);

/* World boundaries reference */
final int         worldPixelWidth                     = 1000;
final int         worldPixelHeight                    = 650;


/* graphical elements */
PShape pGraph, joint, endEffector;
PShape wall;

/* Lab3 variables */
PVector lastPosEE = new PVector(0,0);

enum FFNUM {
  ONE,
  TWO,
  THREE,
  FOUR,
  FIVE
};

FFNUM ffNum = FFNUM.ONE;
PFont f;

/* end elements definition *********************************************************************************************/  


/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 650);
  //size(worldPixelWidth,worldPixelHeight);
  
  /* set font type and size */
  f                   = createFont("Arial", 16, true);

  /* device setup */
  /**  
   * The board declaration needs to be changed depending on which USB serial port the Haply board is connected.
   * In the base example, a connection is setup to the first detected serial device, this parameter can be changed
   * to explicitly state the serial port will look like the following for different OS:
   *
   *      windows:      haplyBoard = new Board(this, "COM10", 0);
   *      linux:        haplyBoard = new Board(this, "/dev/ttyUSB0", 0);
   *      mac:          haplyBoard = new Board(this, "/dev/cu.usbmodem1411", 0);
   */
   
  haplyBoard          = new Board(this, "/dev/cu.usbmodem2101", 0);
  widgetOne           = new Device(widgetOneID, haplyBoard);
  pantograph          = new Pantograph();
  
  widgetOne.set_mechanism(pantograph);

  widgetOne.add_actuator(1, CCW, 2);
  widgetOne.add_actuator(2, CW, 1);
 
  widgetOne.add_encoder(1, CCW, 241, 10752, 2);
  widgetOne.add_encoder(2, CW, -61, 10752, 1);
  
  widgetOne.device_set_parameters();
  
  /* setup simulation thread to run at 1kHz */ 
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/
  
/* draw section ********************************************************************************************************/
void draw(){
  if(!renderingForce) {
    background(255);
    text("*Haptic Experience Window*", 100, 75);
    fill(#000000);
    text("Instructions:\nEach of the three experiences below represents a single word. \nAs you interact with each experience, try to guess what word they represent. \nMake sure the mouse is focussed on the Haptic Experience Window. \nPress 1 for the first experience. \nPress 2 for the second experience. \nPress 3 for the third mode. \n \nI would suggest pressing 1 when you want to end, then closing the window.", 100, 100);
    fill(#000000);
    text("Current mode:", 300, 300);
    fill(#000000);
    if(ffNum == FFNUM.ONE) {
      text("First mode", 500, 300);
      //fill(#000000);
    } else if(ffNum == FFNUM.TWO) {
      text("Second mode", 500, 300);
      fill(#000000);
    } else if(ffNum == FFNUM.THREE) {
      text("Third mode", 500, 300);
      fill(#000000);
    }else if(ffNum == FFNUM.FOUR){
      text("Fourth mode", 500, 300);
      fill(#000000);
    }else {
      text("Fifth mode", 500, 300);
      fill(#000000);
    }
  }
}
/* end draw section ****************************************************************************************************/


  
/* simulation section **************************************************************************************************/
class SimulationThread implements Runnable{
  
  public void run(){
    /* put haptic simulation code here, runs repeatedly at 1kHz as defined in setup */
    renderingForce = true;
    
    if(haplyBoard.data_available()){
      /* GET END-EFFECTOR STATE (TASK SPACE) */
      widgetOne.device_read_data();
    
      lastPosEE = posEE.copy();

      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      posEE.set(device_to_graphics(posEE)); 
      
      /* haptic wall force calculation */
      actingForce.set(0, 0);
            
/* JAGGED  **************************************************************************************************/      
      if(ffNum == FFNUM.ONE) {
        if(posEE.x > -0.06 && posEE.x <= -0.05){
          if(posEE.x > -0.06 && posEE.x <= -0.055) {
          fEE.set(-3000,0);
          }
          else{
          fEE.set(3000,0);
          }
        }
        else if(posEE.x > -0.04 && posEE.x <= -0.03){
          if(posEE.x > -0.04 && posEE.x <= -0.035) {
          fEE.set(-3000,0);
          }
          else{
          fEE.set(3000,0);
          }
        }
        else if(posEE.x > -0.01 && posEE.x <= 0){
          if(posEE.x > -0.01 && posEE.x <= -0.005) {
          fEE.set(-3000,0);
          }
          else{
          fEE.set(3000,0);
          }
        }
        else if(posEE.x > 0.01 && posEE.x <= 0.02){
          if(posEE.x > 0.01 && posEE.x <= 0.015) {
          fEE.set(-3000,0);
          }
          else{
          fEE.set(3000,0);
          }
        }
        else if(posEE.x > 0.03 && posEE.x <= 0.04){
          if(posEE.x > 0.03 && posEE.x <= 0.035) {
          fEE.set(-3000,0);
          }
          else{
          fEE.set(3000,0);
          }
        }
        else if(posEE.x > 0.05 && posEE.x <= 0.06){
          if(posEE.x > 0.03 && posEE.x <= 0.055) {
          fEE.set(-3000,0);
          }
          else{
          fEE.set(3000,0);
          }
        }
        else{
        fEE.set(0,0);
        }
      } 

/* LOVE  **************************************************************************************************/ 
    else if(ffNum == FFNUM.TWO){
      PVector dist = PVector.sub(posEE, center);
      float rad = 0.015;
      float force_scale = 8000;
      
      if(dist.mag() < rad){
        fEE.set(0,0);
      }
      else{
        float scale = (dist.mag() - rad)/dist.mag();
        fEE = PVector.mult(dist, -scale * force_scale);
      }
    }

 /* AVOIDANCE  **************************************************************************************************/ 
    else if(ffNum == FFNUM.THREE){
      PVector dist = PVector.sub(posEE, center);
      float max_force = 250;
      float rad = 0.05;
      
      if(dist.mag() > rad){
        fEE.set(0,0);
       }
      
      else{
        float scale = 1 - dist.mag()/rad;
        fEE = dist.copy();
        fEE.div(fEE.mag());
        double force_value = Math.pow(max_force * scale, 1.5);
        fEE.mult((float)force_value);
      }
    }
    
/* NOISE **************************************************************************************************/  
    else if(ffNum == FFNUM.FOUR){
      float scale = 10;
      float perlin_noise = noise(posEE.x, posEE.y);
      float magnitude = 5 + perlin_noise * scale + random(-2, 2); 
      float direction = perlin_noise * TWO_PI + random(-PI/4, PI/4);
      
      // Create a force vector 
      float forceX = magnitude * cos(direction);
      float forceY = magnitude * sin(direction);
      PVector forceVector = new PVector(forceX, forceY);
      println(forceVector);
      fEE.set(forceVector);
    }

/* SANDPAPER **************************************************************************************************/ 
    else{
      //if end effector moved
      if (posEE.dist(lastPosEE) > 0.0001) { 
        float scale = 10;
        float perlin_noise = noise(posEE.x, posEE.y);
        float magnitude = 5 + perlin_noise * scale + random(-2, 2);
        float direction = perlin_noise * TWO_PI + random(-PI/4, PI/4);
        
        // Create a force vector
        float forceX = magnitude * cos(direction);
        float forceY = magnitude * sin(direction);
        PVector forceVector = new PVector(forceX, forceY);
        println(forceVector);
        fEE.set(forceVector);
      }
      else{
        fEE.set(0,0);
      }
    }    
        
    fEE.set(graphics_to_device(fEE));
    /* end haptic wall force calculation */
    }
    
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
  
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/


/* helper functions section, place helper functions here ***************************************************************/
PVector device_to_graphics(PVector deviceFrame){
  return deviceFrame.set(-deviceFrame.x, deviceFrame.y);
}


PVector graphics_to_device(PVector graphicsFrame){
  return graphicsFrame.set(-graphicsFrame.x, graphicsFrame.y);
}

void keyPressed() {
  if(key == '1') {
    ffNum = FFNUM.ONE;
  } else if(key == '2') {
    ffNum = FFNUM.TWO;
  } else if(key == '3') {
    ffNum = FFNUM.THREE;
  } else if(key == '4') {
    ffNum = FFNUM.FOUR;
  } else if(key == '5'){
    ffNum = FFNUM.FIVE;
  }
}

/* end helper functions section ****************************************************************************************/
