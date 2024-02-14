
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


int               hardwareVersion                     = 3;
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


/* end effector radius in meters */
float             rEE                                 = 0.006;

/* virtual walls parameter  */ /*UPDATE AND CHANGE THESE FOR NEW SENSATIONS*/
float             kSluggish                               = 45000;
float             kWall                               = 80000;
PVector           fWall                               = new PVector(0, 0);
PVector           penSluggish                             = new PVector(0, 0);
PVector           penWall                             = new PVector(0, 0);
PVector           posWall                             = new PVector(0.01, 0.03);
float posWallBottom = .03;
float posWallLeft = -.01;
float posWallRight = .01;
boolean roboticLeft = true;

/* generic data for a 2DOF device */
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
  THREE
};
FFNUM ffNum = FFNUM.ONE;
PFont f;

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  size(1000, 400);
  
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
  haplyBoard          = new Board(this, Serial.list()[0], 0);
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
    text("Instructions:\nPress 1 for the first mode. \nBefore proceeding, please push the end effector back to its initial non-extended position. \nPress 2 for the second mode.\nThen, push the end effector back to its initial non-extended position. \nLastly, press 3 for the third mode. \nYou can press 1 again to go back to the first mode and stop the automatic motions in the third mode.", 100, 100);
    fill(#000000);
    text("Current mode:", 300, 300);
    fill(#000000);
    if(ffNum == FFNUM.ONE) {
      text("First mode", 500, 300);
      fill(#000000);
    } else if(ffNum == FFNUM.TWO) {
      text("Second mode", 500, 300);
      fill(#000000);
    } else {
      text("Third mode", 500, 300);
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
      fWall.set(0, 0);
/* EXPERIENCE 1  **************************************************************************************************/      
      if(ffNum == FFNUM.ONE) {
        if(lastPosEE.x < 0){
        println(lastPosEE.x);
        println(worldPixelWidth/2);
        penSluggish.set(lastPosEE);
        penSluggish.sub(posEE);
        fWall = fWall.add(penSluggish.mult(-kSluggish));    
        fEE = (fWall.copy()).mult(-1);
        } 
        else{
        println(lastPosEE.x);
        println(worldPixelWidth/2);
        penSluggish.set(lastPosEE);
        penSluggish.sub(posEE);
        fWall = fWall.add(penSluggish.mult(-kSluggish));    
        fEE = (fWall.copy()).mult(1);}
      }
/* EXPERIENCE 2  **************************************************************************************************/      
      else if(ffNum == FFNUM.TWO) {
        penWall.set(0, (posWallBottom - posEE.y));
        
        if(penWall.y < 0){
          fWall = fWall.add(penWall.mult(-kWall));  
        } 

        penWall.set(posEE.x - posWallLeft, 0);
      
        if(penWall.x < 0){
          fWall = fWall.add(penWall.mult(kWall));  
        } 
        
        penWall.set(posWallRight - posEE.x, 0);
      
        if(penWall.x < 0){
          fWall = fWall.add(penWall.mult(-kWall));  
        }
        
        fEE = (fWall.copy()).mult(-1);
      } 
/* EXPERIENCE 3  **************************************************************************************************/      
      else {
        if(posEE.y < .08) {
          fEE.set(0,5000);
        } else if (posEE.y > .081){
          fEE.set(0,-5000);
        } else {
          if(roboticLeft) {
            if(posEE.x > -.01) {
              fEE.set(-3000,0);    
            } else {
              roboticLeft = false;
            }
          } else {
            if(posEE.x < .01) {
              fEE.set(3000,0);    
            } else {
              roboticLeft = true;
            }
          }
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
  }
}

/* end helper functions section ****************************************************************************************/
