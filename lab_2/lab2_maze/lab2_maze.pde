/**
 **********************************************************************************************************************
 * @file       Maze.pde
 * @author     Elie Hymowitz, Steve Ding, Colin Gallacher
 * @version    V4.0.0
 * @date       08-January-2021
 * @brief      Maze game example using 2-D physics engine
 **********************************************************************************************************************
 * @attention
 *
 *
 **********************************************************************************************************************
 */



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
float             pixelsPerCentimeter                 = 40.0;

/* generic data for a 2DOF device */
/* joint space */
PVector           angles                              = new PVector(0, 0);
PVector           torques                             = new PVector(0, 0);

/* task space */
PVector           posEE                               = new PVector(0, 0);
PVector           fEE                                 = new PVector(0, 0); 

/* World boundaries */
FWorld            world;
float             worldWidth                          = 25.0;  
float             worldHeight                         = 10.0; 

//float             worldWidth                          = 1000.0;  
//float             worldHeight                         = 650.0; 

//can probably remove this 
float             edgeTopLeftX                        = 0.0; 
float             edgeTopLeftY                        = 0.0; 
float             edgeBottomRightX                    = worldWidth; 
float             edgeBottomRightY                    = worldHeight;

float             gravityAcceleration                 = 980; //cm/s2
/* Initialization of virtual tool */
HVirtualCoupling  s;

//trap doors
boolean           trap1                           = false;
boolean           trap2                           = false;
boolean           isDone                           = false;

/* text font */
PFont             f;

/* define maze blocks */
FBox              b1;
FBox              b2;
FBox              b3;
FBox              b4;
FBox              b5;
FBox              b6;
FBox              b7;
FBox              b8;
FBox              b9;
FBox              b10;
FBox              b11;
FBox              b12;
FBox              b13;
FBox              b14;
FBox              b15;
FBox              b16;
FBox              b17;
FBox              b18;
FBox              b19;
FBox              b20;
FBox              b21;
FBox              b22;
FBox              b23;
FBox              b24;
FBox              b25;
FBox              b26;
FBox              b27;
FBox              b28;
FBox                g1;
FCircle             g2;
FCircle             g3;

//FBox[] boxes = {b1, b2};
//float[] size_x = {0.3, 4.0};
//float[] size_y = {3.0, 0.3};
//float[] pos_x = {worldWidth/2, worldWidth/2};
//float [] pos_y = {worldHeight/6 + 1.5, worldHeight/6};

//FBox[] boxes = {b1, b2, b3, b4, b5};
//float[] size_x = {0.3, 4.0, 1.5, 1.5, 1.5};
//float[] size_y = {3.0, 0.3, 0.3, 0.3, 0.3};
//float[] pos_x = {worldWidth/2, worldWidth/2, worldWidth/2, worldWidth/2, worldWidth/2};
//float [] pos_y = {worldHeight/6 + 1.5, worldHeight/6, worldHeight/6 + 3.0, worldHeight/6 + 3.0 + worldHeight/9,  worldHeight/6 + 3.0 + worldHeight/9};

/* end elements definition *********************************************************************************************/  



/* setup section *******************************************************************************************************/
void setup(){
  /* put setup code here, run once: */
  
  /* screen size definition */
  //size(1000, 650);
  size(1000, 400);
  
  /* set font type and size */
  f                   = createFont("Times", 16, true);
  
  
  
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
  pantograph          = new Pantograph(hardwareVersion);
  
  widgetOne.set_mechanism(pantograph);

  if(hardwareVersion == 2){
    widgetOne.add_actuator(1, CCW, 2);
    widgetOne.add_actuator(2, CW, 1);
 
    widgetOne.add_encoder(1, CCW, 241, 10752, 2);
    widgetOne.add_encoder(2, CW, -61, 10752, 1);
  }
  else if(hardwareVersion == 3){
    widgetOne.add_actuator(1, CCW, 2);
    widgetOne.add_actuator(2, CCW, 1);
 
    widgetOne.add_encoder(1, CCW, 168, 4880, 2);
    widgetOne.add_encoder(2, CCW, 12, 4880, 1); 
  }
  else{
    widgetOne.add_actuator(1, CCW, 2);
    widgetOne.add_actuator(2, CCW, 1);
 
    widgetOne.add_encoder(1, CCW, 168, 4880, 2);
    widgetOne.add_encoder(2, CCW, 12, 4880, 1); 
  }
  
  
  widgetOne.device_set_parameters();
  
  
  /* 2D physics scaling and world creation */
  hAPI_Fisica.init(this); 
  hAPI_Fisica.setScale(pixelsPerCentimeter); 
  world               = new FWorld();
  
  //start position
  b1                  = new FBox(4.0, 0.3);
  b1.setPosition(worldWidth/2, worldHeight/6 + 0.3); 
  b1.setFill(0);
  b1.setNoStroke();
  b1.setStaticBody(true);
  world.add(b1);
  
  //upper bar
  b2                  = new FBox(0.3, 2.7);
  b2.setPosition(worldWidth/2, worldHeight/6 + 1.5); 
  b2.setFill(0);
  b2.setNoStroke();
  b2.setStaticBody(true);
  world.add(b2);
  
  // box in middle
  b3                  = new FBox(1.5, 0.3);
  b3.setPosition(worldWidth/2, worldHeight/6 + 3.0); 
  b3.setFill(0);
  b3.setNoStroke();
  b3.setStaticBody(true);
  world.add(b3);
  
  b4                 = new FBox(0.4, 1.1);
  b4.setPosition(worldWidth/2 - 0.60, worldHeight/6 + 3.15 + worldHeight/9 - (worldHeight/9)/2); 
  b4.setFill(0);
  b4.setNoStroke();
  b4.setStaticBody(true);
  world.add(b4);
  
  b5                 = new FBox(0.4, 1.2);
  b5.setPosition(worldWidth/2 + 0.65, worldHeight/6 + 3.1 + worldHeight/9 - (worldHeight/9)/2); 
  b5.setFill(255);
  b5.setNoStroke();
  b5.setStaticBody(true);
  b5.setSensor(true);
  world.add(b5);
  
  b6                  = new FBox(1.58, 0.3);
  b6.setPosition(worldWidth/2, worldHeight/6 + 3.0 + worldHeight/9); 
  b6.setFill(0);
  b6.setNoStroke();
  b6.setStaticBody(true);
  world.add(b6);
  
  //lower bar
  b7                 = new FBox(0.3, 4.0);
  b7.setPosition(worldWidth/2, 3 * worldHeight/4 + 0.3); 
  b7.setFill(0);
  b7.setNoStroke();
  b7.setStaticBody(true);
  world.add(b7);
  
  //left side of maze
  b8                 = new FBox(9.0, 0.3);
  b8.setPosition(worldWidth/2 - 6.2, worldHeight/6 + 0.3); 
  b8.setFill(0);
  b8.setNoStroke();
  b8.setStaticBody(true);
  world.add(b8);
  
  b9                 = new FBox(0.3, 6.0);
  b9.setPosition(1.95, 3 * worldHeight/6 + 0.1); 
  b9.setFill(0);
  b9.setNoStroke();
  b9.setStaticBody(true);
  world.add(b9);
  
  b10                 = new FBox(9.0, 0.3);
  b10.setPosition(worldWidth/4 + 0.1, 5 * worldHeight/6 - 0.3); 
  b10.setFill(0);
  b10.setNoStroke();
  b10.setStaticBody(true);
  world.add(b10);
  
  b11                 = new FBox(9.0, 0.3);
  b11.setPosition(worldWidth/4 + 1.8, 4 * worldHeight/6); 
  b11.setFill(0);
  b11.setNoStroke();
  b11.setStaticBody(true);
  world.add(b11);
  
  b12                 = new FBox(0.3, 3.5);
  b12.setPosition(3.48, 3 * worldHeight/6 + 0.07); 
  b12.setFill(0);
  b12.setNoStroke();
  b12.setStaticBody(true);
  world.add(b12);
  
  b13                 = new FBox(7.5, 0.3);
  b13.setPosition(worldWidth/4 + 0.84, 2 * worldHeight/6); 
  b13.setFill(0);
  b13.setNoStroke();
  b13.setStaticBody(true);
  world.add(b13);
  
  b14                 = new FBox(6.7, 0.3);
  b14.setPosition(worldWidth/3 + 0.1, worldHeight/6 + 3.0); 
  b14.setFill(0);
  b14.setNoStroke();
  b14.setStaticBody(true);
  world.add(b14);
  
  //right side of maze
  b28                 = new FBox(2.0, 0.3);
  b28.setPosition(3 * worldWidth/4 - 0.4, worldHeight/6 + 0.3); 
  b28.setFill(255);
  b28.setNoStroke();
  b28.setStaticBody(true);
  b28.setSensor(true);
  world.add(b28);
  
  b15                 = new FBox(4.0, 0.3);
  b15.setPosition(5 * worldWidth/8, worldHeight/6 + 0.3); 
  b15.setFill(0);
  b15.setNoStroke();
  b15.setStaticBody(true);
  world.add(b15);
  
  b16                 = new FBox(5.5, 0.3);
  b16.setPosition(7 * worldWidth/8, worldHeight/6 + 0.3); 
  b16.setFill(0);
  b16.setNoStroke();
  b16.setStaticBody(true);
  world.add(b16);
  
  b17                 = new FBox(3.5, 0.3);
  b17.setPosition(2 * worldWidth/4 + 3.3, 2 * worldHeight/6); 
  b17.setFill(0);
  b17.setNoStroke();
  b17.setStaticBody(true);
  world.add(b17);
  
  b18                 = new FBox(0.3, 1.6);
  b18.setPosition(2 * worldWidth/4 + 5.0, 2 * worldHeight/6 - 0.65); 
  b18.setFill(0);
  b18.setNoStroke();
  b18.setStaticBody(true);
  world.add(b18);
  
  b19                 = new FBox(8.0, 0.3);
  b19.setPosition(2 * worldWidth/3 + 0.1, worldHeight/6 + 3.0); 
  b19.setFill(0);
  b19.setNoStroke();
  b19.setStaticBody(true);
  world.add(b19);
  
  b20                 = new FBox(0.3, 1.6);
  b20.setPosition(3 * worldWidth/4 + 0.50, 2 * worldHeight/6 - 0.65); 
  b20.setFill(0);
  b20.setNoStroke();
  b20.setStaticBody(true);
  world.add(b20);
  
  b21                 = new FBox(3.5, 0.3);
  b21.setPosition(7 * worldWidth/8 - 1.0, 2 * worldHeight/6); 
  b21.setFill(0);
  b21.setNoStroke();
  b21.setStaticBody(true);
  world.add(b21);
  
  b22                 = new FBox(0.3, 4.8);
  b22.setPosition(worldWidth - 2.5, 3 * worldHeight/6 + 0.65); 
  b22.setFill(0);
  b22.setNoStroke();
  b22.setStaticBody(true);
  world.add(b22);
  
  b23                 = new FBox(0.3, 3.6);
  b23.setPosition(2 * worldWidth/3 + 4.1, worldHeight/2 + 1.3); 
  b23.setFill(0);
  b23.setNoStroke();
  b23.setStaticBody(true);
  world.add(b23);
  
  b24                 = new FBox(0.3, 3.6);
  b24.setPosition(2 * worldWidth/3 + 2.5, worldHeight/2 + 3); 
  b24.setFill(0);
  b24.setNoStroke();
  b24.setStaticBody(true);
  world.add(b24);
  
  b25                 = new FBox(0.3, 3.6);
  b25.setPosition(2 * worldWidth/3 + 0.9, worldHeight/2 + 1.3); 
  b25.setFill(0);
  b25.setNoStroke();
  b25.setStaticBody(true);
  world.add(b25);
  
  b26                 = new FBox(0.3, 3.6);
  b26.setPosition(2 * worldWidth/3 - 0.7, worldHeight/2 + 3); 
  b26.setFill(0);
  b26.setNoStroke();
  b26.setStaticBody(true);
  world.add(b26);
  
  b27                 = new FBox(0.3, 3.6);
  b27.setPosition(2 * worldWidth/3 - 2.3, worldHeight/2 + 1.3); 
  b27.setFill(0);
  b27.setNoStroke();
  b27.setStaticBody(true);
  //b27.setSensor(false);
  world.add(b27);
  
  //End Position
  g1                  = new FBox(0.9,0.8);
  g1.setPosition(worldWidth/2, worldHeight/2 + 0.23);
  g1.setDensity(80);
  g1.setFill(0,255,0);
  g1.setNoStroke();
  g1.setStaticBody(true);
  g1.setSensor(true);
  g1.setName("Widget");
  world.add(g1);
  
  //secret button to close door
  g2                  = new FCircle(1.0);
  g2.setPosition(3 * worldWidth/4 - 0.35, worldHeight/2 + 2.0);
  g2.setDensity(80);
  g2.setFill(255);
  g2.setNoStroke();
  g2.setStaticBody(true);
  g2.setSensor(true);
  g2.setName("Widget");
  world.add(g2);
  
  //secret button to open door
  g3                  = new FCircle(1.0);
  g3.setPosition(5 * worldWidth/6 - 0.8, worldHeight/6 + 1.0);
  g3.setDensity(80);
  g3.setFill(255);
  g3.setNoStroke();
  g3.setStaticBody(true);
  g3.setSensor(true);
  g3.setName("Widget");
  world.add(g3);
  
  //for (int i=0; i<size_x.length; i++){
    
  //  System.out.println(i);
  //  //FBox box = boxes[i];
  //  //System.out.println(box);
  //  System.out.println(size_x[i]);
  //  boxes[i]                  = new FBox(size_x[i], size_y[0]);
  //  boxes[i].setPosition(pos_x[0], pos_y[0]); 
  //  boxes[i].setFill(0);
  //  boxes[i].setNoStroke();
  //  boxes[i].setStaticBody(true);
  //  world.add(boxes[i]);
  //}
  
  /* Setup the Virtual Coupling Contact Rendering Technique */
  s                   = new HVirtualCoupling((0.75)); 
  s.h_avatar.setDensity(4); 
  s.h_avatar.setFill(255,0,0); 
  s.h_avatar.setSensor(false);

  s.init(world, edgeTopLeftX+worldWidth/2, edgeTopLeftY+2); 
  
  /* World conditions setup */
  world.setGravity((0.0), gravityAcceleration); //1000 cm/(s^2)
  world.setEdges((edgeTopLeftX), (edgeTopLeftY), (edgeBottomRightX), (edgeBottomRightY)); 
  world.setEdgesRestitution(0.4);
  world.setEdgesFriction(0.5);

  world.draw();
  
  /* setup framerate speed */
  frameRate(baseFrameRate);
  
  
  /* setup simulation thread to run at 1kHz */
  SimulationThread st = new SimulationThread();
  scheduler.scheduleAtFixedRate(st, 1, 1, MILLISECONDS);
}
/* end setup section ***************************************************************************************************/



/* draw section ********************************************************************************************************/
void draw(){
  /* put graphical code here, runs repeatedly at defined framerate in setup, else default at 60fps: */
  if(renderingForce == false){
    background(255);
    textFont(f, 22);
    fill(0, 0, 0);
    
    if(trap1 && !trap2){
      textAlign(CENTER);
      text("Find the secret trap door to escape!", width/2, 60);
      b5.setFill(0);
      b28.setFill(0);
    }
      
    else if(trap1 && trap2 && !isDone){
      textAlign(CENTER);
      text("You're free, make it to the green square to win", width/2, 60);
      b4.setFill(255);
      b28.setFill(255);
    }
      
    else if(trap1 && trap2 && isDone){
      textAlign(CENTER);
      text("You've escaped! Congratulations", width/2, 60);
    }
      
    world.draw();
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
    
      angles.set(widgetOne.get_device_angles()); 
      posEE.set(widgetOne.get_device_position(angles.array()));
      
      if(hardwareVersion == 2){
        posEE.set(posEE.copy().mult(200));
      }
      else if(hardwareVersion == 3){
        posEE.set(posEE.copy().mult(150));
      }  
    }
    
    s.setToolPosition(edgeTopLeftX+worldWidth/2-(posEE).x, edgeTopLeftY+(posEE).y-7); 
    s.updateCouplingForce();
 
 
    fEE.set(-s.getVirtualCouplingForceX(), s.getVirtualCouplingForceY());
    fEE.div(100000); //dynes to newtons
    
    torques.set(widgetOne.set_device_torques(fEE.array()));
    widgetOne.device_write_torques();
    
    if (s.h_avatar.isTouchingBody(g2)){
      trap1 = true;
      b5.setSensor(false);
      b28.setSensor(false);
    }
    
    if (s.h_avatar.isTouchingBody(g3) && trap1){
      trap2 = true;
      b4.setSensor(true);
      b28.setSensor(true);
    }
    
    if (s.h_avatar.isTouchingBody(g1) && trap1){
      isDone = true;
    }
  
    world.step(1.0f/1000.0f);
  
    renderingForce = false;
  }
}
/* end simulation section **********************************************************************************************/
