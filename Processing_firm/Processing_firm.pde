// 1 fingers measurement, 8bit, fast scan, 10x10
// 2020 Mar. 25 Hiroyuki Kajimoto

import processing.serial.*;

// The serial port:
Serial myPort;       
Serial harvest_myPort; //HARVEST
//user definitions
String COM_PORT="COM10"; //change this!
String COM_PORT_H="COM18";

//software definitions
final int PC_MBED_MEASURE_REQUEST=0xFE;
final int MBED_PC_MEASURE_RESULT=0xFF;
final int FINGER_CHANGE = 0xFD;
final byte VIB_END = (byte)0b11111110;

//Loop
int count = 0;
int count2 = 0;
int count2_ans = 0;
//graphical attributes
final int WINDOW_SIZE_X=1500;
final int WINDOW_SIZE_Y=1000;
final int SENSOR_X_NUM=10;
final int SENSOR_Y_NUM=10;
final int THERMAL_NUM=2;
final int FINGER_NUM=3;
final int PIXEL_SIZE_X=WINDOW_SIZE_X/3/(SENSOR_X_NUM+1);
final int PIXEL_SIZE_Y=WINDOW_SIZE_Y/2/(SENSOR_Y_NUM);
final float[] THERMAL_XCOORD = {5.0, 6.0}; 
final float[] THERMAL_YCOORD = {3.0, 7.0}; 
final float THERMAL_OFFSET = 28.0; // degree-C, Temperature of the room.
final float THERMAL_MAX = 35.0; // degree-C, Temperature of the room.
final float THERMAL_AMP = 30.0; 
int DrawMode = RECT;
int PressureRange  = 3;
int u_timer=0, prev_t=0;
boolean SerialDataSendRequest = false, SaveDataFlag= false;
PrintWriter output;

int[][][]PressureDistribution = new int[FINGER_NUM][SENSOR_X_NUM][SENSOR_Y_NUM];
float[][] ThermalDistribution = new float[FINGER_NUM][THERMAL_NUM];

//buffer
int[] power_b = new int[150];
int[] power_a = new int[150];
int FingerSelect = 0;
int esp = 0x00;


byte change_0 = 0x00;
void settings() {
  size(WINDOW_SIZE_X, WINDOW_SIZE_Y, P2D);
}


void setup() {  
  //serial setting
  myPort = new Serial(this, COM_PORT, 921600);
  harvest_myPort = new Serial(this, COM_PORT_H, 921600);
  //For mac users
  //myPort = new Serial(this, "/dev/tty.usbmodem1412", 921600);
  myPort.clear();
  harvest_myPort.clear();
  myPort.bufferUntil(MBED_PC_MEASURE_RESULT); 
//  myPort.buffer(FINGER_NUM*(SENSOR_X_NUM*SENSOR_Y_NUM+THERMAL_NUM)+2); 
  myPort.write(PC_MBED_MEASURE_REQUEST); //send initial request.
  textSize(32);
  noStroke();

  print("Welcome to sensor sample code.\n");
  print("1-4:sensing range\n");
  print("s:Start and Stop recording to CSV file\n");
  print("f:Change Finger.\n");
  //harvest_myPort.write((byte)0xaf);


}

void draw() { 
  int x, y, finger, xoffset;

  //clear screen
  background(20);
  count++;
  //ask mbed to send serial data if there is no data
  //  receiveData();
  for (finger=0; finger<FINGER_NUM; finger++) {
    xoffset =  (SENSOR_X_NUM+1)*PIXEL_SIZE_X*finger;
    for (x=0; x<SENSOR_X_NUM; x++) {
      for (y=0; y<SENSOR_Y_NUM; y++) {
        fill(20, PressureDistribution[finger][x][y], 20);
        rect((SENSOR_X_NUM-x)*PIXEL_SIZE_X + xoffset, y*PIXEL_SIZE_Y, PIXEL_SIZE_X, PIXEL_SIZE_Y);
      }
    }
    for (x=0; x<THERMAL_NUM; x++) {
      fill((int)((ThermalDistribution[finger][x]-THERMAL_OFFSET)*THERMAL_AMP), 20, (THERMAL_MAX - ThermalDistribution[finger][x])*THERMAL_AMP);
      rect((int)(THERMAL_XCOORD[x]*(float)PIXEL_SIZE_X) + xoffset, WINDOW_SIZE_Y/2 + (int)(THERMAL_YCOORD[x]*(float)PIXEL_SIZE_Y), PIXEL_SIZE_X, PIXEL_SIZE_Y);
      fill(0, 100, 150);
      text((int)ThermalDistribution[finger][x], (int)(THERMAL_XCOORD[x]*(float)PIXEL_SIZE_X) + xoffset, WINDOW_SIZE_Y/2 + (int)(THERMAL_YCOORD[x]*(float)PIXEL_SIZE_Y));
    }
  }
    

}

//Request and receive serial data.
//Returns -1 if the data is not prepared.
//Returns 8 bit time in milliseconds, measured in mbed. 

void serialEvent(Serial mp)
{
  int rcv, x, y, finger, t;
  int power_num = 0;
  //send next request. This request is issued "before" reading serial buffer, to save time of ESP32.

  myPort.write(PC_MBED_MEASURE_REQUEST); 

  //data reading.This is the data assoc  iated with previous request.
  for (x = 0; x<SENSOR_X_NUM; x++) {
    for (y = 0; y<SENSOR_Y_NUM; y++) {
      PressureDistribution[FingerSelect][x][y]= myPort.read(); //upper 8bits
      power_b[power_num] = PressureDistribution[FingerSelect][x][y] / 150; //buffer data
      power_num++;
    }
  }

  for (x = 0; x<THERMAL_NUM; x++) {
    rcv = myPort.read();
    ThermalDistribution[FingerSelect][x] =0.248 * rcv - 16.557; //Calculation by theoretical formula.
  }

  //timer
  t = myPort.read(); //time (0-255) in milliseconds (mbed)
  if(t != -1){  //t==-1 means insufficient number of data in buffer.
    if(t>=prev_t){
      u_timer = u_timer + t - prev_t;
    }else{ //overflow management
      u_timer = u_timer + t + 255  - prev_t;
    }
    prev_t = t;

    //remove the terminating character
    if(myPort.read()!=MBED_PC_MEASURE_RESULT){
      myPort.clear(); 
    } 

    power_a[0] = (byte)((power_b[1] << 4) + (power_b[2]));
    power_a[1] = (byte)((power_b[3] << 4) + (power_b[4]));
    power_a[2] = (byte)((power_b[5] << 4) + (power_b[6]));
    power_a[3] = (byte)((power_b[7] << 4) + (power_b[8]));
    
    power_a[4] = (byte)((power_b[11] << 4) + (power_b[12]));
    power_a[5] = (byte)((power_b[13] << 4) + (power_b[14]));
    power_a[6] = (byte)((power_b[15] << 4) + (power_b[16]));
    power_a[7] = (byte)((power_b[17] << 4) + (power_b[18]));
    
    power_a[8] = (byte)((power_b[21] << 4) + (power_b[22]));
    power_a[9] = (byte)((power_b[23] << 4) + (power_b[24]));
    power_a[10] = (byte)((power_b[25] << 4) + (power_b[26]));
    power_a[11] = (byte)((power_b[27] << 4) + (power_b[28]));
    
    
    power_a[12] = (byte)((power_b[31] << 4) + (power_b[32]));
    power_a[13] = (byte)((power_b[33] << 4) + (power_b[34]));
    power_a[14] = (byte)((power_b[35] << 4) + (power_b[36]));
    power_a[15] = (byte)((power_b[37] << 4) + (power_b[38]));
    
    power_a[16] = (byte)((power_b[41] << 4) + (power_b[42]));
    power_a[17] = (byte)((power_b[43] << 4) + (power_b[44]));
    power_a[18] = (byte)((power_b[45] << 4) + (power_b[46]));
    power_a[19] = (byte)((power_b[47] << 4) + (power_b[48]));
    
    power_a[20] = (byte)((power_b[51] << 4) + (power_b[52]));
    power_a[21] = (byte)((power_b[53] << 4) + (power_b[54]));
    power_a[22] = (byte)((power_b[55] << 4) + (power_b[56]));
    power_a[23] = (byte)((power_b[57] << 4) + (power_b[58]));
    
    
    power_a[24] = (byte)((power_b[61] << 4) + (power_b[62]));
    power_a[25] = (byte)((power_b[63] << 4) + (power_b[64]));
    power_a[26] = (byte)((power_b[65] << 4) + (power_b[66]));
    power_a[27] = (byte)((power_b[67] << 4) + (power_b[68]));
    
    power_a[28] = (byte)((power_b[71] << 4) + (power_b[72]));
    power_a[29] = (byte)((power_b[73] << 4) + (power_b[74]));
    power_a[30] = (byte)((power_b[75] << 4) + (power_b[76]));
    power_a[31] = (byte)((power_b[77] << 4) + (power_b[78]));
    
    power_a[32] = (byte)((power_b[81] << 4) + (power_b[82]));
    power_a[33] = (byte)((power_b[83] << 4) + (power_b[84]));
    power_a[34] = (byte)((power_b[85] << 4) + (power_b[86]));
    power_a[35] = (byte)((power_b[87] << 4) + (power_b[88]));
    
    power_a[36] = (byte)((power_b[91] << 4) + (power_b[92]));
    power_a[37] = (byte)((power_b[93] << 4) + (power_b[94]));
    power_a[38] = (byte)((power_b[95] << 4) + (power_b[96]));
    power_a[39] = (byte)((power_b[97] << 4) + (power_b[98]));
 

 for(int out_a = 0; out_a < 40; out_a++){
       //harvest_myPort.write((byte)power_a[out_a]);
       harvest_myPort.write(0xff);
    }

    //if(p == harvest_myPort){

    //}    

    //println(count2_ans);
    //0.1sec
 /*
 if( count > 8 )
    {
      count = 0;
      count2_ans = count2 % 8;
    
      if(count2_ans == 0)
      {
      power_a[0] = (byte)(power_a[0] & 0xF0);
      power_a[4] = (byte)(power_a[4] & 0xF0);
      power_a[8] = (byte)(power_a[8] & 0xF0);
      
      power_a[12] = (byte)(power_a[12] & 0xF0);
      power_a[16] = (byte)(power_a[16] & 0xF0);
      power_a[20] = (byte)(power_a[20] & 0xF0);
      power_a[24] = (byte)(power_a[24] & 0xF0);
      power_a[28] = (byte)(power_a[28] & 0xF0);
      power_a[32] = (byte)(power_a[32] & 0xF0);
      power_a[36] = (byte)(power_a[36] & 0xF0);
      
      power_a[1] = 0;
      power_a[2] = 0;
      power_a[3] = 0;
      power_a[5] = 0;
      power_a[6] = 0;
      power_a[7] = 0;
      power_a[9] = 0;
      power_a[10] = 0;
      power_a[11] = 0;
      
      power_a[13] = 0;
      power_a[14] = 0;
      power_a[15] = 0;
      power_a[17] = 0;
      power_a[18] = 0;
      power_a[19] = 0;
      power_a[21] = 0;
      power_a[22] = 0;
      power_a[23] = 0;
      
      power_a[25] = 0;
      power_a[26] = 0;
      power_a[27] = 0;
      power_a[29] = 0;
      power_a[30] = 0;
      power_a[31] = 0;
      power_a[33] = 0;
      power_a[34] = 0;
      power_a[35] = 0;
      
      power_a[37] = 0;
      power_a[38] = 0;
      power_a[39] = 0;
      
      }else if(count2_ans == 1)
      {
      power_a[0] = (byte)(power_a[0] & 0x0F);
      power_a[4] = (byte)(power_a[4] & 0x0F);
      power_a[8] = (byte)(power_a[8] & 0x0F);
      
      power_a[12] = (byte)(power_a[12] & 0x0F);
      power_a[16] = (byte)(power_a[16] & 0x0F);
      power_a[20] = (byte)(power_a[20] & 0x0F);
      power_a[24] = (byte)(power_a[24] & 0x0F);
      power_a[28] = (byte)(power_a[28] & 0x0F);
      power_a[32] = (byte)(power_a[32] & 0x0F);
      power_a[36] = (byte)(power_a[36] & 0x0F);
      
      power_a[1] = 0;
      power_a[2] = 0;
      power_a[3] = 0;
      power_a[5] = 0;
      power_a[6] = 0;
      power_a[7] = 0;
      power_a[9] = 0;
      power_a[10] = 0;
      power_a[11] = 0;
      
      power_a[13] = 0;
      power_a[14] = 0;
      power_a[15] = 0;
      power_a[17] = 0;
      power_a[18] = 0;
      power_a[19] = 0;
      power_a[21] = 0;
      power_a[22] = 0;
      power_a[23] = 0;
      
      power_a[25] = 0;
      power_a[26] = 0;
      power_a[27] = 0;
      power_a[29] = 0;
      power_a[30] = 0;
      power_a[31] = 0;
      power_a[33] = 0;
      power_a[34] = 0;
      power_a[35] = 0;
      
      power_a[37] = 0;
      power_a[38] = 0;
      power_a[39] = 0;
      
      }else if(count2_ans == 2)
      {
      power_a[1] = (byte)(power_a[1] & 0xF0);
      power_a[5] = (byte)(power_a[5] & 0xF0);
      power_a[9] = (byte)(power_a[9] & 0xF0);

      power_a[13] = (byte)(power_a[13] & 0xF0);
      power_a[17] = (byte)(power_a[17] & 0xF0);
      power_a[21] = (byte)(power_a[21] & 0xF0);
      power_a[25] = (byte)(power_a[25] & 0xF0);
      power_a[29] = (byte)(power_a[29] & 0xF0);
      power_a[33] = (byte)(power_a[33] & 0xF0);
      power_a[37] = (byte)(power_a[37] & 0xF0);
      
      power_a[0] = 0;
      power_a[2] = 0;
      power_a[3] = 0;
      power_a[4] = 0;
      power_a[6] = 0;
      power_a[7] = 0;
      power_a[8] = 0;
      power_a[10] = 0;
      power_a[11] = 0;
      
      power_a[12] = 0;
      power_a[14] = 0;
      power_a[15] = 0;
      power_a[16] = 0;
      power_a[18] = 0;
      power_a[19] = 0;
      power_a[20] = 0;
      power_a[22] = 0;
      power_a[23] = 0;
      
      power_a[24] = 0;
      power_a[26] = 0;
      power_a[27] = 0;
      power_a[28] = 0;
      power_a[30] = 0;
      power_a[31] = 0;
      power_a[32] = 0;
      power_a[34] = 0;
      power_a[35] = 0;
      
      power_a[36] = 0;
      power_a[38] = 0;
      power_a[39] = 0;
      
      }else if(count2_ans == 3)
      {
      power_a[1] = (byte)(power_a[1] & 0x0F);
      power_a[5] = (byte)(power_a[5] & 0x0F);
      power_a[9] = (byte)(power_a[9] & 0x0F);
      
      power_a[13] = (byte)(power_a[13] & 0x0F);
      power_a[17] = (byte)(power_a[17] & 0x0F);
      power_a[21] = (byte)(power_a[21] & 0x0F);
      power_a[25] = (byte)(power_a[25] & 0x0F);
      power_a[29] = (byte)(power_a[29] & 0x0F);
      power_a[33] = (byte)(power_a[33] & 0x0F);
      power_a[37] = (byte)(power_a[37] & 0x0F);
      
      power_a[0] = 0;
      power_a[2] = 0;
      power_a[3] = 0;
      power_a[4] = 0;
      power_a[6] = 0;
      power_a[7] = 0;
      power_a[8] = 0;
      power_a[10] = 0;
      power_a[11] = 0;
      
      power_a[12] = 0;
      power_a[14] = 0;
      power_a[15] = 0;
      power_a[16] = 0;
      power_a[18] = 0;
      power_a[19] = 0;
      power_a[20] = 0;
      power_a[22] = 0;
      power_a[23] = 0;
      
      power_a[24] = 0;
      power_a[26] = 0;
      power_a[27] = 0;
      power_a[28] = 0;
      power_a[30] = 0;
      power_a[31] = 0;
      power_a[32] = 0;
      power_a[34] = 0;
      power_a[35] = 0;
      
      power_a[36] = 0;
      power_a[38] = 0;
      power_a[39] = 0;
      
      }else if(count2_ans == 4)
      {
      power_a[2] = (byte)(power_a[2] & 0xF0);
      power_a[6] = (byte)(power_a[6] & 0xF0);
      power_a[10] = (byte)(power_a[10] & 0xF0);
      
      power_a[14] = (byte)(power_a[14] & 0xF0);
      power_a[18] = (byte)(power_a[18] & 0xF0);
      power_a[22] = (byte)(power_a[22] & 0xF0);
      power_a[26] = (byte)(power_a[26] & 0xF0);
      power_a[30] = (byte)(power_a[30] & 0xF0);
      power_a[34] = (byte)(power_a[34] & 0xF0);
      power_a[38] = (byte)(power_a[38] & 0xF0);
      
      power_a[0] = 0;
      power_a[1] = 0;
      power_a[3] = 0;
      power_a[4] = 0;
      power_a[5] = 0;
      power_a[7] = 0;
      power_a[8] = 0;
      power_a[9] = 0;
      power_a[11] = 0;    
      
      power_a[12] = 0;
      power_a[13] = 0;
      power_a[15] = 0;
      power_a[16] = 0;
      power_a[17] = 0;
      power_a[19] = 0;
      power_a[20] = 0;
      power_a[21] = 0;
      power_a[23] = 0;
      
      power_a[24] = 0;
      power_a[25] = 0;
      power_a[27] = 0;
      power_a[28] = 0;
      power_a[29] = 0;
      power_a[31] = 0;
      power_a[32] = 0;
      power_a[33] = 0;
      power_a[35] = 0;
      
      power_a[36] = 0;
      power_a[37] = 0;
      power_a[39] = 0;
      
      }else if(count2_ans == 5)
      {
      power_a[2] = (byte)(power_a[2] & 0x0F);
      power_a[6] = (byte)(power_a[6] & 0x0F);
      power_a[10] = (byte)(power_a[10] & 0x0F);
      
      power_a[14] = (byte)(power_a[14] & 0x0F);
      power_a[18] = (byte)(power_a[18] & 0x0F);
      power_a[22] = (byte)(power_a[22] & 0x0F);
      power_a[26] = (byte)(power_a[26] & 0x0F);
      power_a[30] = (byte)(power_a[30] & 0x0F);
      power_a[34] = (byte)(power_a[34] & 0x0F);
      power_a[38] = (byte)(power_a[38] & 0x0F);
      
      power_a[0] = 0;
      power_a[1] = 0;
      power_a[3] = 0;
      power_a[4] = 0;
      power_a[5] = 0;
      power_a[7] = 0;
      power_a[8] = 0;
      power_a[9] = 0;
      power_a[11] = 0;
      
      power_a[12] = 0;
      power_a[13] = 0;
      power_a[15] = 0;
      power_a[16] = 0;
      power_a[17] = 0;
      power_a[19] = 0;
      power_a[20] = 0;
      power_a[21] = 0;
      power_a[23] = 0;
      
      power_a[24] = 0;
      power_a[25] = 0;
      power_a[27] = 0;
      power_a[28] = 0;
      power_a[29] = 0;
      power_a[31] = 0;
      power_a[32] = 0;
      power_a[33] = 0;
      power_a[35] = 0;
      
      power_a[36] = 0;
      power_a[37] = 0;
      power_a[39] = 0;
      
      }else if(count2_ans == 6)
      {
      power_a[3] = (byte)(power_a[3] & 0xF0);
      power_a[7] = (byte)(power_a[7] & 0xF0);
      power_a[11] = (byte)(power_a[11] & 0xF0);
      
      power_a[15] = (byte)(power_a[15] & 0xF0);
      power_a[19] = (byte)(power_a[19] & 0xF0);
      power_a[23] = (byte)(power_a[23] & 0xF0);
      power_a[27] = (byte)(power_a[27] & 0xF0);
      power_a[31] = (byte)(power_a[31] & 0xF0);
      power_a[35] = (byte)(power_a[35] & 0xF0);
      power_a[39] = (byte)(power_a[39] & 0xF0);
      
      power_a[0] = 0;
      power_a[1] = 0;
      power_a[2] = 0;
      power_a[4] = 0;
      power_a[5] = 0;
      power_a[6] = 0;
      power_a[8] = 0;
      power_a[9] = 0;
      power_a[10] = 0;
      
      power_a[12] = 0;
      power_a[13] = 0;
      power_a[14] = 0;
      power_a[16] = 0;
      power_a[17] = 0;
      power_a[18] = 0;
      power_a[20] = 0;
      power_a[21] = 0;
      power_a[22] = 0;
      
      power_a[24] = 0;
      power_a[25] = 0;
      power_a[26] = 0;
      power_a[28] = 0;
      power_a[29] = 0;
      power_a[30] = 0;
      power_a[32] = 0;
      power_a[33] = 0;
      power_a[34] = 0;
      
      power_a[36] = 0;
      power_a[37] = 0;
      power_a[38] = 0;
      
      }else if(count2_ans == 7)
      {
      power_a[3] = (byte)(power_a[3] & 0x0F);
      power_a[7] = (byte)(power_a[7] & 0x0F);
      power_a[11] = (byte)(power_a[11] & 0x0F);
      
      
      power_a[15] = (byte)(power_a[15] & 0x0F);
      power_a[19] = (byte)(power_a[19] & 0x0F);
      power_a[23] = (byte)(power_a[23] & 0x0F);
      power_a[27] = (byte)(power_a[27] & 0x0F);
      power_a[31] = (byte)(power_a[31] & 0x0F);
      power_a[35] = (byte)(power_a[35] & 0x0F);
      power_a[39] = (byte)(power_a[39] & 0x0F);
      
      power_a[0] = 0;
      power_a[1] = 0;
      power_a[2] = 0;
      power_a[4] = 0;
      power_a[5] = 0;
      power_a[6] = 0;
      power_a[8] = 0;
      power_a[9] = 0;
      power_a[10] = 0;     
      
      
      power_a[12] = 0;
      power_a[13] = 0;
      power_a[14] = 0;
      power_a[16] = 0;
      power_a[17] = 0;
      power_a[18] = 0;
      power_a[20] = 0;
      power_a[21] = 0;
      power_a[22] = 0;
      
      power_a[24] = 0;
      power_a[25] = 0;
      power_a[26] = 0;
      power_a[28] = 0;
      power_a[29] = 0;
      power_a[30] = 0;
      power_a[32] = 0;
      power_a[33] = 0;
      power_a[34] = 0;
      
      power_a[36] = 0;
      power_a[37] = 0;
      power_a[38] = 0;
      
      }
      count2++;
      if(count2 == 8) count2 = 0;
   
    for(int out_a = 0; out_a < 40; out_a++){
    harvest_myPort.write((byte)power_a[out_a]);
    }

  }
*/
 /*
    //harvest_myPort.read();


    /*
    harvest_myPort.write((byte)power_b[11]);
    harvest_myPort.write((byte)power_b[12]);
    harvest_myPort.write((byte)power_b[13]);
    harvest_myPort.write((byte)power_b[14]);
    harvest_myPort.write((byte)power_b[15]);
    harvest_myPort.write((byte)power_b[16]);
    harvest_myPort.write((byte)power_b[17]);
    harvest_myPort.write((byte)power_b[18]);
    */
    //harvest_myPort.write((byte)0xff);

  // println(0xff);
  //for(int n_x = 1; n_x < 9; n_x++){

    //harvest_myPort.write(VIB_END);
  //}
  
  
    if(SaveDataFlag == true){
      output.print((float)u_timer/10.0 + ",");
      //int m = millis(); output.print(m+","); //Check with PC timer
      for (x = 0; x<SENSOR_X_NUM; x++) {
        for (y = 0; y<SENSOR_Y_NUM; y++) {
          output.print(PressureDistribution[FingerSelect][x][y]+",");
        }
      }
      for (x = 0; x<THERMAL_NUM; x++) {
          output.print(ThermalDistribution[FingerSelect][x]+",");
      }
      output.println();
    }
  }
}


void keyTyped() {
  int finger, x,y;

  switch (key) {
    case '1':  myPort.write(1); print("Pressure Range was set to 1\n"); break; 
    case '2':  myPort.write(2); print("Pressure Range was set to 2\n"); break; 
    case '3':  myPort.write(3); print("Pressure Range was set to 3\n"); break; 
    case '4':  myPort.write(4); print("Pressure Range was set to 4\n"); break; 
    case '5':  break; 

    case 'f':  
      myPort.write(FINGER_CHANGE); 
      FingerSelect = (FingerSelect+1)%FINGER_NUM;
      println("Finger was changed to"+FingerSelect); 
      break; 
    case 's':  
      if(SaveDataFlag == false){
        String filename = nf(year(), 2) + nf(month(), 2) + nf(day(), 2) +"-"+ nf(hour(), 2) + nf(minute(), 2) + nf(second(), 2) + ".csv";
        output = createWriter(filename); 
        output.print("time(ms),");
        for (x = 0; x<SENSOR_X_NUM; x++) {
          for (y = 0; y<SENSOR_Y_NUM; y++) {
            output.print("P"+FingerSelect+x+y+",");
          }
        }
        for (x = 0; x<THERMAL_NUM; x++) {
            output.print("T"+FingerSelect+x+",");
        }
        output.println();
        SaveDataFlag = true;
        u_timer=0;
        prev_t=0;
      }else{
        SaveDataFlag=false;
        output.close();
      }
      break;
    default : 
    break;
  }
}
