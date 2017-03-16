/**
 * @file holoneedleSensor.cpp
 * @brief Application for sensor acquisition for HoloNeedle project.
 * @author: Michael Lin, Jung Hwa Bae
 *
 */

// running mode
#define NEEDLE
#define HOLOLENS
//#define FBG_DEBUG
//#define FBG_DEBUG2
//#define FBG_DEBUG3
//#define SAVEDATA

#define FOURTH_POLY
//#define FIFTH_POLY

#define HOLOMAIN
//#define TEST

// timing //
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <sys/time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>
#include <unistd.h>
#include <inttypes.h>

// network sockets //
#include <sys/types.h> 
#include <sys/socket.h>
#include <netinet/in.h>

#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <fstream>
#include <sstream>
#include <cmath>

// interrogator reading //
// ******************** TCP Socket includes **********************//
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h> 
#include <iomanip>
// ******************** End TCP Socket includes **********************//


// timing 
#define RT_PRIORITY 	(49)
#define MAX_SAFE_STACK 	(8*1024)
#define NSEC_PER_SEC   	(1000000000)

// interrogator reading 
#define DATA_SIZE 146
#define DATA_LENGTH 10
#define PORT_NUMB "1852"
#define IP_ADD "10.0.0.126"

// ******************** namespaces **********************//
using namespace std;

typedef unsigned char uchar;

typedef struct data
{
    uchar header[88];
    float sensor_value[12];    
} data;


// ****************** Matrix Constants ****************** //
// Pseudo-inverse of curvature polynomials 
#ifdef FOURTH_POLY
const float pinvA[3][4] = {
    {0.000217013888889, -0.000217013888889, -0.000217013888889,  0.000217013888889},
    {-0.045189950980392,  0.035386029411765,  0.042738970588235, -0.032935049019608},
    {2.028033088235296, -0.645680147058825, -1.307444852941180,  0.925091911764709}};
#endif
#ifdef FIFTH_POLY
const float pinvA[4][3] = {
    {0.000014438849124, -0.000010704903652, 0.000001147678248},
    {-0.002598130538237, 0.001669326063148, -0.000096336506072},
    {0.098086714707812, -0.040364866387243, 0.001853487556038},
    {0.005867116833951, -0.002415066620459, 0.000110916098118} };
#endif
/*{
	{0.000217013888889,-0.000217013888889,-0.000217013888889,-1.307444852941180},
	{-0.045189950980392,0.035386029411765,0.042738970588235,-1.307444852941180},
	{2.028033088235296,-0.645680147058825,-1.307444852941180,-1.307444852941180} };*/

/*
    {0.000158, -0.000253, -0.000066, 0.000162},
	{-0.036307, 0.043784, 0.015760, -0.023237},
	{1.959151,  -1.061998, -0.519004, 0.621851} };*/


// Calibration matrices
/*
C1 =

   0.000035760969545  -0.001145790330845                   0
  -0.000999332265409   0.000504354184183                   0
   0.000960039673864   0.000633035440062                   0


C2 =

   0.000048663836659  -0.001262821225141                   0
  -0.001167266063624   0.000480820372396                   0
   0.001114451767319   0.000772431759602                   0


C3 =

   0.000008241967382  -0.002811119008254                   0
  -0.002359399497847   0.000773562937250                   0
   0.002342280765317   0.002015592530733                   0*/
const float C1[3][3] ={
   	{0.000204950507949441, 0.00121646436986137, 0},
  	{-0.000995644401992136, -0.000537433208215571, 0},
   	{0.00115328503288612, -0.000607264513515554, 0}};



const float C2[3][3] = {
   	{0.000279227750906997, 0.00168804977004751, 0},
  	{-0.000956617541852464, -0.000339882891191607, 0}, 
   	{0.00150548652510142, -0.000374190927814751, 0}};



const float C3[3][3] = {
   	{-0.000274220554606033, 0.00384379872980169, 0},
 	{-0.00265350320879051, 0.00008339400787445, 0},
   	{0.00229380432011879, -0.000955844410493895, 0}};


// ****************** End Matrix Constants ****************** //


/*float WL1, WL2, WL3, WL4, WL5, WL6, WL7, WL8, WL9, WL10, WL11, WL12;
float dWL1,dWL2,dWL3,dWL4,dWL5,dWL6,dWL7,dWL8,dWL9,dWL10,dWL11,dWL12;
float baseWL1, baseWL2, baseWL3, baseWL4, baseWL5, baseWL6, baseWL7, baseWL8, baseWL9, baseWL10, baseWL11, baseWL12;*/
float Fx, Fy, Fz;
float filtFx, filtFy, filtFz;
float y_n=0.0;
float y_n1=0.0;
float x_n=0.0;
float x_n1=0.0;

// *** Shape sensing coefficients *** //
float *est_xz_coeff;
float *est_yz_coeff;

// *************** Data saving buffers ******************* //
#ifdef SAVEDATA
float savedataFx[20000];
float savedataFy[20000];
float savedataFz[20000];
float savedataM1Encoder[20000];
float savedataM2Encoder[20000];
float saveangle[20000];
//float savedataF[20000];
float savedataDesired[20000];
#endif
// *** End Data saving buffers *** //


char *buffer;
char *buffer_size;
int sockfd, portno, n;
int i;
float *WLarray;
float *baseWL_array;

// *** TCP Connection declarations *** //
struct sockaddr_in serv_addr;
struct hostent *server;

int holoSocketfd, newHoloSocketfd;
struct hostent *holoserver;
// *** End TCP Connection declarations *** //

void signal_handler(int signum) {
    switch (signum) {
        case SIGINT:
           
            #ifdef SAVEDATA
      	       std::ofstream output_file;
               //output_file.open("freq_sweep3.txt",std::fstream::out);//0_418deg_moving test_pGain0_25
               //output_file.open("sensor_noise_characterization0.txt",std::fstream::out);
               output_file.open("FXZ_test.txt",std::fstream::out);
               //output_file<<std::fixed<<std::setprecision(5)<<"Time(ms)"<<'\t'<<"real position(deg)"<<std::endl;
               output_file<<std::fixed<<std::setprecision(5)<<"Time(ms)"<<'\t'<<"Force x"<<'\t'<<"Force y"<<'\t'<<"Force z"<<'\t'<<"motor x-axis pos"<<'\t'<<"motor z-axis pos"<<std::endl;
               for(int j=0;j<20000;j++){
                 // output_file<<j<<'\t'<<saveangle[j]<<std::endl;
	             output_file<<j<<'\t'<<savedataFx[j]<<'\t'<<savedataFy[j]<<'\t'<<savedataFz[j]<<'\t'<<savedataM1Encoder[j]<<'\t'<<savedataM2Encoder[j]<<std::endl;
               }
               output_file.close();
            #endif
            free(baseWL_array);
            free(WLarray);
            free(est_yz_coeff);
            free(est_xz_coeff);

            printf("Ending ...\n");
            close(holoSocketfd);
            close(sockfd);
            exit(-1);
            break;
    }
}

void stack_prefault(void)
{
    unsigned char dummy[MAX_SAFE_STACK];
    memset(dummy, 0, MAX_SAFE_STACK);
    return;
}

// interrogator functions
void error(const char *msg)
{
    perror(msg);
    exit(0);
}

float LPFilter(float x)
{
    x_n1=x;
    y_n1=-1*y_n+4.8948*x_n1-2.8948*x_n;
    x_n=x_n1;
    y_n=y_n1;
    return y_n1;
}

void initServer( int portnum ) // initialize server and listen for connection on portno
{
     socklen_t clilen;
     char holo_buffer[256];
     struct sockaddr_in holo_serv_addr, cli_addr;
     int n;
     holoSocketfd = socket(AF_INET, SOCK_STREAM, 0);
     if (holoSocketfd < 0) 
        error("ERROR opening socket");
     bzero((char *) &holo_serv_addr, sizeof(holo_serv_addr));
     holo_serv_addr.sin_family = AF_INET;
     holo_serv_addr.sin_addr.s_addr = INADDR_ANY;
     holo_serv_addr.sin_port = htons(portnum);
     if (bind(holoSocketfd, (struct sockaddr *) &holo_serv_addr,
              sizeof(holo_serv_addr)) < 0) 
              error("ERROR on binding");
     listen(holoSocketfd,5);
     printf("Listening on port %d\n", portnum);
     clilen = sizeof(cli_addr);
     newHoloSocketfd = accept(holoSocketfd, 
                 (struct sockaddr *) &cli_addr, 
                 &clilen);
     if (newHoloSocketfd < 0) 
          error("ERROR on accept");
     close(holoSocketfd);
     printf("Connected to new client. Ready to send data\n");
}


void initWL(){ // init the tcp communication

    portno = atoi(PORT_NUMB);
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) 
        error("ERROR opening socket");
    server = gethostbyname(IP_ADD);
    if (server == NULL) {
        fprintf(stderr,"ERROR, no such host\n");
        exit(0);
    }
    bzero((char *) &serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr, 
         (char *)&serv_addr.sin_addr.s_addr,
         server->h_length);
    serv_addr.sin_port = htons(portno);
    if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0) 
        error("ERROR connecting");
    cout << "Connected to interrogrator!" << endl;
}

int read_wavelength(float *inWLArray, int arrLen)
{   
    // already connected, send command to get data from interrogator

   // getting wavelength data automatically
    char get_data[21] = {'#','G','E','T','_','U','N','B','U','F','F','E','R','E','D','_','D','A','T','A','\n'};
   // char get_data[10] = {'#','G','E','T','_','D','A','T','A','\n'};  
    n = write(sockfd,get_data,strlen(get_data));
    if (n < 0) 
         error("ERROR writing to socket");
        
    // read first 10bytes data (including size information)
    buffer_size = (char *) malloc(sizeof(char)*DATA_LENGTH);
    n = read(sockfd,buffer_size,DATA_LENGTH);
    if (n < 0) 
         error("ERROR reading from socket");
    free(buffer_size);

    // read the data
    buffer = (char *) malloc(sizeof(char)*DATA_SIZE);
    n = read(sockfd,buffer,DATA_SIZE);
    if (n < 0) 
         error("ERROR reading from socket");

    char granularity[4];
    //granularity = (char*) malloc(sizeof(char)*4);

    for(int j=0; j<4; j++)
	granularity[j] = buffer[72+j];

    int *granularityf;
    granularityf = (int *)granularity;

    char peak_temp[4];
    int *peakf_temp;
    int wl_number=0;

    // Allocate it for size 4
    for(int h=0;h <= 44; h+=4) {
        for(int j=0; j<4; j++)
        {
            peak_temp[j] = buffer[j+88+h];
        }
        peakf_temp = (int *)peak_temp; // this one will also be dummy var
        //store result into wavelength array
        wl_number=h/4;
        inWLArray[wl_number] = ((float)*peakf_temp)/(*granularityf); /// this will be stored in a array of wavelengths allocated
    }
    //free(granularityf);
    free(buffer);

#ifdef FBG_DEBUG
   cout <<  *peakf1 << ", " << *peakf2 << ", " << *peakf3 << ", "
    << *peakf4 << ", " << *peakf5 << ", " << *peakf6 << ", "
    << *peakf7 << ", " << *peakf8 << ", " << *peakf9  << ", "
    << *peakf10 << ", " << *peakf11 << ", " << *peakf12 << endl;
#endif

#ifdef FBG_DEBUG3
   cout <<  WLarray << endl;
/*    << *peakf4 << ", " << *peakf5 << ", " << *peakf6 << ", "
    << *peakf7 << ", " << *peakf8 << ", " << *peakf9  << ", "
    << *peakf10 << ", " << *peakf11 << ", " << *peakf12 << endl;*/
#endif
   
    //close(sockfd);
    return 0;
}

void printWLs(float *inWLArray, int arrLen) {
    for (int i=0; i< arrLen; i++) {
        printf("%f,", inWLArray[i]);
    }
    printf("\n");
}


void get_baseWL(float *inWLArray, int arrLen,float *tempbaseWL_array){

   for (int i = 0; i<arrLen; i++)
   {
    	tempbaseWL_array[i] = inWLArray[i];
   }

/*   baseWL1=WL1;
   baseWL2=WL2;
   baseWL3=WL3;
   baseWL4=WL4;
   baseWL5=WL5;
   baseWL6=WL6;
   baseWL7=WL7;
   baseWL8=WL8;
   baseWL9=WL9;
   baseWL10=WL10;
   baseWL11=WL11;
   baseWL12=WL12;*/

   Fx=0.0;
   Fy=0.0;
   Fz=0.0;
}


void get_needle_shape(float *inWLArray, int arrLen,float *tempbaseWL_array, float *temp_est_yz_coeff, float *temp_est_xz_coeff){
//   float dWL1,dWL2,dWL3,dWL4,dWL5,dWL6,dWL7,dWL8,dWL9,dWL10,dWL11,dWL12;
  float *dWL; //TODO
  dWL = (float*) malloc(sizeof(float)*12); //TODO

  for(int i=0; i<arrLen; i++)
  {
	dWL[i]=inWLArray[i]-tempbaseWL_array[i];
  } 
 
/*  dWL1=(WL1-baseWL1);
  dWL2=(WL2-baseWL2);
  dWL3=(WL3-baseWL3);
  dWL4=(WL4-baseWL4);
  dWL5=(WL5-baseWL5);
  dWL6=(WL6-baseWL6);
  dWL7=(WL7-baseWL7);
  dWL8=(WL8-baseWL8);
  dWL9=(WL9-baseWL9);
  dWL10=(WL10-baseWL10);
  dWL11=(WL11-baseWL11);
  dWL12=(WL12-baseWL12); */
  
   // Find curvatures
  float curv_xz[4] = {
	dWL[1]*C1[0][0]+dWL[5]*C1[1][0]+dWL[9]*C1[2][0],
	dWL[2]*C2[0][0]+dWL[6]*C2[1][0]+dWL[10]*C2[2][0],
	dWL[3]*C3[0][0]+dWL[7]*C3[1][0]+dWL[11]*C3[2][0],
        0                                         };

  float curv_yz[4] = {
	dWL[1]*C1[0][1]+dWL[5]*C1[1][1]+dWL[9]*C1[2][1],
	dWL[2]*C2[0][1]+dWL[6]*C2[1][1]+dWL[10]*C2[2][1],
	dWL[3]*C3[0][1]+dWL[7]*C3[1][1]+dWL[11]*C3[2][1],
        0                                         };

  // Fit into polynomial using pinvA and find the polynomial coefficients
  temp_est_xz_coeff[0] = pinvA[0][0]*curv_xz[0] + pinvA[0][1]*curv_xz[1] + pinvA[0][2]*curv_xz[2] + pinvA[0][3]*curv_xz[3];
  temp_est_xz_coeff[1] = pinvA[1][0]*curv_xz[0] + pinvA[1][1]*curv_xz[1] + pinvA[1][2]*curv_xz[2] + pinvA[1][3]*curv_xz[3];
  temp_est_xz_coeff[2] = pinvA[2][0]*curv_xz[0] + pinvA[2][1]*curv_xz[1] + pinvA[2][2]*curv_xz[2] + pinvA[2][3]*curv_xz[3];
#ifdef FIFTH_POLY
  temp_est_xz_coeff[3] = pinvA[3][0]*curv_xz[0] + pinvA[3][1]*curv_xz[1] + pinvA[3][2]*curv_xz[2] + pinvA[3][3]*curv_xz[3];
#endif

  temp_est_yz_coeff[0] = pinvA[0][0]*curv_yz[0] + pinvA[0][1]*curv_yz[1] + pinvA[0][2]*curv_yz[2] + pinvA[0][3]*curv_yz[3];
  temp_est_yz_coeff[1] = pinvA[1][0]*curv_yz[0] + pinvA[1][1]*curv_yz[1] + pinvA[1][2]*curv_yz[2] + pinvA[1][3]*curv_yz[3];
  temp_est_yz_coeff[2] = pinvA[2][0]*curv_yz[0] + pinvA[2][1]*curv_yz[1] + pinvA[2][2]*curv_yz[2] + pinvA[2][3]*curv_yz[3];
#ifdef FIFTH_POLY
  temp_est_yz_coeff[3] = pinvA[3][0]*curv_yz[0] + pinvA[3][1]*curv_yz[1] + pinvA[3][2]*curv_yz[2] + pinvA[3][3]*curv_yz[3];
#endif
  
  free(dWL);
}


#ifdef HOLOMAIN
int main(int argc, char *argv[])//, int argc2, char **argv)
{

  // setup signal for ctrl-c
  signal(SIGINT, signal_handler);

  //variables to control motor
  unsigned counter = 0;

  //allocate wavelength array
  int length_array=12;
  WLarray = (float*) malloc(sizeof(float)*12); //TODO
  baseWL_array = (float*) malloc(sizeof(float)*12); //TODO
  #ifdef FOURTH_POLY
  est_xz_coeff = (float*) malloc(sizeof(float)*3); //TODO
  est_yz_coeff = (float*) malloc(sizeof(float)*3); //TODO
  #endif
  #ifdef FIFTH_POLY
  est_xz_coeff = (float*) malloc(sizeof(float)*4); //TODO
  est_yz_coeff = (float*) malloc(sizeof(float)*4); //TODO
  #endif

  // ******* Initialize interrogator functions ******* //
  //init to read wavelength data
  #ifdef NEEDLE
    initWL();
    read_wavelength(baseWL_array,length_array);
  #endif

  // ******* Initialize server for sending data to HoloLens ******* //
  #ifdef HOLOLENS
    initServer(20650);
  #endif


  // start after two second
  printf("Starting loop in 2 seconds ...\n");
  while(1) {
 
    // send wavelength data here to HoloLens through network
    if (counter % 10 == 0) { 
        int write_result;
        stringstream ss;
        ss.unsetf(ios::floatfield);
        ss.precision(8);
        ss << fixed;

    #ifdef FOURTH_POLY
        ss << (float)-1*est_xz_coeff[0]+1.0f <<"," 
           << (float)-1*est_xz_coeff[1]+1.0f <<","
           << (float)-1*est_xz_coeff[2]+1.0f <<","
           << (float)-1*est_yz_coeff[0]+1.0f <<","
           << (float)-1*est_yz_coeff[1]+1.0f <<","
           << (float)-1*est_yz_coeff[2]+1.0f << endl;
        string myString = ss.str();
        #ifdef HOLOLENS
            char *data2send; // calculate the length of buffer with 9*#floats-1
            data2send = (char *) malloc(sizeof(char) * (65));
            strncpy(data2send, myString.c_str(), sizeof(char)*65);
            printWLs(WLarray, length_array);
            write_result = write(newHoloSocketfd, data2send, 65);
            if (write_result < 0) error("ERROR writing to socket");
            free(data2send);
        #endif
    #endif

    #ifdef FIFTH_POLY
        ss << (float)-1*est_xz_coeff[0]+1.0f <<"," 
           << (float)-1*est_xz_coeff[1]+1.0f <<","
           << (float)-1*est_xz_coeff[2]+1.0f <<","
           << (float)-1*est_xz_coeff[3]+1.0f <<","
           << (float)est_yz_coeff[0]+1.0f <<","
           << (float)est_yz_coeff[1]+1.0f <<","
           << (float)est_yz_coeff[2]+1.0f <<","
           << (float)est_yz_coeff[3]+1.0f << endl;
        string myString = ss.str();
        #ifdef HOLOLENS
            char *data2send; // calculate the length of buffer with 9*#floats-1
            data2send = (char *) malloc(sizeof(char) * (87));
            strncpy(data2send, myString.c_str(), sizeof(char)*87);
            printWLs(WLarray, length_array);
           // write_result = write(newHoloSocketfd, data2send, 87);
            //if (write_result < 0) error("ERROR writing to socket");
            free(data2send);
        #endif
    #endif
        
    }
    counter ++;

    #ifdef NEEDLE 
    memset(WLarray, 0, sizeof(float)*12);
    #ifdef FOURTH_POLY
    memset(est_yz_coeff, 0, sizeof(float)*3);
    memset(est_xz_coeff, 0, sizeof(float)*3);
    #endif
    #ifdef FIFTH_POLY
    memset(est_yz_coeff, 0, sizeof(float)*4);
    memset(est_xz_coeff, 0, sizeof(float)*4);
    #endif
    read_wavelength(WLarray,length_array);
    // perform shape sensing algorithm
    get_needle_shape(WLarray,length_array,baseWL_array,est_yz_coeff,est_xz_coeff);
    #endif

    #ifdef SAVEDATA
      savedataFx[counter]=Fx;
      savedataFy[counter]=Fy;
      savedataFz[counter]=Fz;
      
    #endif

    // calculate next tick
//  t.tv_nsec += interval;
//  while (t.tv_nsec >= NSEC_PER_SEC)
//  {
//      t.tv_nsec -= NSEC_PER_SEC;
//      t.tv_sec++;
//  }
    
  }
}
#endif


#ifdef TEST
// here is were everything begins
int main(int argc, char* argv[]) {
  tagTracker tracker;

  // Setup tracker options
  tracker.setTagCodes("16h5");
  tracker.setup();
  tracker.m_draw = false;

  if (tracker.isVideo()) {
    cout << "Processing video" << endl;

    // setup image source, window for drawing, serial port...
    tracker.setupVideo();
  } else {
    return -1;
  }
  while (true) {
    // the actual processing loop where tags are detected and visualized
    //cout << hTn << endl;
    //cout << "-----------" << endl;
  
  }

  return 0;
}
#endif
