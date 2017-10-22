/**
 * @file holoneedleSensor.cpp
 * @brief Application for sensor acquisition for HoloNeedle project.
 * @author: Michael Lin, Jung Hwa Bae
 *
 */

// *** Macro parameters for activating/inactivating different components *** //
#define CONNECT2NEEDLE
#define CONNECT2HOLOLENS

// *** Macro parameters for activating/inactivating different test prints *** //
// Following for printing FBG values a loop rate
//#define FBG_DEBUG_PRINT

// *** Macro parameters for choosing polynomial interpolation mode *** //
#define FOURTH_POLY
//#define FIFTH_POLY

// *** Macro parameters for interrogator readings *** //
#define DATA_SIZE 146
#define DATA_LENGTH 10
#define PORT_NUMB "1852"
#define IP_ADD "10.0.0.126"

// timing 
#define RT_PRIORITY     (49)
#define MAX_SAFE_STACK  (8*1024)
#define NSEC_PER_SEC    (1000000000)


// *** General includes *** //
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdarg.h>
#include <iostream>
#include <sstream>

// *** Includes for sockets *** //
#include <sys/types.h> 
#include <sys/socket.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sched.h>
#include <netinet/in.h>
#include <netdb.h> 

// *** Namespace definitions *** //
using namespace std;


// *** Typedefs *** //
typedef unsigned char uchar;
typedef unsigned char byte;
// struct definition for saving sensor data
typedef struct data
{
    uchar header[88];
    float sensor_value[12];    
} data;

// *** Matrix Constants ***//
// Pseudo-inverse matrix for curvature polynomials 
#ifdef FOURTH_POLY
const float pinvA[3][4] = {
    {0.000196670987170, -0.000185506596030, -0.000215754756108,  0.000204590364968},
    {-0.040712424136224,  0.029143140319940,  0.041572491743759, -0.030003207927475},
    {1.808653477078151, -0.407769396371889, -1.188711386659099,  0.787827305952836}};
#endif
#ifdef FIFTH_POLY
const float pinvA[4][3] = {
    {0.000014438849124, -0.000010704903652, 0.000001147678248},
    {-0.002598130538237, 0.001669326063148, -0.000096336506072},
    {0.098086714707812, -0.040364866387243, 0.001853487556038},
    {0.005867116833951, -0.002415066620459, 0.000110916098118} };
#endif
// This matrix was found using sensor locations 0.02015, 0.04365, 0.11615 m
// from the base and doing a pinv to get linear fitting of curvature
// pinvX * [K1;K2;K3] = [a;b]
const float pinvX[2][3] = {
	{-7.953675663083631, -3.261339811640989, 11.215015474724614},
	{0.810421311857300, 0.528959366368266, -0.339380678225565}
};

// Calibration matrices for wavelength to curvature reading. 
// C1 is for the first triplet of gratings close to the needle base
// C2 is for the middle triplet of gratings
// C3 is for the distal triplet of gratings
// Calibration from Jung Hwa on 03/01/2017
const float C1[3][3] = {
    {-0.000748867336927, 0.001068881969116, 0},
    {0.000149139340337, 0.001503803740640, 0},
    {-0.001976798053222, 0.000365319953008, 0}
};

const float C2[3][3] = {
    {-0.000873030745682, 0.002173139599615, 0},
    {0.001842941061259, -0.000594529565956, 0},
    {-0.003619990771172, 0.000367250476008, 0}
};

const float C3[3][3] = {
    {-0.001413614466484, -0.001629045053891, 0},
    {0.000261312453896, -0.001086421100232, 0},
    {0.001099139334677, -0.001165117978684, 0}
};


// Calibration matrix for wavelength to curvature reading. 
// C calibrates all the gratings at once
const float C[9][6] = {
{2.1007, -30.7799, 1.2063, -22.5888, -1.7125, 3.3502},
{11.3066, -23.6497, 7.3403, -18.7960, -5.0677, -3.1288},
{19.7811, -8.3739, 13.6011, -9.9832, -4.6852, -14.3471},
{9.8821, 45.5031, 7.0307, 30.9025, -1.1298, -15.0839},
{-6.7334, 22.5959, -3.5591, 17.5402, 6.3271, 0.8480},
{-20.3822, 3.3775, -14.1323, 7.2099, 4.4038, 18.4040},
{-25.2809, -30.8675, -17.5649, -17.6012, 5.4307, 23.4698},
{-5.1552, 9.2662, -4.8202, 8.0941, -3.0109, 5.3219},
{11.6203, 6.2016, 8.9900, 1.2319, 0.6455, -14.5923}
};


// *** Module variable definitions *** //
float *WLarray;           // buffer for containing real time wavelength data
float *baseWLarray;       // buffer for containing baseline wavelength

// *** Shape sensing coefficients *** //
float *est_coeff;

// *** TCP Connection declarations *** //
// TCP socket pointers for Interrogator
struct sockaddr_in serv_addr;
struct hostent *server;
int sockfd, portno, n;

// TCP socket pointers for server for HoloLens connection
int holoSocketfd, newHoloSocketfd;
struct hostent *holoserver;


// *** Function prototypes *** //
void initInterrogator();
int readWavelength(float *inWLArray, int arrLen);
void getNeedleShape(float *inWLArray, int arrLen,
                      float *inbaseWL_array, float *in_est_yz_coeff, 
                      float *in_est_xz_coeff);
void printWLs(float *inWLArray, int arrLen);
void float2Bytes( float *val, byte *byte_array );
void signal_handler(int signum);
//string string_format(const string fmt, ...);
void error(const char *msg);
void stack_prefault(void);


/***
 *  function: initInterrogator
 *  param: portnum - port number where the server will listen for connections
 *  description: this function will initialize socket and wait until a connection 
 *  from interrogator is established.
 *  Changes: 
 *      03/16/2017 - Michael L. re-writing all code
*/
void initInterrogator(){ 
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
    printf("Connected to interrogrator!\n");
}


/***
 *  function: initHoloLensServer
 *  param: portnum - port number where the server will listen for connections
 *  description: this function will initialize socket and wait until a connection 
 *  from HoloLens.
 *  Changes: 
 *      03/16/2017 - Michael L. re-writing all code
*/
void initHoloLensServer( int portnum ) // initialize server and listen for connection on portno
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

/***
 *  function: readWavelength
 *  param: inWLArray - input buffer where wavelength will be written as floats.
 *         arrLen - input buffer allocated size.
 *  description: this function send a read sensor command to the interrogator
 *  and it will wait to receive sensor data. It will then write this data into
 *  the allocated buffer. Function assumes it is already connected to server.
 *  Changes: 
 *      03/16/2017 - Michael L. re-writing all code
*/
int readWavelength(float *inWLArray, int arrLen) {   
   // getting wavelength data automatically
    char get_data[21] = {'#','G','E','T','_','U','N','B','U','F','F','E','R','E','D','_','D','A','T','A','\n'};
    n = write(sockfd,get_data,strlen(get_data));
    if (n < 0) 
         error("ERROR writing to socket");
        
    // read first 10 bytes data (including size information) and dump away
    char bufferSkip[DATA_LENGTH];
    //bufferSkip = (char *) malloc(sizeof(char)*DATA_LENGTH);
    n = read(sockfd,bufferSkip,DATA_LENGTH);
    if (n < 0) 
         error("ERROR reading from socket");
    //free(bufferSkip);

    // read the data into bytes buffer
    char *buffer;
    buffer = (char *) malloc(sizeof(char)*DATA_SIZE);
    n = read(sockfd,buffer,DATA_SIZE);
    if (n < 0) 
         error("ERROR reading from socket");

    // granularity buffer used to scaled the outputs 
    char *granularity;
    granularity = (char*) malloc(sizeof(char)*4);
    for(int j=0; j<4; j++) 
        granularity[j] = buffer[72+j];

    int *granularityf;
    granularityf = (int *)granularity;

    char peak_temp[4];
    int *peakf_temp = (int *) malloc(sizeof(int)*4);
    int wl_number=0;

    // Convert every 4 bytes into a float
    for(int h=0;h <= 44; h+=4) {
        memcpy(peak_temp, &buffer[88+h], 4);
        //  for(int j=0; j<4; j++) {
        //      peak_temp[j] = buffer[j+88+h];
        //  }
        peakf_temp = (int *)peak_temp; // this one is just dummy var
        //printf("%d, ", *peakf_temp);
        //store result into wavelength array
        wl_number=h/4;
        inWLArray[wl_number] = ((float)*peakf_temp)/(*granularityf); // store into allocated buffer
        //printf("%.8f\n", inWLArray[wl_number]);
    }
    //printf("\n");

    // free all allocated temporary buffers
    free(granularity);
    free(buffer);

    return 0;
}


/***
 *  function: getNeedleShape
 *  param: inWLArray       - input buffer where real wavelength are contained.
 *         arrLen          - input buffer allocated size.
 *         inbaseWL_array  - input baseline wavelength.
 *         in_est_yz_coeff - input shape coefficients in xz plane.
 *         in_est_xz_coeff - input shape coefficients in yz plane.
 *  description: this function takes in an array of wavelength readings
 *  and uses calibration information to turn them into shape coefficients. 
 *  Changes: 
 *      03/16/2017 - Michael L. re-writing all code
*/
void getNeedleShape(float *inWLArray, int arrLen,
                      float *inbaseWL_array, float *in_est_coeff) {
  float *dWL;
  dWL = (float*) malloc(sizeof(float)*12);

  for(int i=0; i<arrLen; i++)
  {
	dWL[i]=inWLArray[i]-inbaseWL_array[i];
  } 

  // Find all curvatures
	// K = dWL * C
	// dWL = [dWL[0] dWL[4] dWL[8] dWL[1] dWL[5] dWL[9] dWL[2] dWL[6] dWL[10]]
	float dWLt[9] = {dWL[0], dWL[4], dWL[8], dWL[1], dWL[5], dWL[9], dWL[2], dWL[6], dWL[10]};
  printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", dWLt[0], dWLt[1], dWLt[2], dWLt[3], dWLt[4], dWLt[5], dWLt[6], dWLt[7], dWLt[8]);
	// curvs = [Kx1 Ky1 Kx2 Ky2 Kx3 Ky3] where 1 is close to base and 3 is close to tip
	float curvs[6] = {
		dWLt[0]*C[0][0] + dWLt[1]*C[1][0] + dWLt[2]*C[2][0] + dWLt[3]*C[3][0] + dWLt[4]*C[4][0] + dWLt[5]*C[5][0] + dWLt[6]*C[6][0] + dWLt[7]*C[7][0] + dWLt[8]*C[8][0],
		dWLt[0]*C[0][1] + dWLt[1]*C[1][1] + dWLt[2]*C[2][1] + dWLt[3]*C[3][1] + dWLt[4]*C[4][1] + dWLt[5]*C[5][1] + dWLt[6]*C[6][1] + dWLt[7]*C[7][1] + dWLt[8]*C[8][1],
		dWLt[0]*C[0][2] + dWLt[1]*C[1][2] + dWLt[2]*C[2][2] + dWLt[3]*C[3][2] + dWLt[4]*C[4][2] + dWLt[5]*C[5][2] + dWLt[6]*C[6][2] + dWLt[7]*C[7][2] + dWLt[8]*C[8][2],
		dWLt[0]*C[0][3] + dWLt[1]*C[1][3] + dWLt[2]*C[2][3] + dWLt[3]*C[3][3] + dWLt[4]*C[4][3] + dWLt[5]*C[5][3] + dWLt[6]*C[6][3] + dWLt[7]*C[7][3] + dWLt[8]*C[8][3],
		dWLt[0]*C[0][4] + dWLt[1]*C[1][4] + dWLt[2]*C[2][4] + dWLt[3]*C[3][4] + dWLt[4]*C[4][4] + dWLt[5]*C[5][4] + dWLt[6]*C[6][4] + dWLt[7]*C[7][4] + dWLt[8]*C[8][4],
		dWLt[0]*C[0][5] + dWLt[1]*C[1][5] + dWLt[2]*C[2][5] + dWLt[3]*C[3][5] + dWLt[4]*C[4][5] + dWLt[5]*C[5][5] + dWLt[6]*C[6][5] + dWLt[7]*C[7][5] + dWLt[8]*C[8][5]
	};
 
   // Find curvatures
//float curv_xz[4] = {
//dWL[0]*C1[0][0]+dWL[2]*C1[1][0]+dWL[4]*C1[2][0],
//dWL[1]*C2[0][0]+dWL[3]*C2[1][0]+dWL[6]*C2[2][0],
//dWL[7]*C3[0][0]+dWL[9]*C3[1][0]+dWL[11]*C3[2][0],
//      0                                         };

//float curv_yz[4] = {
//dWL[0]*C1[0][1]+dWL[2]*C1[1][1]+dWL[4]*C1[2][1],
//dWL[1]*C2[0][1]+dWL[3]*C2[1][1]+dWL[6]*C2[2][1],
//dWL[7]*C3[0][1]+dWL[9]*C3[1][1]+dWL[11]*C3[2][1],
//      0                                         };

  // Fit into polynomial using pinvA and find the polynomial coefficients
  // put xz plane coefficients first
//in_est_coeff[0] = pinvA[0][0]*curv_xz[0] + pinvA[0][1]*curv_xz[1] + pinvA[0][2]*curv_xz[2] + pinvA[0][3]*curv_xz[3];
//in_est_coeff[1] = pinvA[1][0]*curv_xz[0] + pinvA[1][1]*curv_xz[1] + pinvA[1][2]*curv_xz[2] + pinvA[1][3]*curv_xz[3];
//in_est_coeff[2] = pinvA[2][0]*curv_xz[0] + pinvA[2][1]*curv_xz[1] + pinvA[2][2]*curv_xz[2] + pinvA[2][3]*curv_xz[3];
	in_est_coeff[0] = pinvX[0][0]*curvs[0] + pinvX[0][1]*curvs[2] + pinvX[0][2]*curvs[4];
	in_est_coeff[1] = pinvX[1][0]*curvs[0] + pinvX[1][1]*curvs[2] + pinvX[1][2]*curvs[4];

  // put yz plane coefficient second
//in_est_coeff[3] = pinvA[0][0]*curv_yz[0] + pinvA[0][1]*curv_yz[1] + pinvA[0][2]*curv_yz[2] + pinvA[0][3]*curv_yz[3];
//in_est_coeff[4] = pinvA[1][0]*curv_yz[0] + pinvA[1][1]*curv_yz[1] + pinvA[1][2]*curv_yz[2] + pinvA[1][3]*curv_yz[3];
//in_est_coeff[5] = pinvA[2][0]*curv_yz[0] + pinvA[2][1]*curv_yz[1] + pinvA[2][2]*curv_yz[2] + pinvA[2][3]*curv_yz[3];
	in_est_coeff[2] = pinvX[0][0]*curvs[1] + pinvX[0][1]*curvs[3] + pinvX[0][2]*curvs[5];
	in_est_coeff[3] = pinvX[1][0]*curvs[1] + pinvX[1][1]*curvs[3] + pinvX[1][2]*curvs[5];
  
  free(dWL);
}


/***
 *  function: printWLs
 *  param: inWLArray - input buffer where wavelength will be printed.
 *         arrLen - input buffer allocated size.
 *  description: this function simply prints all the values in the buffer.
 *  Changes: 
 *      03/16/2017 - Michael L. re-writing all code
*/
void printWLs(float *inWLArray, int arrLen) {
    for (int i=0; i< arrLen; i++) {
        printf("%f,", inWLArray[i]);
    }
    printf("\n");
}


int main(int argc, char* argv[]) {
    // Timing declarations
    struct timespec t;
    struct sched_param param;
    int interval = 500000; // 1 nanosecond per tick, 1ms

    // declare ourself as a real time task
    param.sched_priority = RT_PRIORITY;
    int result = sched_setscheduler(0, SCHED_FIFO, &param);
    if(result < 0)
    {
        perror("sched_setscheduler failed");
        exit(-1);
    }

    // lock all memory spaces
    if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1)
    {
        perror("mlockall failed");
        exit(-2);
    }

    // setup signal for ctrl-c
    signal(SIGINT, signal_handler);

    // allocate wavelength arrays
    int WLarrayLen=12;
    WLarray = (float*) malloc(sizeof(float)*WLarrayLen);
    baseWLarray = (float*) malloc(sizeof(float)*WLarrayLen);

    memset(WLarray, 0, sizeof(float)*12);
    memset(baseWLarray, 0, sizeof(float)*12);
    est_coeff = (float*) malloc(sizeof(float)*4);
    //est_yz_coeff = (float*) malloc(sizeof(float)*3);
    memset(est_coeff, 0, sizeof(float)*4);

#ifdef CONNECT2NEEDLE
    // initialize socket comm. to interrogator
    initInterrogator();
    
    // read the baseline wavelength
    readWavelength(baseWLarray, WLarrayLen);
#endif

#ifdef CONNECT2HOLOLENS
    // initialize socket comm. to HoloLens
    printf("Waiting for connection...\n");
    initHoloLensServer(20602);
#endif

    // counter for sending data to HoloLens at lower rate
    int commCounter = 0;

    // pre-fault our stack
    stack_prefault();
    clock_gettime(CLOCK_MONOTONIC ,&t);

    // start after two second
    printf("Starting loop in 2 seconds ...\n");
    t.tv_sec++;
    t.tv_sec++;

    int x = 1;
    while (true) {

      //// wait until next tick
      //clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
        if (commCounter % 10 == 0) { 
            memset(WLarray, 0, sizeof(float)*12);
            memset(est_coeff, 0, sizeof(float)*4);

            readWavelength(WLarray,WLarrayLen);
            getNeedleShape(WLarray, WLarrayLen, baseWLarray, est_coeff);
            byte bytes[4*4];
            //float2Bytes(&est_coeff[0], &bytes[0]);
            printf("%f, %f, %f, %f\n", est_coeff[0], est_coeff[1], est_coeff[2], est_coeff[3]);
            //printf("%f, %f, %f, %f, %f, %f\n", WLarray[0]-baseWLarray[0], WLarray[4]-baseWLarray[4], WLarray[8]-baseWLarray[8], WLarray[3]-baseWLarray[3], WLarray[4]-baseWLarray[4], WLarray[5]-baseWLarray[5]);
            //printf("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", WLarray[0], WLarray[1], WLarray[2], WLarray[3], WLarray[4], WLarray[5],
            //                                                            WLarray[6], WLarray[7], WLarray[8], WLarray[9], WLarray[10], WLarray[11]);
            //printf("%f\n", WLarray[11]);

#ifdef CONNECT2HOLOLENS
            int write_result;
            write_result = write(newHoloSocketfd, est_coeff, 4*4);
            if (write_result < 0) error("ERROR writing to socket");
#endif
        }
        commCounter ++;

        // calculate next tick
      //t.tv_nsec += interval;
      //while (t.tv_nsec >= NSEC_PER_SEC)
      //{
      //    t.tv_nsec -= NSEC_PER_SEC;
      //    t.tv_sec++;
      //}

    }
    printf("%d\n", x); 
    return 0;
}


void float2Bytes( float *val, byte *bytes_array ) {
    union {
        float float_variable[16];
        byte temp_array[4*6];
    } u;
    for (int i = 0; i < 6; i ++) {
        u.float_variable[i] = val[i];
    }
    memcpy(bytes_array, u.temp_array, 4*6);
}


// *** ========== Less important functions ========= *** //
void signal_handler(int signum) {
    switch (signum) {
        case SIGINT:
            // free all allocated buffers 
            free(baseWLarray);
            free(WLarray);
            free(est_coeff);

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

void error(const char *msg)
{
    perror(msg);
    exit(1);
}
