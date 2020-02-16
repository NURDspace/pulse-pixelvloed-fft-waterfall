#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <errno.h>
#include <signal.h>
#include <pulse/simple.h>
#include <pulse/error.h>
#include <fftw3.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <time.h>
#include <sys/time.h>
#include <systemd/sd-daemon.h>

//Tnx to https://gitlab.com/nitroxis/pasa/ for providing a good example of how to do this

#pragma pack(1)

//#define displayIP "10.208.42.159"
#define displayIP "127.0.0.1"

#define displayX 32 
#define displayY 128
#define displayXOffset 0
#define displayYOffset 0

struct Pixel {
    uint16_t y;
    uint16_t x;
    uint8_t r;
    uint8_t g;
    uint8_t b;	   
} pixel;  

struct Packet {
    uint16_t header;
    struct Pixel pixel[displayY];
} packet;

uint8_t frameBuffer[displayX][displayY];

struct sigaction old_sigint;
volatile bool run;

int framesPerSecond = 25;
double upperFrequency = 3520.0; // A7
double gain = 15.0;

void HSV_to_RGB(float h, float s, float v, uint8_t *r, uint8_t *g, uint8_t *b)
{
  int i;
  float f,p,q,t;

  s /= 100;
  v /= 100;

  if(s == 0) {
    // Achromatic (grey)
    *r = *g = *b = round(v*255);
    return;
  }

  h /= 60; // sector 0 to 5
  i = floor(h);
  f = h - i; // factorial part of h
  p = v * (1 - s);
  q = v * (1 - s * f);
  t = v * (1 - s * (1 - f));
  switch(i) {
    case 0:
      *r = round(255*v);
      *g = round(255*t);
      *b = round(255*p);
      break;
    case 1:
      *r = round(255*q);
      *g = round(255*v);
      *b = round(255*p);
      break;
    case 2:
      *r = round(255*p);
      *g = round(255*v);
      *b = round(255*t);
      break;
    case 3:
      *r = round(255*p);
      *g = round(255*q);
      *b = round(255*v);
      break;
    case 4:
      *r = round(255*t);
      *g = round(255*p);
      *b = round(255*v);
      break;
    default: // case 5:
      *r = round(255*v);
      *g = round(255*p);
      *b = round(255*q);
    }
}

long long current_timestamp() {
    struct timeval te; 
    gettimeofday(&te, NULL); // get current time
    long long milliseconds = te.tv_sec*1000LL + te.tv_usec/1000; // calculate milliseconds
    return milliseconds;
}

long map(long x, long in_min, long in_max, long out_min, long out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void onSigInt()
{
    sigaction(SIGINT, &old_sigint, NULL);
    run = false;
}

// hanning window.
double windowFunction(int n, int N)
{
    return 0.5 * (1.0 - cosf(2.0 * M_PI * n / (N - 1.0)));
}

void printUsage()
{
    printf("PulseAudio PixelVloed Spectrum Waterfall Analyzer\n");
    printf("\nUsage:\n");
    printf("pixelfft [options]\n");
    printf("\nOptions:\n");
    printf("-r <n>\tframes per second (default 30)\n");
    printf("-f <n>\tmaximum frequency (default 3520)\n");
    printf("-g <n>\tgain (i.e. a scaling factor for the bars, default 1.0)\n");
}

void calculateBars(fftw_complex* fft, int fftSize, int* bars, int numBars)
{
    double barWidthD = upperFrequency / (framesPerSecond * numBars);
    int barWidth = (int)ceil(barWidthD);

    double scale = 2.0 / fftSize * gain;

    // interpolate bars.
    int i = 0;
    for(int bar = 0; bar < numBars; bar++)
    {
        // get average.
        double power = 0.0;
        for(int j = 0; j < barWidth && i < fftSize; i++, j++)
        {
            double re = fft[i][0] * scale;
            double im = fft[i][1] * scale;
            power += re * re + im * im; // abs(c)
        }
        power *= (1.0 / barWidth); // average.
        if(power < 1e-255) power = 1e-255; // prevent overflows.

        // compute decibels.
        int dB = 255 + (int)(10.0 * log10(power));
        if(dB > 255) dB = 255;
        if(dB < 0) dB = 0;

        // set bar.
        bars[bar] = dB;
    }
}

int main(int argc, char* argv[])
{
    static const pa_sample_spec ss =
    {
        .format = PA_SAMPLE_FLOAT32LE,
        .rate = 44100,
        .channels = 2
    };

    // parse command line arguments.
    int c;
    while ((c = getopt(argc, argv, "r:f:g:")) != -1)
    {
        switch(c)
        {
            case 'r':
                framesPerSecond = atoi(optarg);
                break;

            case 'f':
                upperFrequency = atof(optarg);
                break;

            case 'g':
                gain = atof(optarg);
                break;

            case '?':
                printUsage();
                return 1;

            default:
                abort();
        }
    }

    //Do UDP thingies fixen
    int clientSocket, portNum;
    struct sockaddr_in serverAddr;
    socklen_t addr_size;
    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(5004);
    serverAddr.sin_addr.s_addr = inet_addr(displayIP);
    memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);
    addr_size = sizeof serverAddr;	
    clientSocket = socket(PF_INET, SOCK_DGRAM, 0);

    // open record device
    int error;
    pa_simple *s = pa_simple_new(NULL, "pasa", PA_STREAM_RECORD, NULL, "record", &ss, NULL, NULL, &error);

    // check error
    if (!s)
    {
        fprintf(stderr, "pa_simple_new() failed: %s\n", pa_strerror(error));
        return 1;
    }

    // input buffer.
    const int size = ss.rate / framesPerSecond;
    float window[size];
    float buffer[ss.channels * size];

    // compute window.
    for(int n = 0; n < size; n++)
        window[n] = windowFunction(n, size);

    // replace SIGINT handler.
    struct sigaction sigIntAction;
    memset(&sigIntAction, 0, sizeof(sigIntAction));
    sigIntAction.sa_handler = &onSigInt;
    sigaction(SIGINT, &sigIntAction, &old_sigint);

    // fftw setup
    double *in = (double*)fftw_malloc(sizeof(double) * size);
    fftw_complex *out = (fftw_complex*)fftw_malloc(sizeof(fftw_complex) * size);
    fftw_plan plan = fftw_plan_dft_r2c_1d(size, in, out, FFTW_MEASURE);

    run = true;

    uint8_t barHeight;
    struct Packet myPacket;
    myPacket.header = 0;
    float color[3];
    uint8_t r,g,b;

    sd_notify(0, "READY=1");

    // record loop
    while(run)
    {
        int barsL[displayY / 2];
        int barsR[displayY / 2];

        // read from device.
        if (pa_simple_read(s, buffer, sizeof(buffer), &error) < 0)
        {
            pa_simple_free(s);
            fprintf(stderr, "pa_simple_read() failed: %s\n", pa_strerror(error));
            return 1;
        }

        // left input.
        for (int i = 0; i < size; i++) {
            in[i] = (double)(window[i] * buffer[i * 2]);
        }
        fftw_execute(plan);
        calculateBars(out, size, barsL, displayY / 2);

        // right input.
        for (int i = 0; i < size; i++) {
            in[i] = (double)(window[i] * buffer[i * 2 + 1]);
        }
        fftw_execute(plan);
        calculateBars(out, size, barsR, displayY / 2);

        //Shift framebuffer by 1
        memmove(frameBuffer[1], frameBuffer[0], sizeof(frameBuffer[1])*(displayX - 1));

        for(int i = 0; i < displayY / 2; i++)
        {
            /*
            barHeight = (uint8_t)map((int)barsL[i], 0, 255, 0, 255);
            frameBuffer[0][i] = barHeight;
            barHeight = (uint8_t)map((int)barsR[i], 0, 255, 0, 255);
            frameBuffer[0][displayY - 1 - i] = barHeight;
            */

            frameBuffer[0][i] = barsL[i]; 
            frameBuffer[0][displayY - 1 - i] = barsR[i];
        }

        //Draw Pixels
        for (int x = 0; x < displayX; x++) {
            for (int y = 0; y < displayY; y++) {
                myPacket.pixel[y].x = x + displayXOffset;
                myPacket.pixel[y].y = y + displayYOffset;
                int value = map(frameBuffer[x][y],0,255,0,360);
                HSV_to_RGB((float)value, 100.0, 50.0, &r, &g, &b);
                myPacket.pixel[y].r = r;
                myPacket.pixel[y].g = g;
                myPacket.pixel[y].b = b;
            }
            sendto(clientSocket,(void*)&myPacket,sizeof(myPacket),0,(struct sockaddr *)&serverAddr,addr_size);
        }
    }

    // clean up fftw
    fftw_destroy_plan(plan);
    fftw_free(in);
    fftw_free(out);

    // clean up pulseaudio
    pa_simple_free(s);

    return 0;
}

