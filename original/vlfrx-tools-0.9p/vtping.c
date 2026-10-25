//
//  Copyright (c) 2010 Paul Nicholson
//  All rights reserved.
//  
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//  1. Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//  2. Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//  
//  THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
//  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
//  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
//  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
//  NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
//  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "config.h"
#include "vtport.h"
#include "vtlib.h"

#include <fftw3.h>
#include <png.h>

static int rqDF = 10;     // Hertz.  Vertical resolution of spectrogram

static double DTHRESH = 0.5;    // Detection threshold, -D option
static double RTHRESH = 0.5;    // Rejection threshold, -R option

#define TSPAN 6.0      // Approximate span of the STFT, seconds

static double tdelay = 0.4; // Delay after event detection before saving the
                            // event.  Gives time for event to be improved by
                            // a better match at another time position.

// Exponential moving average coefficients for tracking the noise floor.
#define SFAC1 0.00002
#define SFAC2 0.02

// Input signal variables
static int sample_rate;
static double srcal = 1.0;
static uint64_t NS = 0;                  // Number of input samples read so far

// Variables for X display
static int XFLAG = 0;         // Use X display
static int run = 1;           // Continuous running
static int single_step = 0;   // Do one FFT frame, re-evaluate then pause

// STFT dimensions
static int xxW = 0;       // Number of columns in the internal STFT

// Fourier transform variables
static int FFTWID = 0;    // Fourier transform input width
static int BINS = 0;      // Number of frequency bins
static double DF = 0.0;   // Frequency resolution
static double DT = 0.0;   // Time resolution
static int HOP = 4;       // Fourier transform overlap factor is 1/HOP

// Output thumbnail spectrogram
static double sg_LOW = -1;   // Spectrogram base, Hz, from -s option
static double sg_HIGH = -1;  // Spectrogram top,Hz, from -s option
static int sgH = 0;       // Spectrogram height, number of cells
static int sgW = 0;       // Spectrogram width, number of cells
static int sgB = 0;       // Base bin of spectrogram

#define SET_T  30         // Initial seconds of faster time constant

// Limits for input dynamic range compression
static double gfloor = 1.0;   
static double gtop = 5.0;

static char *outdir = NULL;     // -d option: output directory for events

static int OFLAG_TE = FALSE;    // TRUE if -ote option given
static int OFLAG_EXT = FALSE;   // TRUE -f -oext option given

// Table of frequency ranges to examine
static struct FLIST {
   double f1, f2;    // Hertz, low and high
   int b1, b2;       // Bins, low and high, inclusive
}
 *dlist = NULL, *rlist = NULL;

static int ndlist = 0;
static int nrlist = 0;

///////////////////////////////////////////////////////////////////////////////
//  Ring Buffers                                                             //
///////////////////////////////////////////////////////////////////////////////

//
//  A set of circular buffers, all sharing the same base pointer.
//

//    BP + 1  oldest column
//    BP      most recently filled column
//    BP - 1  last column
//    BP - 2  last but one

static int BP = 0;
static int NP = 0;

// Aring: normalised input spectrum
static double *Aring;              
#define ARING(X,Y) Aring[(X)*BINS + (Y)]

// Ering: input time domain ring
static double *Ering;      
#define ERING(X) (Ering + (X) * FFTWID)

// Tring: timestamp of first sample of each STFT column
static timestamp *Tring;
#define TRING(X) Tring[X]

static double *rring;   // Column sums of cell strengths in reject bands

// Converts from the x coordinate of the STFT (0..xxW-1) to ring buffer index.
#define xtoBP( x) ((BP + 1 + x) % xxW)

static double *ipacc;
static double *ipin;

static void format_timestamp( timestamp T, char *s)
{
   time_t xsec = timestamp_secs( T);
   struct tm *tm = gmtime( &xsec);
   sprintf( s, "%04d-%02d-%02d_%02d:%02d:%02d.%03d",
                tm->tm_year + 1900, tm->tm_mon+1, tm->tm_mday,
                tm->tm_hour, tm->tm_min, tm->tm_sec,
                (int)(1e3 * timestamp_frac( T)));
}
///////////////////////////////////////////////////////////////////////////////
//  Principal Component Analysis                                             //
///////////////////////////////////////////////////////////////////////////////

// A ring buffer: a 2-D array of results aligned with the STFT 
static double *Pring;
#define PRING(X,Y) Pring[(X)*BINS + (Y)]

///////////////////////////////////////////////////////////////////////////////
//  X Display                                                                //
///////////////////////////////////////////////////////////////////////////////

//  This is only used for development

#if USE_X11
   #include <X11/Xlib.h>
   #include <X11/Xutil.h>

// Parameters that only affect the display
//

#define FTS     2

#define sgwin_L 0
#define sgwin_T 0
#define sgwin_W (FTS * xxW)
#define sgwin_H (FTS * BINS)

#define edwin_L 0
#define edwin_T 0
#define edwin_W (FTS * xxW/2)
#define edwin_H (FTS * BINS)

#define ipwin_L 0
#define ipwin_T 0
#define ipwin_W (FTS * BINS)
#define ipwin_H (FTS * 100)

static Display *xdisplay;
static int      screen;
static Window sgwin, edwin, ipwin, rootwin;

static Font    ch_font;
static GC gc_text;
static XColor yellow, magenta, green, red, cyan, blue;
static unsigned long black_pixel, white_pixel;

#define MAXLEV 64
static GC whiteGC[MAXLEV];
static GC redGC[MAXLEV];

static void draw_edwin( int x1, int x2)
{
   int x;

   for (x = x1; x <= x2; x++)
   {
      int cx = xtoBP( x);

      int y;
      for (y = 0; y < BINS; y++)
      {
         int sy = BINS - y - 1;

         int bmin = 20;
         int pval = bmin + PRING( cx, y)/DTHRESH/3 * (MAXLEV - bmin);
         if (pval >= MAXLEV) pval = MAXLEV-1;
         if (pval < bmin) pval = bmin;

         if (PRING(cx,y) > DTHRESH)
            XFillRectangle( xdisplay, edwin, 
                         whiteGC[pval], x*FTS, sy*FTS, FTS, FTS);
         else
         {
            XFillRectangle( xdisplay, edwin, 
                         redGC[pval], x*FTS, sy*FTS, FTS, FTS);
          }
      }
   }
//   XSync( xdisplay, 0);
}

static void draw_sgwin( void)
{
   int x, y;

   for (x = 0; x < xxW; x++)
   {
      int cx = xtoBP( x);

      for (y = 0; y < BINS; y++)
      {
         int sy = BINS - y - 1;
         int bmin = 20;
         int pval = bmin + ARING( cx, y) * (MAXLEV - bmin);
         if (pval >= MAXLEV) pval = MAXLEV-1;
         if (pval < bmin) pval = bmin;

         XFillRectangle( xdisplay, sgwin, 
                         whiteGC[pval], x*FTS, sy*FTS, FTS, FTS);
      }
   }
   XSync( xdisplay, 0);
}

static void report_ipwin( int x)
{
   double F = DF * x;

   VT_printf( "F=%.0f ipin=%.3e ACC=%.3e\n", F, ipin[x], ipacc[x]);
}

static void draw_ipwin( void)
{
   int x, k;
   int ipmin = -8, ipmax = 3;
   double v;

   XFillRectangle( xdisplay, ipwin, 
                         whiteGC[0], 0, 0, BINS * FTS, ipwin_H);
   for (x = 0; x < BINS; x++)
   {
      v = ipin[x]; if (v == 0) v = 1e-9; v = log10( v);

      if (v >= ipmin && v < ipmax)
      {
         int y = ipwin_H * (v-ipmin)/(ipmax - ipmin);
         y = ipwin_H - y;
         for (k=0; k<FTS; k++)
            XDrawPoint( xdisplay, ipwin, 
                         whiteGC[MAXLEV-1], x*FTS + k, y);
      }

      v = ipacc[x]; if (v == 0) v = 1e-9; v = log10( v);

      if (v >= ipmin && v < ipmax)
      {
         int y = ipwin_H * (v-ipmin)/(ipmax - ipmin);
         y = ipwin_H - y;
         for (k=0; k<FTS; k++)
            XDrawPoint( xdisplay, ipwin, 
                         redGC[MAXLEV-1], x*FTS + k, y);
      }
   }
}

static int service( void)
{
   int n = 0;
   XEvent ev;

   while (XPending( xdisplay))
   {
      XNextEvent( xdisplay, &ev);

      if (ev.type == ButtonPress &&
          ev.xbutton.button == 1
        )
      {
         if (ev.xany.window == edwin ||
             ev.xany.window == sgwin)
         {
            int x = ev.xbutton.x/FTS;
            int y = BINS - ev.xbutton.y/FTS - 1;

            double T = x * DT;
            double F = DF * y;

            char temp[50];
            int bx = xtoBP( x);
            format_timestamp( TRING( bx), temp);
            fprintf( stderr,
                    "Ta=%s T=%.2f F=%.0f a=%.2f p=%.3f\n",
                      temp, T, F, ARING( bx, y), PRING( bx, y));
         }

         if (ev.xany.window == ipwin)
            report_ipwin( ev.xbutton.x/FTS);
      }
      else
      if (!run &&
          ev.type == ButtonPress &&
          ev.xbutton.button == 2
        ) { single_step = 1;  run = 1; }
      else
      if (ev.type == ButtonPress &&
          ev.xbutton.button == 3
        ) run = !run;
      else
      if (ev.type == Expose && ev.xexpose.count == 0)
      {
         if (ev.xany.window == sgwin) draw_sgwin();
         if (ev.xany.window == edwin) draw_edwin( 0, xxW/2);
         if (ev.xany.window == ipwin) draw_ipwin();
      }
      n++;
   }

   return n;
}

static void display_pause( void)
{
   run = 0;
   draw_edwin( 0, xxW/2);
   while (!run && XFLAG) {service(); XSync( xdisplay, 0); usleep( 10000); }
}

static void update_spectrogram( void)
{
   int y;

   XCopyArea( xdisplay, sgwin, sgwin, whiteGC[0],
                 FTS, 0, xxW * FTS, BINS * FTS, 0, 0); 

   int x = xtoBP( xxW - 1);

   for (y = 0; y < BINS; y++)
   {
      int sy = BINS - y - 1;

      int bmin = 20;
      int pval = bmin + ARING( x, y) * (MAXLEV - bmin);
      if (pval >= MAXLEV) pval = MAXLEV-1;
      if (pval < bmin) pval = bmin;

      XFillRectangle( xdisplay, sgwin, 
                         whiteGC[pval], (xxW - 1) * FTS, sy * FTS, FTS, FTS);
   }

   x = xxW/2;
   // int cx = xtoBP( x);

   XCopyArea( xdisplay, edwin, edwin, whiteGC[0],
                 FTS, 0, xxW/2 * FTS, BINS * FTS, 0, 0); 
   draw_edwin( x, x);
   XSync( xdisplay, 0);
//   usleep( DT * 1e6 * 0.9);
}

static void setup_display( int argc, char *argv[])
{
   int i;
   XEvent ev;
   long megamask;

   if ((xdisplay = XOpenDisplay( NULL)) == NULL)
      VT_bailout( "cannot open xdisplay");

   screen = DefaultScreen( xdisplay);
   rootwin = DefaultRootWindow( xdisplay);
   ch_font = XLoadFont( xdisplay, "6x13");

   Colormap cmap = DefaultColormap( xdisplay, screen);
  
   black_pixel = BlackPixel( xdisplay, screen);
   white_pixel = WhitePixel( xdisplay, screen);

   yellow.red = 0xff00;
   yellow.green = 0xff00;
   yellow.blue = 0;
   magenta.red = 0x3c00;
   magenta.green = 0x6000;
   magenta.blue = 0x8000;

   green.red = 0;
   green.green = 0xff00;
   green.blue = 0;

   red.red = 0xff00; 
   red.green = 0;
   red.blue = 0;

   cyan.red = 0xc000;
   cyan.green = 0xc000;
   cyan.blue = 0xff00;

   blue.red = 0;
   blue.green = 0;
   blue.blue = 0xff00;

   if (!XAllocColor( xdisplay, cmap, &yellow) ||
       !XAllocColor( xdisplay, cmap, &magenta) ||
       !XAllocColor( xdisplay, cmap, &red) ||
       !XAllocColor( xdisplay, cmap, &cyan) ||
       !XAllocColor( xdisplay, cmap, &blue) ||
       !XAllocColor( xdisplay, cmap, &green)) VT_bailout( "alloc color");


   sgwin = XCreateSimpleWindow( xdisplay, rootwin, 
                                0, 0, sgwin_W, sgwin_H, 
                                1, white_pixel,  black_pixel);
   XSetStandardProperties( xdisplay, sgwin, 
                           "vtping input", "vtping",
                           None, NULL, 0, NULL);

   edwin = XCreateSimpleWindow( xdisplay, rootwin, 
                                0, 0, edwin_W, edwin_H, 
                                1, white_pixel,  black_pixel);
   XSetStandardProperties( xdisplay, edwin, 
                           "vtping detector", "vtping",
                           None, NULL, 0, NULL);

   ipwin = XCreateSimpleWindow( xdisplay, rootwin, 
                                0, 0, ipwin_W, ipwin_H,
                                1, white_pixel,  black_pixel);
   XSetStandardProperties( xdisplay, ipwin, 
                           "vtping floor", "vtping",
                           None, NULL, 0, NULL);

   gc_text = XCreateGC( xdisplay, rootwin, 0L, 0);
   XSetForeground( xdisplay, gc_text, white_pixel);
   XSetBackground( xdisplay, gc_text, black_pixel);
   XSetFont( xdisplay, gc_text, ch_font);

   for (i = 0; i < MAXLEV; i++)
   {
      whiteGC[i] = XCreateGC( xdisplay, rootwin, 0, 0);
      redGC[i] = XCreateGC( xdisplay, rootwin, 0, 0);

      XColor tx;
      tx.red = 255 *i/(double) MAXLEV; tx.red <<= 8;
      tx.green = 255 *i/(double) MAXLEV;  tx.green <<= 8;
      tx.blue = 255 *i/(double) MAXLEV;   tx.blue <<= 8;
      if (!XAllocColor( xdisplay, cmap, &tx))
         VT_bailout( "cannot alloc col %d", i);
      XSetForeground( xdisplay, whiteGC[i], tx.pixel);

      tx.red = 255 *i/(double) MAXLEV; tx.red <<= 8;
      tx.green = 0;  tx.green <<= 8;
      tx.blue = 0;   tx.blue <<= 8;
      if (!XAllocColor( xdisplay, cmap, &tx))
         VT_bailout( "cannot alloc col %d", i);
      XSetForeground( xdisplay, redGC[i], tx.pixel);
   }

   XSelectInput( xdisplay, sgwin, ExposureMask);
   XSelectInput( xdisplay, edwin, ExposureMask);
   XSelectInput( xdisplay, ipwin, ExposureMask);

   XMapRaised( xdisplay, sgwin);
   XMapRaised( xdisplay, edwin);
   XMapRaised( xdisplay, ipwin);

   do 
   {
      XNextEvent( xdisplay, &ev);
   }
    while (ev.type != Expose);

   megamask = KeyPressMask | ExposureMask |
              EnterWindowMask | LeaveWindowMask | 
              ButtonPressMask | ButtonReleaseMask;

   XSelectInput( xdisplay, sgwin, megamask);
   XSelectInput( xdisplay, edwin, megamask);
   XSelectInput( xdisplay, ipwin, megamask);
}

#endif

///////////////////////////////////////////////////////////////////////////////
//  Result set                                                               //
///////////////////////////////////////////////////////////////////////////////

//
//  A structure to hold detected events.  
//

static struct RSET {

   struct FLIST *band;

   timestamp T1;                                             // Event timestamp
   timestamp T2;                                            // Event completion
   double srcal;                                     // Sample rate calibration

   double D;      // Detection level                           
   double R;      // Rejection level
   double F;      // Detection bin frequency

   int bp1;       // Ring buffer index at T1
} rset;

static void reset_rset( struct RSET *r)
{
   memset( r, 0, sizeof( struct RSET));
}

//
//  Store a PNG thumbnail spectrogram of the event.
//

#define th_MARGIN 0.0   // Time offset of left margin in STFT, seconds
#define th_scale 2      // Scale factor, STFT cells to pixels   
#define th_W ((int)((sgW - tdelay/DT) * th_scale))   // Thumbnail width, pixels
#define th_H (sgH * th_scale)                       // Thumbnail height, pixels

static void store_rset_thumb( struct RSET const *r, char const *prefix)
{
   char fname[200], tname[200];

   sprintf( fname, "%s/%s.png", outdir, prefix);
   sprintf( tname, "%s/new.png", outdir);

   FILE *fp = fopen( tname, "w");
   if (!fp) VT_bailout( "cannot open %s, %s", tname, strerror( errno));

   png_structp png_ptr = png_create_write_struct
       (PNG_LIBPNG_VER_STRING, (png_voidp) NULL, NULL, NULL);
   if (!png_ptr) VT_bailout( "cannot alloc png_ptr");

   png_infop info_ptr = png_create_info_struct( png_ptr);
   if (!info_ptr) VT_bailout( "cannot create png info_struct");

   if (setjmp( png_jmpbuf( png_ptr))) VT_bailout( "png jmp error");

   png_init_io( png_ptr, fp); 

   png_set_IHDR( png_ptr, info_ptr, th_W, th_H,
       8, PNG_COLOR_TYPE_RGB, PNG_INTERLACE_NONE,
       PNG_COMPRESSION_TYPE_DEFAULT, PNG_FILTER_TYPE_DEFAULT);

   png_write_info( png_ptr, info_ptr);

   int x, y;
   int bmin = 20;

   for (y = th_H-1; y >= 0; y--)
   {
      unsigned char rbuf[th_W * 3], *pr = rbuf;

      for (x = 0; x < th_W; x++)
      {
         int qx = (x + th_W/2)/th_scale - (BP - r->bp1);
         int cx = xtoBP( qx);
         int cy = y/th_scale + sgB;
         int dval = bmin + 2 * PRING( cx, cy)/DTHRESH/3 * (255 - bmin);
         if (dval > 255) dval = 255;
         if (dval < bmin) dval = bmin;

         int aval = bmin + ARING( cx, cy) * (255 - bmin);
         if (aval > 255) aval = 255;
         if (aval < bmin) aval = bmin;

         if (PRING(cx,cy) > DTHRESH)
         {
            *pr++ = dval;
            *pr++ = dval;
            *pr++ = dval;
         }
         else
         {
            *pr++ = aval;
            *pr++ = 0;
            *pr++ = 0;
         }
      }

      png_write_row( png_ptr, rbuf);
   }
   
   png_write_end( png_ptr, NULL);
   fclose( fp);
   png_destroy_write_struct( &png_ptr, &info_ptr);

   if (rename( tname, fname) < 0)
      VT_bailout( "cannot rename output file, %s", strerror( errno));
}

//
//  Time domain analytic signal (if OFLAG_EXT is set)
//
//  Columns:
//  1:   Index;
//  2:   Timestamp;
//  3:   Raw signal, full bandwidth;
//  4:   I signal, bandlimited to the detection band;
//  5:   Q signal, bandlimited to the detection band;
//  6:   Instantaneous frequency;
//  7:   Analytic magnitude;

static void store_rset_analytic( struct RSET const *r, char const *prefix)
{
   char fname[200], tname[200];

   sprintf( fname, "%s/%s.td", outdir, prefix);
   sprintf( tname, "%s/new.td", outdir);

   FILE *fp = fopen( tname, "w");
   if (!fp) VT_bailout( "cannot open %s, %s", tname, strerror( errno));

   int i1 = r->bp1 - 1;
   int i2 = r->bp1 + 0.5/DT;

   int tlen = (i2 - i1) * FFTWID/HOP;
   double *raw = VT_malloc_zero( sizeof( double) * tlen);
   double *out1 = VT_malloc_zero( sizeof( double) * tlen);
   double *out2 = VT_malloc_zero( sizeof( double) * tlen);
   timestamp *times = VT_malloc_zero( sizeof( timestamp) * tlen);

   int i;
   for (i = i1; i < i2; i++)
   {
      int j;
      double dt = 1/(sample_rate * srcal);

      for (j = 0; j < FFTWID/HOP; j++)
      {
         int k = (i - i1) * FFTWID/HOP + j;
         int w = (i + xxW) % xxW;
         raw[k] = ERING( w)[j];
         times[k] = timestamp_add( TRING( w), j * dt);
      }
   }

   complex double *X1 = VT_malloc( sizeof( complex double) * tlen);
   complex double *X2 = VT_malloc( sizeof( complex double) * tlen);
   fftw_plan plan1 = fftw_plan_dft_r2c_1d( tlen, raw, X1,
               FFTW_ESTIMATE |  FFTW_PRESERVE_INPUT);;

   fftw_plan plan2 = fftw_plan_dft_c2r_1d( tlen, X1, out1, FFTW_ESTIMATE);
   fftw_plan plan3 = fftw_plan_dft_c2r_1d( tlen, X2, out2, FFTW_ESTIMATE);

   fftw_execute( plan1);

   double df = (double)sample_rate/tlen;
   for (i = 0; i < r->band->f1/df; i++) X1[i] = 0;
   for (i = r->band->f2/df; i < tlen / 2; i++) X1[i] = 0;

   for (i = 0; i < tlen/2; i++) X2[i] = X1[i] * -I;

   fftw_execute( plan2);
   fftw_execute( plan3);

   for (i = 0; i < tlen; i++)
   {
      double F = 0;
      complex double c2 = out1[i] + I * out2[i];
      double m = cabs( c2);

      if (i > 0)
      {
         complex double c1 = out1[i-1] + I * out2[i-1];

         double dp = carg( c2) - carg( c1);
         while (dp > 2*M_PI) dp -= 2 * M_PI;
         while (dp < 0) dp += 2 * M_PI;
         F = dp/(2 * M_PI) * sample_rate;
      }

      char temp[50];
      if (OFLAG_TE) timestamp_string6( times[i], temp);
      else
         VT_format_timestamp( temp, times[i]);

      fprintf( fp, "%d %s %.3e %.3e %.3e %.1f %.3e\n",
            i, temp, raw[i], out1[i]/tlen, out2[i]/tlen, F, m);
   }
   fclose( fp);
   if (rename( tname, fname) < 0)
      VT_bailout( "cannot rename output file, %s", strerror( errno));

   free( raw);
   free( out1);
   free( out2);
   free( times);
   free( X1);
   free( X2);
   fftw_destroy_plan( plan1);
   fftw_destroy_plan( plan2);
   fftw_destroy_plan( plan3);
}

//
//  Write a text file giving info about the event
//

static void store_rset_txt( struct RSET const *r, char const *prefix)
{
   char fname[200];
   char tname[200];

   sprintf( fname, "%s/%s.txt", outdir, prefix);
   sprintf( tname, "%s/new.txt", outdir);

   FILE *fp = fopen( tname, "w");
   if (!fp) VT_bailout( "cannot open %s, %s", tname, strerror( errno));

   char temp[30];  timestamp_string3( r->T1, temp);
   fprintf( fp, "T=%s", temp);

   fprintf( fp, " D=%.2f R=%.2f", r->D, r->R);
   if (r->R > RTHRESH) fprintf( fp, " S");
   fprintf( fp, " dt=%.3f", timestamp_diff( r->T2, r->T1));
   fprintf( fp, "\n");
   fclose( fp);

   if (rename( tname, fname) < 0) 
      VT_bailout( "cannot rename output file, %s", strerror( errno));
}

//
//  Store the signal.
//

static void store_rset_vt( struct RSET const *r, char const *prefix)
{
   char *fname, *tname, *vname;

   if (asprintf( &fname, "%s/%s.vt", outdir, prefix) < 0 ||
       asprintf( &tname, "%s/new.vt", outdir) < 0 ||
       asprintf( &vname, "%s/new.vt,i2", outdir) < 0)
      VT_bailout( "out of memory");

   int chans = 1;

   VTFILE *vtout = VT_open_output( vname, chans, 0, sample_rate);
   if (!vtout) VT_bailout( "cannot open %s: %s", tname, strerror( errno));

   double frame[3];

   VT_set_timebase( vtout, TRING( xtoBP( 0)), srcal);
   int x, i;
   for (x = 0; x < xxW; x++)
      for (i = 0; i < FFTWID/HOP; i++)
      {
         int xp = xtoBP( x);
         frame[0] = ERING( xp)[i];
         VT_insert_frame( vtout, frame);
      }

   VT_release( vtout);
   VT_close( vtout);

   if (rename( tname, fname) < 0) 
      VT_bailout( "cannot rename output file, %s", strerror( errno));

   free( vname); free( tname); free( fname);
}

//
//  Store the result set in the events directory.
//

static void store_rset( struct RSET const *r)
{
   mkdir( outdir, 0777);

   char temp[30];   timestamp_string3( r->T1, temp);

   char prefix[50];
   sprintf( prefix, "%d.%d",
            timestamp_secs( r->T1),
            (int)(timestamp_frac( r->T1) * 10));

   VT_report( 0, "RSET: %s T=%s D=%.2f R=%.2f dt=%.3f",
        prefix, temp, r->D, r->R, timestamp_diff( r->T2, r->T1));

   store_rset_vt( r, prefix);
   if (OFLAG_EXT) store_rset_analytic( r, prefix);
   store_rset_thumb( r, prefix);
   store_rset_txt( r, prefix);
}

///////////////////////////////////////////////////////////////////////////////
//  Ping Register                                                            //
///////////////////////////////////////////////////////////////////////////////

static double *ping_mask = NULL;

static int HW1 = 0; 
static int HW2 = 0;

static double Thold = 1.0;
static timestamp tholdoff = timestamp_ZERO;

static void setup_ping_mask( void)
{
   HW1 = 0.25 / DT;
   HW2 = 3.0 / DT;

   VT_report( 1, "HW1 %d HW2 %d", HW1, HW2);

   ping_mask = VT_malloc_zero( sizeof( double) * (HW1 + HW2));

   int h;

   for (h = 0; h < HW1; h++)
   {
      double x = h;

      double y = 2*(1/(1 + exp(-x*4)) - 0.5);
      y *= exp(-x*x/200);

      ping_mask[HW2 + h] = y;
   }

   for (h = 1; h <= HW2; h++)
   {
      double x = -h;
      double y = 2*(1/(1 + exp(-x*4)) - 0.5);
      y *= exp(-x*x/4000);
 
      ping_mask[HW2 - h] = y;
   }

//   for (h = 0; h < HW1; h++) ping_mask[HW2 + h] = +1.0/HW1;
//   for (h = 1; h <= HW2; h++) ping_mask[HW2 - h] = -1.0/(HW2);

   //
   //  Normalise the mask.
   //

   double m1 = 0;
   for (h = 1; h <= HW2; h++) m1 += ping_mask[HW2 - h];
   for (h = 1; h <= HW2; h++) ping_mask[HW2 - h] /= fabs( m1);

   double m2 = 0;
   for (h = 0; h < HW1; h++) m2 += ping_mask[HW2 + h];
   for (h = 0; h < HW1; h++) ping_mask[HW2 + h] /= fabs( m2);

   FILE *f = fopen( "/tmp/vtping.mask", "w");
   if (f)
   {
      for (h = 0; h < HW1 + HW2; h++)
         fprintf( f, "%d %.3f %.3f\n", h, DT * (h - HW2), ping_mask[h]);
      fclose( f);
   }

   double s = 0;
   for (h = 0; h < HW1 + HW2; h++) s += ping_mask[h];
   VT_report( 1, "s = %.6f m1 = %.3f m2 = %.3f\n", s, m1, m2);
}

static void register_events( void)
{
   //
   //  Don't register any events until we've been running for 30 seconds, to
   //  allow time for moving averages to settle.
   //

   if (NS/sample_rate < 10) return;

   int n;
   int x = xxW/2;
   int bp = xtoBP( x);

   //
   //  Check rejection bands.
   //

   double ra = 0;
   for (n = 0; n < nrlist; n++)
   {
      struct FLIST *f = rlist + n;

      int b;
      double g = 0;
      for (b = f->b1; b <= f->b2; b++) g += ARING( bp, b);
      ra += g / (f->b2 - f->b1 + 1);
   }

   if (ra > RTHRESH)
   {
      rset.D = 0;
      return;
   }

   //
   //  Ping detect.
   //

   double plevel = 0;
   struct FLIST *pband = NULL;

   int pbin = -1;

   for (n = 0; n < ndlist; n++)
   {
      struct FLIST *f = dlist + n;

      int b;
      for (b = f->b1; b <= f->b2; b++)
      {
         int h;

         double ta = 0;
         for (h = 0; h < HW1 + HW2; h++)
            ta += ping_mask[h] * ARING( xtoBP(x - HW2 + h), b);

         if (ta < 0) ta = 0;
         PRING( bp, b) = ta;

         if (ta > DTHRESH && ta > plevel)
         {
            plevel = ta;
            pband = f;
            pbin = b;
         } 
      }
   }

   if (timestamp_diff( TRING( bp), tholdoff) < 0) return;

   if (plevel > DTHRESH && plevel > rset.D) 
   {
      if (!rset.D)
      {
         rset.band = pband;
         rset.T1 = TRING( bp);
         rset.bp1 = bp;
      }

      rset.T2 = timestamp_ZERO;
      rset.srcal = srcal;
      rset.D = plevel;
      rset.R = ra;
      rset.F = pbin * DF;
   }

   if (plevel > DTHRESH) return;

   if (!rset.D) return;
   if (timestamp_is_ZERO( rset.T2)) rset.T2 = TRING( bp);

   //  If we have an event and it's now older than tdelay seconds, then
   //  save the event.
   
   if (timestamp_GT( TRING( bp), timestamp_add( rset.T1, tdelay)))
   {
      if (!outdir)
      {
         char temp[100];

         if (OFLAG_TE)
            timestamp_string6( rset.T1, temp);
         else
            VT_format_timestamp( temp, rset.T1);

         VT_printf( "ping %s D=%.3f R=%.3f F=%.1f\n",
                    temp, rset.D, rset.R, rset.F);
         fflush( stdout);
      }

      if (outdir) store_rset( &rset);
      tholdoff = timestamp_add( rset.T2, Thold);

      reset_rset( &rset);

      #if USE_X11
      if (XFLAG) display_pause();
      #endif
   }
}

//
// Track the noise floor.  Asymmetric time constants in a moving
// average.
//

static void update_noise_floor( int j, double v)
{
   static int set = 0;

   // For first SET_T seconds, use faster time constants to settle the
   // background quickly after start-up.
   if (!set)
   {
      if (NS < SET_T * sample_rate)
      {
         if (v > ipacc[j])
            ipacc[j] = ipacc[j] * (1 - SFAC1 * 10) + SFAC1 * 10 * v;
         else
            ipacc[j] = ipacc[j] * (1 - SFAC2 * 10) + SFAC2 * 10 * v;

         return;
      }

      set = 1;   // Move to normal time constants from now on
      VT_report( 1, "settling completed");
   }

   if (v > ipacc[j])
      ipacc[j] = ipacc[j] * (1 - SFAC1) + SFAC1 * v;
   else
      ipacc[j] = ipacc[j] * (1 - SFAC2) + SFAC2 * v;
}

static void process_buffer( double const *ebuf, timestamp T)
{
   static complex double *Xout = NULL;
   static double *Xin;
   static fftw_plan ffp;
   static double *save;

   //
   // First time through, initialise the Fourier transform.
   //

   if (!Xout)
   {
      Xout = VT_malloc( sizeof( complex double) * FFTWID);
      Xin = VT_malloc( sizeof( double) * FFTWID);
      ffp = fftw_plan_dft_r2c_1d( FFTWID, Xin, Xout, FFTW_ESTIMATE);
      save = VT_malloc_zero( sizeof( double) * FFTWID);
   }

   //
   // Line up the ring buffer column to be filled by this transform frame.
   //

   BP = (BP + 1) % xxW;

   // Save the timestamp
   TRING( BP) = T;

   // Save the raw signal and prepare it for FFT

   int i;
   double *tp = ERING( BP);
   for ( i = 0; i < FFTWID/HOP; i++) *tp++ = ebuf[i];

   memmove( save, save + FFTWID/HOP, sizeof( double) * (FFTWID - FFTWID/HOP));
   for (i = 0; i < FFTWID/HOP; i++) save[FFTWID - FFTWID/HOP + i] = ebuf[i];

   // Run the FFT
   for (i = 0; i < FFTWID; i++) Xin[i] = save[i];
   fftw_execute( ffp);

   //
   // Maintain a moving average noise floor power for each bin.
   //

   int bin;
   for (bin = 0; bin < BINS; bin++)
   {
      double v = cabs( Xout[bin]) * cabs( Xout[bin]);

      ipin[bin] = v;   // Bin power

      update_noise_floor( bin, v);
   }

   //
   //  Limit the signal in each bin to logarithmic range gfloor to gtop and
   //  scale to range 0..1.   Store the result in the analyser input
   //  buffer ARING.
   //

   for (bin = 0; bin < BINS; bin++)
   {
      double v = ipin[bin] / ipacc[bin];   // Power S/N ratio

      // Log power S/N ratio and restrict dynamic range
      v = log10( isnan( v) || v <= 0 ? v = gfloor : v);
      v = (v - gfloor)/(gtop - gfloor);
      if (v < 0) v = 0;
      if (v > 1) v = 1;

      ARING( BP, bin) = v;    // Insert to the analyser ring buffer
   }

   #if USE_X11
   if (XFLAG) draw_ipwin();
   #endif

   if (NP < xxW) NP++;
   else 
   {
      register_events();

      #if USE_X11
         if (XFLAG) update_spectrogram();
      #endif

   }
}

///////////////////////////////////////////////////////////////////////////////
//  Main                                                                     //
///////////////////////////////////////////////////////////////////////////////

static void usage( void)
{
   fprintf( stderr, 
      "usage:  vtping [options] -d outdir input\n"
      "\n"
      "options:\n"
      " -v          Increase verbosity\n"
      " -B          Run in background\n"
      " -L name     Specify logfile\n"
      "\n"
      " -D thresh   Detection threshold (default 0.5)\n"
      " -R thresh   Rejection threshold (default 0.5)\n"
      "\n"
      " -F detect,low,high    Detection frequency range, Hz\n"
      " -F reject,low,high    Rejection frequency range, Hz\n"
      "                       Multiple -F options may be used\n"
      "\n"
      " -d outdir    Place event files under outdir\n"
      " -s low,high  Output spectrogram frequency range\n"
      "\n"
      " -ote        Output unix epoch timestamp\n"
      " -oti        Output ISO timestamps (default)\n"
      " -oext       Extended output\n"
   );

   exit( 1);
}

static void parse_output_option( char *s)
{
   if (!strcmp( s, "te")) { OFLAG_TE = TRUE; return; }
   if (!strcmp( s, "ti")) { OFLAG_TE = FALSE; return; }
   if (!strcmp( s, "ext")) { OFLAG_EXT = TRUE; return; }

   VT_bailout( "unrecognised output option: [%s]", s);
}

int main(int argc, char *argv[])
{
   VT_init( "vtping");

   char *bname;
   VTFILE *vtfile;
   int background = 0;

   while (1)
   {  
      int c = getopt( argc, argv, "vBXhd:p:o:F:R:D:L:s:?");
   
      if (c == 'v') VT_up_loglevel();
      else
      if (c == 'B') background = 1;
      else
      if (c == 'L') VT_set_logfile( "%s", optarg);
      else
      if (c == 'X') XFLAG = 1;
      else
      if (c == 'F' && !strncmp( optarg, "detect,", 7))
      {
         dlist = VT_realloc( dlist, (ndlist+1) * sizeof( struct FLIST));
         VT_parse_freqspec( optarg+7, &dlist[ndlist].f1, &dlist[ndlist].f2);
         if (dlist[ndlist].f1 > dlist[ndlist].f2)
            VT_bailout( "invalid -F argument");
         ndlist++;
      }
      else
      if (c == 'F' && !strncmp( optarg, "reject,", 7))
      {
         rlist = VT_realloc( rlist, (nrlist+1) * sizeof( struct FLIST));
         VT_parse_freqspec( optarg+7, &rlist[nrlist].f1, &rlist[nrlist].f2);
         if (rlist[nrlist].f1 > rlist[nrlist].f2)
            VT_bailout( "invalid -F argument");
         nrlist++;
      }
      else
      if (c == 'F') VT_bailout( "bad argument to -F");
      else
      if (c == 's') VT_parse_freqspec( optarg, &sg_LOW, &sg_HIGH);
      else
      if (c == 'd') outdir = strdup( optarg);
      else
      if (c == 'o') parse_output_option( optarg);
      else
      if (c == 'D') DTHRESH = atof( optarg);
      else
      if (c == 'R') RTHRESH = atof( optarg);
      else
      if (c == -1) break;
      else
         usage();
   }

   if (!outdir)
      VT_report( 1, "no output directory given, events will not be stored");

   if (!ndlist) VT_bailout( "no frequency range given, needs -F");

   #if !USE_X11
      if (XFLAG) VT_report( 0, "-X ignored, not compiled with X11");
   #endif

   if (argc > optind + 1) usage();
   bname = strdup( optind < argc ? argv[optind] : "-");

   if (background)
   {
      int flags = bname[0] == '-' ? KEEP_STDIN : 0;
      if (!outdir) flags |= KEEP_STDOUT;
      VT_daemonise( flags);
   }

   struct VT_CHANSPEC *chspec = VT_parse_chanspec( bname);
   vtfile = VT_open_input( bname);
   if (!vtfile)
      VT_bailout( "cannot open input %s: %s", bname, VT_error);

   sample_rate = VT_get_sample_rate( vtfile);

   VT_init_chanspec( chspec, vtfile);
   VT_report( 1, "channels: %d, sample_rate: %d", chspec->n, sample_rate);

   //
   //  Setup FFT.
   //  rqDF is the requested frequency resolution, DF is the actual resolution.
   //

   int nyquist = sample_rate / 2;
   BINS = nyquist / rqDF;
   DF = nyquist / (double) BINS;

   FFTWID = BINS * 2;
   DT = FFTWID / HOP / (double) sample_rate;

   //
   //  Setup STFT.   xxW is the number of columns, approx TSPAN seconds but
   //  make it an odd number.
   // 

   xxW = TSPAN / DT; 
   if (xxW % 1 == 0) xxW++;

   //
   //  Prepare output spectrogram
   //

   if (sg_LOW < 0) sg_LOW = 0;
   if (sg_HIGH < 0) sg_HIGH = sample_rate/2;
   sgB = sg_LOW / DF;
   sgH = sg_HIGH / DF - sgB;
   sgW = xxW;

   VT_report( 1, "xxW=%d bins=%d DF=%.1f DT=%.3f sgH=%d sgW=%d", 
                     xxW, BINS, DF, DT, sgH, sgW);

   Aring = VT_malloc_zero( sizeof( double) * xxW * BINS);
   Pring = VT_malloc_zero( sizeof( double) * xxW * BINS);
   rring = VT_malloc_zero( sizeof( double) * xxW);
   ipin = VT_malloc_zero( sizeof( double) * BINS);
   Ering = VT_malloc_zero( sizeof( double) * xxW * FFTWID);
   Tring = VT_malloc_zero( sizeof( timestamp) * xxW);

   // Prepare frequency ranges
   int i;
   for (i = 0; i < ndlist; i++)
   {
      dlist[i].b1 = dlist[i].f1 / DF;
      if (dlist[i].b1 <0) dlist[i].b1 = 0;
      dlist[i].b2 = dlist[i].f2 / DF;
      if (dlist[i].b2 >= BINS) dlist[i].b2 = BINS - 1;
      VT_report( 1, "detect %.0f to %.0f bins %d to %d", 
             dlist[i].f1, dlist[i].f2, dlist[i].b1, dlist[i].b2);
   }

   for (i = 0; i < nrlist; i++)
   {
      rlist[i].b1 = rlist[i].f1 / DF;
      if (rlist[i].b1 <0) rlist[i].b1 = 0;
      rlist[i].b2 = rlist[i].f2 / DF;
      if (rlist[i].b2 >= BINS) rlist[i].b2 = BINS - 1;
      VT_report( 1, "reject %.0f to %.0f bins %d to %d", 
             rlist[i].f1, rlist[i].f2, rlist[i].b1, rlist[i].b2);
   }

   #if USE_X11
   if (XFLAG) setup_display( argc, argv);
   #endif

   ipacc = VT_malloc_zero( sizeof( double) * BINS);

   setup_ping_mask();

   //
   //  Main loop
   //

   double *ebuf = VT_malloc( sizeof( double) * FFTWID);
   double *inframe;
   int nb = 0;
   timestamp timebase = timestamp_ZERO;

   while (1)
   {
      if (run)
      {
         // Capture the timestamp of the first sample of each FT frame,
         // along with the current sample rate calibration.
         if (!nb)
         {
            timebase = VT_get_timestamp( vtfile);
            srcal = VT_get_srcal( vtfile);
         }
   
         inframe = VT_get_frame( vtfile);
         if (!inframe) break;
   
         ebuf[nb] = inframe[chspec->map[0]];
         if (++nb < FFTWID/HOP) continue;
         nb = 0;
   
         //
         //  The FT input buffer is full, process the frame
         //
   
         process_buffer( ebuf, timebase); NS += FFTWID / HOP;
         if (XFLAG && single_step) single_step = run = 0;
      }
      else usleep( 10000);

      #if USE_X11
      if (XFLAG && service()) XSync( xdisplay, 0);
      #endif
   }

   VT_exit( "end of input");
   return 0;
}

