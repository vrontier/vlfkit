//
// Copyright (c) 2010 Paul Nicholson
// All rights reserved.
//  
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  
// THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
// IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
// NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
// THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//

#include "config.h"
#include "vtport.h"
#include "vtlib.h"

static char *bname = NULL;
static VTFILE *vtfile;

static int IFLAG = FALSE;            // Run info only
static int RFLAG = FALSE;            // Rectify signals before use

static uint64_t nft = 0;

static double *sum = NULL;
static double *sumsq = NULL;
static double *peak = NULL;

static int navg = 0;
static int breaks = 0;
static int nib = 0;
static int sample_rate = 0;
static char *data_format = "";
static char *stamptype = "";
static int chans = 0;
static double Etime = 0;
static timestamp Tstart = timestamp_ZERO, Tend = timestamp_ZERO;
static uint64_t clips = 0;          // Count of samples exceeding +/- 1.0

// Output format controls
static int OFLAG_TE = FALSE;   // Set by -ote: Use unix epoch for timestamps
static int OFLAG_TR = FALSE;   // Set by -otr: Output seconds offset

static char *format_timestamp( timestamp T)
{
   static char s[50];

   if (OFLAG_TR) sprintf( s, "%.6f", timestamp_diff( T, Tstart));
   else
   if (OFLAG_TE)
   {
      char temp[30]; timestamp_string6( T, temp); sprintf( s, "%s", temp);
   }
   else
      VT_format_timestamp( s, T);

   return s;
}

static void parse_output_options( char const *s)
{
   if (!strcmp( s, "te")) { OFLAG_TE = TRUE;  return; }
   if (!strcmp( s, "ti")) { OFLAG_TE = FALSE; return; }
   if (!strcmp( s, "tr")) { OFLAG_TR = TRUE; return; }

   VT_bailout( "unrecognised output format option: [%s]", s);
}

///////////////////////////////////////////////////////////////////////////////
// Amplitude                                                                 //
///////////////////////////////////////////////////////////////////////////////

static int MFLAG = FALSE;                // Enable moments reporting, -m option
                                                   // Set by -m option

static double *moment1 = NULL;
static double *moment2 = NULL;
static double *moment3 = NULL;

static timestamp period_T = timestamp_ZERO;     // Start time of current period
static int period_N = 0;           // Number of samples into the current period
static double period_R = 0;             // Reporting period, seconds, set by -p

static void parse_amp_options( char *s)   // Deprecated option
{
   MFLAG = TRUE;
   RFLAG = TRUE;
   while (s && *s)
   {  
      char *p = strchr( s, ',');
      if (p) p++;
      
      if (!strncmp( s, "r=", 2))
      {
         period_R = atof( s+2);
         VT_report( 0,
                    "deprecated -a r=%s option, use -m %s instead", s+2, s+2);
      }
      else
         VT_bailout( "unrecognised amplitude option: %s", s);
      
      s = p;
   }
}

static void moments_init( void)
{
   int size = chans * sizeof( double);

   moment1 = VT_malloc( size);
   moment2 = VT_malloc( size);
   moment3 = VT_malloc( size);
}

static void moments_reset( void)
{
   int size = chans * sizeof( double);

   memset( moment1, 0, size);
   memset( moment2, 0, size);
   memset( moment3, 0, size);

   period_T = VT_get_timestamp( vtfile);
   period_N = 0;
}

static void moments_output( void)
{
   if (period_R) VT_printf( "%s ", format_timestamp( period_T));
  
   VT_printf( "%.6f", period_N/(double) sample_rate);

   int ch;
   for (ch = 0; ch < chans; ch++)
      VT_printf( " %.3e %.3e %.3e",
         moment1[ch]/period_N, moment2[ch]/period_N, moment3[ch]/period_N);

   VT_printf( "\n");
   if (fflush( stdout) < 0) VT_exit( "output closed");
}

static void moments_update( struct VT_CHANSPEC const * chspec,
                            double const *frame)
{  
   int ch;
   
   for (ch = 0; ch < chans; ch++)
   {  
      double v = frame[chspec->map[ch]];
      if (RFLAG && v < 0) v = -v;
      moment1[ch] += v; 
      moment2[ch] += v*v; 
      moment3[ch] += v*v*v; 
   }

   period_N++;

   if (period_R)
   {
      timestamp T = VT_get_timestamp( vtfile);
      if (timestamp_diff( T, period_T) >= period_R - 0.5/sample_rate)
      {
         moments_output();
         moments_reset();
      }
   }
}

///////////////////////////////////////////////////////////////////////////////
// Histogram                                                                 //
///////////////////////////////////////////////////////////////////////////////

static int HFLAG = FALSE;              // Run histogram, set by -h option

static uint64_t *hdata = NULL;         // Array of histogram counters
static int hbins = 0;                  // Number of histogram bins
                                       // Set by -h bins= option
static double hmax = 1.0;              // Upper limit of histogram
                                       // Set  by -h max= option
static double hmin = 0.0;              // Lower limit of histogram
                                       // Set  by -h min= option

static int hlog = FALSE;             // TRUE if logarithmic intervals requested
static double hbase = 0;             // Base of the logarithm

static void parse_histo_options( char *s)
{
   HFLAG = TRUE;

   while (s && *s)
   {
      char *p = strchr( s, ',');
      if (p) p++;

      if (!strncmp( s, "max=", 4)) hmax = atof( s+4);
      else
      if (!strncmp( s, "min=", 4)) hmin = atof( s+4);
      else
      if (!strncmp( s, "bins=", 5)) hbins = atoi( s+5);
      else
      if (!strncmp( s, "log", 3)) hlog = TRUE;
      else
         VT_bailout( "unrecognised histogram option: %s", s);

      s = p;
   }

   if (hlog)
   {
      if (!hmin) hmin = hmax * pow( 2, -32);

      hbase = pow( hmax/hmin, 1.0/hbins);
      VT_report( 1, "base %.6e hmin %.6e hmax %.6e", hbase, hmin, hmax);
   }
}

static void histo_init( void)
{
   int size = 8 * chans * hbins;
   hdata = VT_malloc_zero( size);
}

static void histo_reset( void)
{
   int size = 8 * chans * hbins;
   memset( hdata, 0, size);

   period_T = VT_get_timestamp( vtfile);
   period_N = 0;
}

static void histo_output( void)
{
   int i, ch;
   uint64_t *totals = VT_malloc_zero( 8 * chans);

   for (i = 0; i < hbins; i++)
      for (ch = 0; ch < chans; ch++) totals[ch] += hdata[i*chans + ch];

   double dh = (hmax - hmin) / hbins;

   for (i = 0; i < hbins; i++)
   {
      double v;
 
      if (!hlog) v = hmin + i * dh;
      else v = hmin * pow( hbase, i);

      if (period_R) VT_printf( "%s ", format_timestamp( period_T));

      VT_printf( "%+.6e", v);
      for (ch = 0; ch < chans; ch++)
         VT_printf( " %.5e", hdata[i*chans + ch]/(double) totals[ch]);
      VT_printf( "\n");
   }

   free( totals);
}

static void histo_update( struct VT_CHANSPEC const * chspec,
                          double const *frame)
{
   int ch;

   double dh = (hmax - hmin) / hbins;
   for (ch = 0; ch < chans; ch++)
   {
      double v = frame[chspec->map[ch]];

      if (RFLAG && v < 0) v = -v;

      int bin;
      if (!hlog)
         bin = (v - hmin) / dh;
      else
         bin = log( v/hmin)/log( hbase);

      if (bin >= 0 && bin < hbins) hdata[bin * chans + ch]++;
   }

   period_N++;

   if (period_R)
   {
      timestamp T = VT_get_timestamp( vtfile);
      if (timestamp_diff( T, period_T) >= period_R - 0.5/sample_rate)
      {
         histo_output();
         histo_reset();
      }
   }
}

///////////////////////////////////////////////////////////////////////////////
// Curses display                                                            //
///////////////////////////////////////////////////////////////////////////////

static void curses_bailout_hook( void)
{
   curs_set( 2);
   endwin();
}

static void xyprintf( int x, int y, char *format, ...)
{
   va_list ap;
   char temp[200];

   va_start( ap, format);
   vsprintf( temp, format, ap);
   va_end( ap);

   int i;
   for (i = 0; temp[i]; i++) mvaddch( y, x+i, temp[i]);
}

static void curses_init( void)
{
   VT_bailout_hook( curses_bailout_hook);
   initscr();
   scrollok( stdscr, FALSE); clearok( stdscr, FALSE);
   nonl();
   curs_set( 0);
   attrset( A_NORMAL);

   xyprintf( 2, 1, "buffer: %s", bname);
   xyprintf( 2, 2, "format: %s", data_format);
}

static void curses_update( double *frame)
{
   int ch;

   // Update screen 5 times/sec
   if (navg < sample_rate/5) return;

   xyprintf( 2, 3, "sample rate: %d, correction %10.8f", sample_rate,
                VT_get_srcal( vtfile));
   xyprintf( 2, 4, "blocks: %5d", VT_get_bcnt( vtfile));
   xyprintf( 2, 5, "frames per block: %d", vtfile->bsize);

   timestamp T = VT_get_timestamp( vtfile);
   char temp[50];

   VT_format_timestamp( temp, T);

   xyprintf( 2, 7, "timestamp: %s %s", temp, stamptype);

   xyprintf( 2, 9, "latency: %.6f", timestamp_diff( VT_rtc_time(), T));
   xyprintf( 2, 10, "breaks: %d", breaks);

   for (ch = 0; ch < chans; ch++)
   {
      double rms = sqrt(sumsq[ch]/navg);
      xyprintf( 2, 12+ch, "channel %2d: rms=%.3f peak=%.3f", 
               ch+1, rms, peak[ch]);
      sum[ch] = sumsq[ch] = peak[ch] = 0;
   }

   navg = 0;
   refresh();
}

///////////////////////////////////////////////////////////////////////////////
// Main                                                                      //
///////////////////////////////////////////////////////////////////////////////

static void usage( void)
{
   fprintf( stderr,
       "usage: vtstat [options] buffer\n"
       "       vtstat -V\n"
       "\n"
       "options:\n"
       "\n"
       "  -E secs     Analyse for this number of seconds, then exit\n"
       "  -i          Report info only\n"
       "  -V          Report vtlib package version\n"
       "  -p seconds  Reporting period\n"
       "  -m          Report first three moments of amplitude\n"
       "  -o te       Output numeric timestamps (default string)\n"
       "  -o tr       Output relative times\n"
       "  -r          Rectify signals first\n"
       "\n"
       "histogram options\n"
       "(options can be comma separated by a -h)\n" 
       "\n"
       "  -h bins=integer   Number of amplitude bins to use (no default)\n"
       "  -h min=float      Minimum bin value (default 1.0)\n"
       "  -h max=float      Maximum bin value (default 0.0)\n"
       "  -h log            Use log intervals (default linear)\n"
       "                    (base computed from min,max,bins)\n"
     );

   exit( 1);
}

int main( int argc, char *argv[])
{
   VT_init( "vtstat");

   while (1)
   {
      int c = getopt( argc, argv, "Vvirh:E:a:mo:p:?");
  
      if (c == 'V')
      {
         VT_printf( "vtlib version %s\n", PACKAGE_VERSION);
         VT_report( 1, "int %d", (int) sizeof( int));
         VT_report( 1, "long %d", (int) sizeof( long));
         VT_report( 1, "double %d", (int) sizeof( double));
         VT_report( 1, "long double %d", (int) sizeof( long double));
         #ifdef USE_COMPOUND_TIMESTAMP
            VT_report( 1, "using compound timestamp");
         #endif
         exit( 0);
      }
      else
      if (c == 'v') VT_up_loglevel();
      else
      if (c == 'r') RFLAG = TRUE;
      else
      if (c == 'p') period_R = atof( optarg);
      else
      if (c == 'o') parse_output_options( optarg);
      else
      if (c == 'h') parse_histo_options( optarg);
      else
      if (c == 'a')   // Deprecated, replaced by -m
         parse_amp_options( optarg);
      else
      if (c == 'm') MFLAG = TRUE;
      else
      if (c == 'i') IFLAG = TRUE;
      else
      if (c == 'E') Etime = atof( optarg);
      else
      if (c == -1) break;
      else
         usage();
   }  

   // Check mutually exclusive options
   int co = 0;
   if (HFLAG) co++; 
   if (MFLAG) co++; 
   if (IFLAG) co++; 
   if (co > 1) VT_bailout( "incompatible options");

   if (HFLAG && hbins <= 0) VT_bailout( "-h needs bins=");
   
   if (argc > optind + 1) usage();
   bname = strdup( optind < argc ? argv[optind] : "-");

   struct VT_CHANSPEC *chspec = VT_parse_chanspec( bname);

   if ((vtfile = VT_open_input( bname)) == NULL)
      VT_bailout( "cannot open: %s", VT_error);

   VT_init_chanspec( chspec, vtfile);
   chans = chspec->n;

   sum = VT_malloc_zero( sizeof( double) * chans);
   sumsq = VT_malloc_zero( sizeof( double) * chans);
   peak = VT_malloc_zero( sizeof( double) * chans);

   sample_rate = VT_get_sample_rate( vtfile);
   nft = 0;

   switch (VT_flags( vtfile) & VTFLAG_FMTMASK)
   {
      case VTFLAG_FLOAT4: data_format = "float4"; break;
      case VTFLAG_FLOAT8: data_format = "float8"; break;
      case VTFLAG_INT1: data_format = "int1"; break;
      case VTFLAG_INT2: data_format = "int2"; break;
      case VTFLAG_INT4: data_format = "int4"; break;
      default: VT_bailout( "invalid data format");
   }

   stamptype = VT_flags( vtfile) & VTFLAG_RELT ?  "relative" : "absolute";
   Tstart = VT_get_timestamp( vtfile);

   if (HFLAG)
   {
      histo_init();
      histo_reset();
   }
   else
   if (MFLAG)
   {
      moments_init();
      moments_reset();
   }
   else
   if (!IFLAG)
      curses_init();

   double srcal = 1.0;

   while (1)
   {
      int e = VT_is_block( vtfile);
      if (e < 0)
      {
         VT_report( 1, "end of stream");
         break;
      }

      if (VT_rbreak( vtfile) > 0)
      {
         breaks++;

         char temp[50];
         VT_format_timestamp( temp, VT_get_timestamp( vtfile));
         VT_report( 1, "break to %s", temp);
      }

      if (e)
      {
         Tend = VT_get_timestamp( vtfile);
         srcal = VT_get_srcal( vtfile);
        
         char temp[50];
         VT_format_timestamp( temp, Tend); 
         VT_report( 2, "block %s %.9f nfb=%d", temp, srcal, vtfile->nfb);
         nib = 0;
      }
 
      double *frame = VT_get_frame( vtfile);

      int ch;
      for (ch = 0; ch < chans; ch++)
      {
         double val = frame[chspec->map[ch]];
         if (val < 0) val = -val;
         sum[ch] += val;
         sumsq[ch] += val * val;

         if (val > 1) clips++;

         if (val > peak[ch]) peak[ch] = val;
         else
         if (val < -peak[ch]) peak[ch] = -val;
      }

      navg++;
      nft++;
      nib++;

      if (Etime && nft/(double) sample_rate > Etime)
      {
         VT_report( 1, "completed %f seconds", Etime);
         break;
      }

      if (HFLAG)
         histo_update( chspec, frame);
      else
      if (MFLAG)
         moments_update( chspec, frame);
      else
      if (!IFLAG)
         curses_update( frame);
   }

   //
   // Finalisation after input stream completed.
   //

   Tend = timestamp_add( Tend, nib / (double) sample_rate);

   if (HFLAG && !period_R) histo_output();
   if (MFLAG && !period_R) moments_output();
 
   if (IFLAG)
   {  
      VT_printf( "format: %s\n", data_format);
      VT_printf( "channels: %d\n", chans);
      VT_printf( "samples: %lld\n", (long long) nft);
      VT_printf( "clips: %lld\n", (long long) clips);
      VT_printf( "breaks: %d\n", breaks);
      VT_printf( "sample rate: %d, correction %10.8f\n", sample_rate, srcal);
      VT_printf( "duration: %.7f seconds\n",  nft/(sample_rate * srcal));

      char temp[50];
      VT_format_timestamp( temp, Tstart);
      VT_printf( "start: %s\n", temp);

      VT_format_timestamp( temp, Tend);
      VT_printf( "end: %s, interval %.7f seconds\n",
                  temp, timestamp_diff( Tend, Tstart));

      int ch;
      if (nft)
         for (ch = 0; ch < chans; ch++)
            VT_printf( "mean,rms,peak: %d %.3e,%.3e,%.3e\n", ch+1,
                        sum[ch]/nft, sqrt( sumsq[ch]/nft), peak[ch]);
   }
   else
   {
      curs_set( 2);
      endwin();
   }

   return 0;
}

