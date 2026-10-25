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

static int NFLAG = 0;                                       // Set by -n option
static int SFLAG = 0;                                       // Set by -s option
static int IFLAG = 0;                                       // Set by -i option
static long double trunc_secs = 0;                          // Set by -t option
static long double add_secs = 0;                            // Set by -a option

static void usage( void)
{
   fprintf( stderr,
       "usage:  vtdate [options] [timestamp|timespec]\n"
       "\n"
       "options:\n"
       "  -v       Increase verbosity\n"
       "  -n       Output numeric timestamp\n"
       "  -s       Output string timestamp\n"
       "  -i       Integer output\n"
       "  -t secs  Truncate to multiple of secs\n"
       "  -a secs  Add offset of secs\n"
     );
   exit( 1);
}

static void print_timestamp( timestamp T)
{
   char temp[50];

   T = timestamp_normalise( T);

   if (NFLAG)
   {
      if (IFLAG) VT_printf( "%d", timestamp_secs( T));
      else
      {
         timestamp_string6( T, temp);
         VT_printf( "%s", temp);
      }
   }
   if (NFLAG && SFLAG) VT_printf( " "); 
   if (SFLAG)
   {
      VT_format_timestamp( temp, T);
      IFLAG ? VT_printf( "%.19s", temp) : VT_printf( "%s", temp);
   }
}

static void evaluate( char *argument)
{
   int is_timespec = strchr( argument, ',') ? 1 : 0;

   if (is_timespec)
   {
      timestamp TS = timestamp_ZERO, TE = timestamp_ZERO;

      VT_parse_timespec( argument, &TS, &TE);

      TS = timestamp_add( TS, add_secs);
      TE = timestamp_add( TE, add_secs);

      if (trunc_secs)
      {
         TS = timestamp_truncate( TS, trunc_secs);
         TE = timestamp_truncate( TE, trunc_secs);
      }

      print_timestamp( TS);
      VT_printf( " ");
      print_timestamp( TE);

      if (NFLAG || IFLAG)
      {
         VT_printf( " ");
         if (IFLAG) VT_printf( "%ld", (long) timestamp_diff( TE, TS));
         else VT_printf( "%.6f", timestamp_diff( TE, TS));
      }
   }
   else
   {
      timestamp T;

      T = VT_parse_timestamp( argument);

      T = timestamp_add( T, add_secs);
      if (trunc_secs) T = timestamp_truncate( T, trunc_secs);

      print_timestamp( T);
   }
}

int main( int argc, char *argv[])
{
   VT_init( "vtdate");

   while (1)
   {
      int c = getopt( argc, argv, "vnsit:a:?");

      if (c == 'v') VT_up_loglevel();
      else
      if (c == 'n') NFLAG = 1;
      else
      if (c == 's') SFLAG = 1;
      else
      if (c == 'i') IFLAG = 1;
      else
      if (c == 't') trunc_secs = strtold( optarg, NULL);
      else
      if (c == 'a') add_secs = strtold( optarg, NULL);
      else
      if (c == -1) break;
      else
         usage();
   }

   // If neither -n nor -s given, then do both
   if (!NFLAG && !SFLAG) NFLAG = SFLAG = 1;

   if (optind == argc)     // Stream of arguments into stdin?
   {
      char temp[512];
      while (fgets( temp, 512, stdin))
      {
         // Format: timestamp|timespec rest
         char *ts = strtrim( temp);

         // Allow blank lines through
         if (!*ts)
         {
            VT_printf( "\n");
            continue;
         }

         char *rest = skip_field( ts);
         if (*rest) *rest++ = 0;   // Terminating ts
         if (*rest) skip_space( rest);
 
         VT_printf( "%s ", ts); 
         evaluate( ts);
         *rest ? VT_printf( " %s\n", rest) : VT_printf( "\n");
      }
   }
   else
   if (optind + 1 == argc) // Arguments on the command line?
   {
      char *argument = argv[optind];
      evaluate( argument);
      VT_printf( "\n");
   }
   else usage();


   return 0;
}


