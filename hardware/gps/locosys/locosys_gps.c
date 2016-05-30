/*
* Copyright (C) 2011 The Android Open Source Project
* Copyright (C) 2011 Atheros Communications, Inc.
* Copyright (C) 2011-2014 Freescale Semiconductor, Inc.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*      http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*/
#include <errno.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/epoll.h>
#include <math.h>
#include <time.h>
#include <semaphore.h>
#include <signal.h>
#include <unistd.h>
#include <stdio.h>
#include <time.h>
#include <sys/inotify.h>
#include <poll.h>

#define  LOG_TAG  "locosys_gps"

#include <cutils/log.h>
#include <cutils/sockets.h>
#include <cutils/properties.h>
#include <hardware/gps.h>

#undef  GPS_DEBUG
#undef  GPS_DEBUG_TOKEN	/* print out NMEA tokens */

#ifdef GPS_DEBUG
#  define  D(...)   ALOGD(__VA_ARGS__)
#else
#  define  D(...)   ((void)0)
#endif

#define  I(...)     ALOGI(__VA_ARGS__)
#define  E(...)     ALOGE(__VA_ARGS__)

#define GPS_STATUS_CB(_cb, _s)    \
  if ((_cb).status_cb) {          \
    GpsStatus gps_status;         \
    gps_status.status = (_s);     \
    (_cb).status_cb(&gps_status); \
    D("gps status callback: 0x%x", _s); \
  }

/* Nmea Parser stuff */
#define  NMEA_MAX_SIZE  83

enum {
  STATE_QUIT  = 0,
  STATE_INIT  = 1,
  STATE_START = 2
};

typedef struct
{
    int valid;
    double systime;
    GpsUtcTime timestamp;
} LocosysTimemap_t;

typedef struct {
    int     pos;
    int     overflow;
    int     utc_year;
    int     utc_mon;
    int     utc_day;
    int     utc_diff;
    GpsLocation  fix;
    GpsSvStatus  sv_status;
    int     sv_status_changed;
    char    in[ NMEA_MAX_SIZE+1 ];
    int     gsa_fixed;
    LocosysTimemap_t timemap;
} NmeaReader;

typedef struct {
    int                     init;
    int                     fd;
    GpsCallbacks            callbacks;
    pthread_t               thread;
    pthread_t		        nmea_thread;
    pthread_t               tmr_thread;
    int                     control[2];
    int                     fix_freq;
    sem_t                   fix_sem;
    NmeaReader              reader;
} GpsState;

int 	gps_opentty(GpsState *state);
void 	gps_closetty(GpsState *state);
int     gps_checkstate(GpsState *state);

GpsCallbacks* g_gpscallback = NULL;
static GpsState  _gps_state[1];
static GpsState *gps_state = _gps_state;
static int started    = 0;
static int continue_thread = 1;

static const char GPS_PERFORM_COLD_START[9]     = {0xF1, 0xD9, 0x06, 0x40, 0x01, 0x00, 0x01, 0x48, 0x22};
static const char GPS_PERFORM_WARM_START[9]     = {0xF1, 0xD9, 0x06, 0x40, 0x01, 0x00, 0x02, 0x49, 0x23};
static const char GPS_PERFORM_HOT_START[9]      = {0xF1, 0xD9, 0x06, 0x40, 0x01, 0x00, 0x03, 0x4A, 0x24};
static const char GPS_PERFORM_FACTORY_RESET[16] = {0xF1, 0xD9, 0x06, 0x09, 0x08, 0x00, 0x02, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x15, 0x01};
static const char GPS_CONFIG_B115200[16]        = {0xF1, 0xD9, 0x06, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0xD1, 0xE0};
static const char GPS_CONFIG_B9600[16]          = {0xF1, 0xD9, 0x06, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x25, 0x00, 0x00, 0xB3, 0x07};
static const char GPS_ENABLE_ZDA[11]            = {0xF1, 0xD9, 0x06, 0x01, 0x03, 0x00, 0xF0, 0x07, 0x01, 0x02};

#define GPS_DEV_SLOW_UPDATE_RATE (10)
#define GPS_DEV_HIGH_UPDATE_RATE (1)

#define GPS_DEV_LOW_BAUD  (B9600)
#define GPS_DEV_HIGH_BAUD (B115200)

static void gps_nmea_thread( void*  arg );
static void gps_timer_thread( void*  arg );

static void gps_state_lock_fix(GpsState *state) {
    int ret;

    do { ret=sem_wait(&state->fix_sem); }
    while (ret < 0 && errno == EINTR);

    if (ret < 0) {
        E("Error in GPS state lock:%s\n", strerror(errno));
    }
}

static void gps_state_unlock_fix(GpsState *state) {
    if (sem_post(&state->fix_sem) == -1) {
        if(errno == EAGAIN) {
            if(sem_post(&state->fix_sem)== -1) {
                E("Error in GPS state unlock:%s\n", strerror(errno));
            }
        }
    }
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   T O K E N I Z E R                     *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

typedef struct {
    const char*  p;
    const char*  end;
} Token;

#define  MAX_NMEA_TOKENS  32

typedef struct {
    int     count;
    Token   tokens[ MAX_NMEA_TOKENS ];
} NmeaTokenizer;

static int
nmea_tokenizer_init( NmeaTokenizer*  t, const char*  p, const char*  end )
{
    int    count = 0;
    char*  q;

    // the initial '$' is optional
    if (p < end && p[0] == '$')
        p += 1;

    // remove trailing newline
    if (end > p && end[-1] == '\n') {
        end -= 1;
        if (end > p && end[-1] == '\r')
            end -= 1;
    }

    // get rid of checksum at the end of the sentecne
    if (end >= p+3 && end[-3] == '*') {
        end -= 3;
    }

    while (p < end) {
        const char*  q = p;

        q = memchr(p, ',', end-p);
        if (q == NULL)
            q = end;

        if (count < MAX_NMEA_TOKENS) {
            t->tokens[count].p   = p;
            t->tokens[count].end = q;
            count += 1;
        }

        if (q < end)
            q += 1;

        p = q;
    }

    t->count = count;
    return count;
}

static Token
nmea_tokenizer_get( NmeaTokenizer*  t, int  index )
{
    Token  tok;
    static const char*  dummy = "";

    if (index < 0 || index >= t->count) {
        tok.p = tok.end = dummy;
    } else
        tok = t->tokens[index];

    return tok;
}


static int
str2int( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p;

    if (len == 0) {
      return -1;
    }

    for ( ; len > 0; len--, p++ )
    {
        int  c;

        if (p >= end)
            goto Fail;

        c = *p - '0';
        if ((unsigned)c >= 10)
            goto Fail;

        result = result*10 + c;
    }
    return  result;

Fail:
    return -1;
}

static double
str2float( const char*  p, const char*  end )
{
    int   result = 0;
    int   len    = end - p + 1;
    char  temp[32];

    if (len == 0) {
      return -1.0;
    }

    if (len >= (int)sizeof(temp))
        return 0.;

    memcpy( temp, p, len );
    temp[len] = 0;
    return strtod( temp, NULL );
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       N M E A   P A R S E R                           *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

static void nmea_reader_update_utc_diff( NmeaReader*  r )
{
    time_t         now = time(NULL);
    struct tm      tm_local;
    struct tm      tm_utc;
    long           time_local, time_utc;

    gmtime_r( &now, &tm_utc );
    localtime_r( &now, &tm_local );

    time_local = tm_local.tm_sec +
                 60*(tm_local.tm_min +
                 60*(tm_local.tm_hour +
                 24*(tm_local.tm_yday +
                 365*tm_local.tm_year)));

    time_utc = tm_utc.tm_sec +
               60*(tm_utc.tm_min +
               60*(tm_utc.tm_hour +
               24*(tm_utc.tm_yday +
               365*tm_utc.tm_year)));

	r->utc_diff = time_utc - time_local;
}


static void nmea_reader_init( NmeaReader*  r )
{
    D("nmea_reader_init");

    memset( r, 0, sizeof(*r) );

    r->pos      = 0;
    r->overflow = 0;
    r->utc_year = -1;
    r->utc_mon  = -1;
    r->utc_day  = -1;
    r->fix.size = sizeof(r->fix);

    nmea_reader_update_utc_diff( r );
}

static int nmea_reader_get_timestamp(NmeaReader*  r, Token  tok, time_t *timestamp)
{
    int        hour, minute;
    double     seconds;
    struct tm  tm;
    time_t     ttime;

    if (tok.p + 6 > tok.end)
        return -1;

    if (r->utc_year < 0) {
        return -1;
    }

    hour    = str2int(tok.p,   tok.p+2);
    minute  = str2int(tok.p+2, tok.p+4);
    seconds = str2float(tok.p+4, tok.end);

    tm.tm_hour = hour;
    tm.tm_min  = minute;
    tm.tm_sec   = (int) seconds;
    tm.tm_year = r->utc_year - 1900;
    tm.tm_mon  = r->utc_mon - 1;
    tm.tm_mday = r->utc_day;
    tm.tm_isdst = -1;

    D("h: %d, m: %d, s: %d", tm.tm_hour, tm.tm_min, tm.tm_sec);
    D("Y: %d, M: %d, D: %d", tm.tm_year, tm.tm_mon, tm.tm_mday);

	nmea_reader_update_utc_diff(r);

	ttime = mktime( &tm );
	*timestamp = ttime - r->utc_diff;

    return 0;
}

static int
nmea_reader_update_time( NmeaReader*  r, Token  tok )
{
    time_t timestamp = 0;
    int ret = nmea_reader_get_timestamp( r, tok, &timestamp);
    if (0 == ret)
        r->fix.timestamp = (long long)timestamp * 1000;
    return ret;
}

static int
nmea_reader_update_cdate( NmeaReader*  r, Token  tok_d, Token tok_m, Token tok_y )
{

    if ( (tok_d.p + 2 > tok_d.end) ||
         (tok_m.p + 2 > tok_m.end) ||
         (tok_y.p + 4 > tok_y.end) )
        return -1;

    r->utc_day = str2int(tok_d.p,   tok_d.p+2);
    r->utc_mon = str2int(tok_m.p, tok_m.p+2);
    r->utc_year = str2int(tok_y.p, tok_y.end+4);

    return 0;
}

static int
nmea_reader_update_date( NmeaReader*  r, Token  date, Token  mtime )
{
    Token  tok = date;
    int    day, mon, year;

    if (tok.p + 6 != tok.end) {
        E("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        /* no date info, will use host time in _update_time function
         */
    }
    /* normal case */
    day  = str2int(tok.p, tok.p+2);
    mon  = str2int(tok.p+2, tok.p+4);
    year = str2int(tok.p+4, tok.p+6) + 2000;

    if ((day|mon|year) < 0) {
        E("date not properly formatted: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }

    r->utc_year  = year;
    r->utc_mon   = mon;
    r->utc_day   = day;

    return nmea_reader_update_time( r, mtime );
}


static double
convert_from_hhmm( Token  tok )
{
    double  val     = str2float(tok.p, tok.end);
    int     degrees = (int)(floor(val) / 100);
    double  minutes = val - degrees*100.;
    double  dcoord  = degrees + minutes / 60.0;
    return dcoord;
}


static int
nmea_reader_update_latlong( NmeaReader*  r,
                            Token        latitude,
                            char         latitudeHemi,
                            Token        longitude,
                            char         longitudeHemi )
{
    double   lat, lon;
    Token    tok;

    tok = latitude;
    if (tok.p + 6 > tok.end) {
        E("latitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lat = convert_from_hhmm(tok);
    if (latitudeHemi == 'S')
        lat = -lat;

    tok = longitude;
    if (tok.p + 6 > tok.end) {
        E("longitude is too short: '%.*s'", tok.end-tok.p, tok.p);
        return -1;
    }
    lon = convert_from_hhmm(tok);
    if (longitudeHemi == 'W')
        lon = -lon;

    r->fix.flags    |= GPS_LOCATION_HAS_LAT_LONG;
    r->fix.latitude  = lat;
    r->fix.longitude = lon;
    return 0;
}


static int
nmea_reader_update_altitude( NmeaReader*  r,
                             Token        altitude,
                             Token        units )
{
    double  alt;
    Token   tok = altitude;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_ALTITUDE;
    r->fix.altitude = str2float(tok.p, tok.end);
    return 0;
}

static int
nmea_reader_update_accuracy( NmeaReader*  r,
                             Token        accuracy )
{
    double  acc;
    Token   tok = accuracy;

    if (tok.p >= tok.end)
        return -1;

    //tok is cep*cc, we only want cep
    r->fix.accuracy = str2float(tok.p, tok.end);

    if (r->fix.accuracy == 99.99){
      return 0;
    }

    r->fix.flags   |= GPS_LOCATION_HAS_ACCURACY;
    return 0;
}

static int
nmea_reader_update_bearing( NmeaReader*  r,
                            Token        bearing )
{
    double  alt;
    Token   tok = bearing;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_BEARING;
    r->fix.bearing  = str2float(tok.p, tok.end);
    return 0;
}


static int
nmea_reader_update_speed( NmeaReader*  r,
                          Token        speed )
{
    double  alt;
    Token   tok = speed;

    if (tok.p >= tok.end)
        return -1;

    r->fix.flags   |= GPS_LOCATION_HAS_SPEED;
    r->fix.speed    = str2float(tok.p, tok.end);
//    r->fix.speed   *= 0.514444;    // fix for Speed Unit form Knots to Meters per Second
    return 0;
}

static int
nmea_reader_update_timemap( NmeaReader* r,
                            Token       systime_tok,
                            Token       timestamp_tok)
{
    int ret;
    time_t timestamp;

    if ( systime_tok.p >= systime_tok.end ||
         timestamp_tok.p >= timestamp_tok.end)
    {
        r->timemap.valid = 0;
        return -1;
    }

    ret = nmea_reader_get_timestamp(r, timestamp_tok, &timestamp);
    if (ret)
    {
        r->timemap.valid = 0;
        return ret;
    }

    r->timemap.valid = 1;
    r->timemap.systime = str2float(systime_tok.p, systime_tok.end);
    r->timemap.timestamp = (GpsUtcTime)((long long)timestamp * 1000);
    return 0;
}

static void
nmea_reader_parse( NmeaReader*  r )
{
   /* we received a complete sentence, now parse it to generate
    * a new GPS fix...
    */
    NmeaTokenizer  tzer[1];
    Token          tok;
    bool           isBDMsg;

    D("Received: '%.*s'", r->pos, r->in);
    if (r->pos < 9) {
        E("Too short. discarded.");
        return;
    }

    if (gps_state->callbacks.nmea_cb) {
        struct timeval tv;
        unsigned long long mytimems;
        gettimeofday(&tv,NULL);
        mytimems = tv.tv_sec * 1000 + tv.tv_usec / 1000;
        gps_state->callbacks.nmea_cb(mytimems, r->in, r->pos);
    }

    nmea_tokenizer_init(tzer, r->in, r->in + r->pos);
#ifdef GPS_DEBUG_TOKEN
    {
        int  n;
        D("Found %d tokens", tzer->count);
        for (n = 0; n < tzer->count; n++) {
            Token  tok = nmea_tokenizer_get(tzer,n);
            D("%2d: '%.*s'", n, tok.end-tok.p, tok.p);
        }
    }
#endif

    tok = nmea_tokenizer_get(tzer, 0);
    if (tok.p + 5 > tok.end) {
        E("sentence id '%.*s' too short, ignored.", tok.end-tok.p, tok.p);
        return;
    }

    isBDMsg = !memcmp(tok.p, "BD", 2);

    // ignore first two characters.
    tok.p += 2;

    if ( !memcmp(tok.p, "GGA", 3) ) {
        // GPS fix
        Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);

        if (tok_fixstaus.p[0] > '0') {

            Token  tok_time          = nmea_tokenizer_get(tzer,1);
            Token  tok_latitude      = nmea_tokenizer_get(tzer,2);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,3);
            Token  tok_longitude     = nmea_tokenizer_get(tzer,4);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,5);
            Token  tok_altitude      = nmea_tokenizer_get(tzer,9);
            Token  tok_altitudeUnits = nmea_tokenizer_get(tzer,10);

            nmea_reader_update_time(r, tok_time);
            nmea_reader_update_latlong(r, tok_latitude,
                                        tok_latitudeHemi.p[0],
                                        tok_longitude,
                                        tok_longitudeHemi.p[0]);
            nmea_reader_update_altitude(r, tok_altitude, tok_altitudeUnits);
        }
    } else if ( !memcmp(tok.p, "GLL", 3) ) {
            Token  tok_fixstaus      = nmea_tokenizer_get(tzer,6);

            if (tok_fixstaus.p[0] == 'A') {

            Token  tok_latitude      = nmea_tokenizer_get(tzer,1);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,2);
            Token  tok_longitude     = nmea_tokenizer_get(tzer,3);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,4);
            Token  tok_time          = nmea_tokenizer_get(tzer,5);

            nmea_reader_update_time(r, tok_time);
            nmea_reader_update_latlong(r, tok_latitude,
                                        tok_latitudeHemi.p[0],
                                        tok_longitude,
                                        tok_longitudeHemi.p[0]);
            }
    } else if ( !memcmp(tok.p, "GSA", 3) ) {
        Token  tok_fixStatus   = nmea_tokenizer_get(tzer, 2);
        int i;

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != '1') {

            Token tok_accuracy = nmea_tokenizer_get(tzer, 15);
            nmea_reader_update_accuracy(r, tok_accuracy);

            if (!isBDMsg) {
                r->sv_status.used_in_fix_mask = 0ul;
            }

            for (i = 3; i <= 14; ++i){
                Token  tok_prn  = nmea_tokenizer_get(tzer, i);
                int prn = str2int(tok_prn.p, tok_prn.end);

                /* only available for PRN 1-32 */
                D("isBDMsg: %d, prn:%d", isBDMsg, prn);
                if ((prn > 0) && (prn < 33)){
                    r->sv_status.used_in_fix_mask |= (1ul << (prn-1));
                    /* mark this parameter to identify the GSA is in fixed state */
                    r->gsa_fixed = 1;
                    if (isBDMsg) {
                        r->sv_status_changed = 1;
                    }
                }
            }
        } else {
            if ( isBDMsg ) {
                r->sv_status_changed = 1;
            } else {
                if ( r->gsa_fixed ) {
                    r->sv_status.used_in_fix_mask = 0ul;
                    r->gsa_fixed = 0;
                }
            }
        }
    } else if ( !memcmp(tok.p, "GSV", 3) ) {
        Token  tok_noSatellites  = nmea_tokenizer_get(tzer, 3);
        int    noSatellites = str2int(tok_noSatellites.p, tok_noSatellites.end);

        if (noSatellites > 0) {

            Token  tok_noSentences   = nmea_tokenizer_get(tzer, 1);
            Token  tok_sentence      = nmea_tokenizer_get(tzer, 2);

            int sentence = str2int(tok_sentence.p, tok_sentence.end);
            int totalSentences = str2int(tok_noSentences.p, tok_noSentences.end);
            int curr;
            int i;

            if ((sentence == 1) && (!isBDMsg)) {
                r->sv_status_changed = 0;
                r->sv_status.num_svs = 0;
            }

            if ((sentence == 1) && isBDMsg) {
                noSatellites += r->sv_status.num_svs;
            }
            D("noSatellites: %d", noSatellites);

            curr = r->sv_status.num_svs;

            i = 0;

            while (i < 4 && r->sv_status.num_svs < noSatellites){

                Token  tok_prn = nmea_tokenizer_get(tzer, i * 4 + 4);
                Token  tok_elevation = nmea_tokenizer_get(tzer, i * 4 + 5);
                Token  tok_azimuth = nmea_tokenizer_get(tzer, i * 4 + 6);
                Token  tok_snr = nmea_tokenizer_get(tzer, i * 4 + 7);

                r->sv_status.sv_list[curr].prn = str2int(tok_prn.p, tok_prn.end);
                r->sv_status.sv_list[curr].elevation = str2float(tok_elevation.p, tok_elevation.end);
                r->sv_status.sv_list[curr].azimuth = str2float(tok_azimuth.p, tok_azimuth.end);
                r->sv_status.sv_list[curr].snr = str2float(tok_snr.p, tok_snr.end);

                if ( ++r->sv_status.num_svs > GPS_MAX_SVS ) {
                    D("sv_status.num_svs: %d", r->sv_status.num_svs);
                    r->sv_status.num_svs = GPS_MAX_SVS;
                }

                if ( ++curr > GPS_MAX_SVS ) {
                    D("curr: %d", curr);
                    curr = GPS_MAX_SVS;
                }

                i += 1;
            }

            if ((sentence == totalSentences) && isBDMsg) {
                r->sv_status_changed = 1;
            }
        }
    } else if ( !memcmp(tok.p, "RMC", 3) ) {
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,2);

        if (tok_fixStatus.p[0] == 'A') {
            Token  tok_time          = nmea_tokenizer_get(tzer,1);
            Token  tok_latitude      = nmea_tokenizer_get(tzer,3);
            Token  tok_latitudeHemi  = nmea_tokenizer_get(tzer,4);
            Token  tok_longitude     = nmea_tokenizer_get(tzer,5);
            Token  tok_longitudeHemi = nmea_tokenizer_get(tzer,6);
            Token  tok_speed         = nmea_tokenizer_get(tzer,7);
            Token  tok_bearing       = nmea_tokenizer_get(tzer,8);
            Token  tok_date          = nmea_tokenizer_get(tzer,9);

            nmea_reader_update_date( r, tok_date, tok_time );

            nmea_reader_update_latlong( r, tok_latitude,
                                           tok_latitudeHemi.p[0],
                                           tok_longitude,
                                           tok_longitudeHemi.p[0] );

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }
    } else if ( !memcmp(tok.p, "VTG", 3) ) {
        Token  tok_fixStatus     = nmea_tokenizer_get(tzer,9);

        if (tok_fixStatus.p[0] != '\0' && tok_fixStatus.p[0] != 'N') {
            Token  tok_bearing       = nmea_tokenizer_get(tzer,1);
            Token  tok_speed         = nmea_tokenizer_get(tzer,5);

            nmea_reader_update_bearing( r, tok_bearing );
            nmea_reader_update_speed  ( r, tok_speed );
        }
    } else if ( !memcmp(tok.p, "ZDA", 3) ) {
        Token  tok_time;
        Token  tok_year  = nmea_tokenizer_get(tzer,4);

        if (tok_year.p[0] != '\0') {

            Token  tok_day   = nmea_tokenizer_get(tzer,2);
            Token  tok_mon   = nmea_tokenizer_get(tzer,3);

            nmea_reader_update_cdate( r, tok_day, tok_mon, tok_year );
        }

        tok_time  = nmea_tokenizer_get(tzer,1);

        if (tok_time.p[0] != '\0') {
            nmea_reader_update_time(r, tok_time);
        }
    } else {
        tok.p -= 2;
        E("unknown sentence '%.*s", tok.end-tok.p, tok.p);
    }

#if 0
    if (r->fix.flags != 0) {
        char temp[256];
        char *p = temp;
        char *end = p + sizeof(temp);
        struct tm utc;
        p += snprintf( p, end-p, "sending fix" );
        if (r->fix.flags & GPS_LOCATION_HAS_LAT_LONG) {
            p += snprintf(p, end-p, " lat=%g lon=%g", r->fix.latitude, r->fix.longitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ALTITUDE) {
            p += snprintf(p, end-p, " altitude=%g", r->fix.altitude);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_SPEED) {
            p += snprintf(p, end-p, " speed=%g", r->fix.speed);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_BEARING) {
            p += snprintf(p, end-p, " bearing=%g", r->fix.bearing);
        }
        if (r->fix.flags & GPS_LOCATION_HAS_ACCURACY) {
            p += snprintf(p,end-p, " accuracy=%g", r->fix.accuracy);
        }
        gmtime_r( (time_t*) &r->fix.timestamp, &utc );
        p += snprintf(p, end-p, " time=%s", asctime( &utc ) );
        ALOGD("%s",temp);
    }
#endif
}

static void
nmea_reader_addc( NmeaReader*  r, int  c )
{
    int cnt;

    if (r->overflow) {
        r->overflow = (c != '\n');
        return;
    }

    if (r->pos >= (int) sizeof(r->in)-1 ) {
        r->overflow = 1;
        r->pos      = 0;
        return;
    }

    r->in[r->pos] = (char)c;
    r->pos       += 1;

    if (c == '\n') {
        gps_state_lock_fix(gps_state);
        nmea_reader_parse( r );
        gps_state_unlock_fix(gps_state);
        r->pos = 0;
    }
}

/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       C O N N E C T I O N   S T A T E                 *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/

/* commands sent to the gps thread */
enum {
    CMD_QUIT  = 0,
    CMD_START = 1,
    CMD_STOP  = 2
};

static void
gps_state_done( GpsState*  s )
{
    // tell the thread to quit, and wait for it
    char   cmd = CMD_QUIT;
    void*  dummy;
    int ret;

	D("%s: called", __FUNCTION__);

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    D("gps waiting for command thread to stop");
    pthread_join(s->thread, &dummy);

    /* Timer thread depends on this state check */
    s->init = STATE_QUIT;
    s->fix_freq = -1;

    // close the control socket pair
    close( s->control[0] ); s->control[0] = -1;
    close( s->control[1] ); s->control[1] = -1;

    // close connection to the GPS daemon
    if (s->fd != -1) {
        close( s->fd );
        s->fd = -1;
    }

    sem_destroy(&s->fix_sem);
    g_gpscallback = NULL;
}

static void
gps_state_start( GpsState*  s )
{
    char  cmd = CMD_START;
    int   ret;

	D("%s: called", __FUNCTION__);

    do { ret=write( s->control[0], &cmd, 1 ); }
    while (ret < 0 && errno == EINTR);

    if (ret != 1) {
        E("%s: could not send CMD_START command: ret=%d: %s",
            __FUNCTION__, ret, strerror(errno));
    }
}


static void
gps_state_stop( GpsState*  s )
{
    char  cmd = CMD_STOP;
    int   ret;

	D("%s: called", __FUNCTION__);

    do { ret=write( s->control[0], &cmd, 1 ); }
	while (ret < 0 && errno == EINTR);

    if (ret != 1) {
        E("%s: could not send CMD_STOP command: ret=%d: %s",
            __FUNCTION__, ret, strerror(errno));
    }
}


static int
epoll_register( int  epoll_fd, int  fd )
{
    struct epoll_event  ev;
    int                 ret, flags;

    /* important: make the fd non-blocking */
    flags = fcntl(fd, F_GETFL);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);

    ev.events  = EPOLLIN;
    ev.data.fd = fd;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_ADD, fd, &ev );
    } while (ret < 0 && errno == EINTR);
    return ret;
}


static int
epoll_deregister( int  epoll_fd, int  fd )
{
    int  ret;
    do {
        ret = epoll_ctl( epoll_fd, EPOLL_CTL_DEL, fd, NULL );
    } while (ret < 0 && errno == EINTR);
    return ret;
}

/* this is the main thread, it waits for commands from gps_state_start/stop and,
 * when started, messages from the QEMU GPS daemon. these are simple NMEA sentences
 * that must be parsed to be converted into GPS fixes sent to the framework
 */
static int         epoll_ctrlfd ;
static int         epoll_nmeafd ;

static void
gps_state_thread( void*  arg )
{
    GpsState*   state = (GpsState*) arg;
    int         gps_fd     = state->fd;
    int         control_fd = state->control[1];
	epoll_ctrlfd   = epoll_create(1);
	epoll_nmeafd   = epoll_create(1);

    // register control file descriptors for polling
    epoll_register( epoll_ctrlfd, control_fd );

    D("gps thread running");

	state->tmr_thread = state->callbacks.create_thread_cb("locosys_gps_tmr", gps_timer_thread, state);
	if (!state->tmr_thread) {
		ALOGE("could not create gps timer thread: %s", strerror(errno));
		started = 0;
		state->init = STATE_INIT;
		goto Exit;
	}

	state->nmea_thread = state->callbacks.create_thread_cb("locosys_nmea_thread", gps_nmea_thread, state);
	if (!state->nmea_thread) {
		ALOGE("could not create gps nmea thread: %s", strerror(errno));
		started = 0;
		state->init = STATE_INIT;
		goto Exit;
	}

	started = 0;
	state->init = STATE_INIT;

    // now loop
    for (;;) {
        struct epoll_event   events[1];
        int                  ne, nevents;

        nevents = epoll_wait( epoll_ctrlfd, events, 1, -1 );
        if (nevents < 0) {
            if (errno != EINTR) {
                ALOGE("epoll_wait() unexpected error: %s", strerror(errno));
            }
            continue;
        }

        D("gps thread received %d events", nevents);
        for (ne = 0; ne < nevents; ne++) {
            if ((events[ne].events & (EPOLLERR|EPOLLHUP)) != 0) {
                ALOGE("EPOLLERR or EPOLLHUP after epoll_wait() !?");
                goto Exit;
            }
            if ((events[ne].events & EPOLLIN) != 0) {
                int  fd = events[ne].data.fd;

                if (fd == control_fd) {
                    char  cmd = 255;
                    int   ret;

                    D("gps control fd event");
                    do {
                        ret = read( fd, &cmd, 1 );
                    } while (ret < 0 && errno == EINTR);

                    if (cmd == CMD_QUIT) {
                        D("gps thread quitting on demand");
                        goto Exit;
                    } else if (cmd == CMD_START) {
                        if (!started) {
							NmeaReader  *reader;
							reader = &state->reader;
							nmea_reader_init( reader );
                            D("gps thread starting  location_cb=%p", state->callbacks.location_cb);
                            started = 1;
                            state->init = STATE_START;
                        } else {
							D("LM already start");
                        }
                    } else if (cmd == CMD_STOP) {
                        if (started) {
                            started = 0;
                            state->init = STATE_INIT;
                       }
                    }
                } else {
                    ALOGE("epoll_wait() returned unkown fd %d ?", fd);
					gps_fd = _gps_state->fd; //resign fd to gps_fd
                }
            }
        }
    }
Exit:
	{
		void *dummy;
        continue_thread = 0;
		close(epoll_ctrlfd);
		close(epoll_nmeafd);
		pthread_join(state->tmr_thread, &dummy);
		pthread_join(state->nmea_thread, &dummy);
		D("gps control thread destroyed");
	}
    return;
}

static void
gps_nmea_thread( void*  arg )
{
	GpsState *state = (GpsState *)arg;
	NmeaReader  *reader;
    reader = &state->reader;

	D("gps entered nmea thread");

    // now loop
    while (continue_thread) {
		char buf[512];
		int  nn, ret;
		struct timeval tv;

        if ( (!started) || (state->fd == -1) ) {
            sleep(1);
            continue;
        }

        if (!continue_thread) {
            break;
        }

		fd_set readfds;
		FD_ZERO(&readfds);
		FD_SET(state->fd, &readfds);
		/* Wait up to 100 ms. */
		tv.tv_sec = 0;
		tv.tv_usec = 100;

		ret = select(state->fd + 1, &readfds, NULL, NULL, &tv);

		if (FD_ISSET(state->fd, &readfds)) {
			memset(buf,0,sizeof(buf));
			ret = read( state->fd, buf, sizeof(buf) );
			if (ret > 0) {
				for (nn = 0; nn < ret; nn++) {
					nmea_reader_addc( reader, buf[nn] );
                }
			} else {
				E("Error on NMEA read :%s",strerror(errno));
                if (errno == EINTR) {
                    continue;
                }
			}
		}
	}
Exit:
	D("gps nmea thread destroyed");
	return;
}

static void
gps_timer_thread( void*  arg )
{
    int need_sleep = 0;
    int sleep_val = 0;

    GpsState *state = (GpsState *)arg;

    D("gps entered timer thread");

    do {
        while( (started != 1) && continue_thread ) {
            sleep(1);
        }

        gps_state_lock_fix(state);

        if ((state->reader.fix.flags & GPS_LOCATION_HAS_LAT_LONG) != 0) {

            D("gps fix cb: 0x%x", state->reader.fix.flags);

            if (state->callbacks.location_cb) {
                state->callbacks.location_cb( &state->reader.fix );
                state->reader.fix.flags = 0;
            }

            if (state->fix_freq == 0) {
                state->fix_freq = -1;
            }
        }

        if (state->reader.sv_status_changed != 0) {

            D("gps sv status callback");

            if (state->callbacks.sv_status_cb) {
                state->callbacks.sv_status_cb( &state->reader.sv_status );
                state->reader.sv_status_changed = 0;
            }
        }

        need_sleep = (state->fix_freq != -1 && (state->init != STATE_QUIT) ? 1 : 0);
        sleep_val = state->fix_freq;

        gps_state_unlock_fix(state);

        if (need_sleep) {
            sleep(sleep_val);
        } else {
            E("won't sleep because fix_freq=%d state->init=%d",state->fix_freq, state->init);
        }
    } while(continue_thread);

    D("gps timer thread destroyed");

    return;
}

static char   prop[PROPERTY_VALUE_MAX] = "/dev/ttymxc2";

int gps_opentty(GpsState *state)
{
    ssize_t ret;

	D("%s: called", __FUNCTION__);

	if(state->fd != -1) {
		gps_closetty(state);
    }

    do {
        state->fd = open( prop, O_RDWR | O_NOCTTY | O_NONBLOCK);
    } while (state->fd < 0 && errno == EINTR);

    if (state->fd < 0) {
        ALOGE("could not open gps serial device %s: %s", prop, strerror(errno) );
        return -1;
    }

    D("gps will read from %s", prop);

    // disable echo on serial lines
    if ( isatty( state->fd ) ) {
        D("gps disable echo on serial lines");
        struct termios  ios;
        tcgetattr( state->fd, &ios );
        bzero(&ios, sizeof(ios));
        ios.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
        ios.c_iflag = IGNPAR;
        ios.c_oflag = 0;
        ios.c_lflag = 0;  /* disable ECHO, ICANON, etc... */
        tcsetattr( state->fd, TCSANOW, &ios );
		tcflush(state->fd,TCIOFLUSH);
    }

	epoll_register( epoll_nmeafd, state->fd );

	return 0;
}

void gps_closetty(GpsState *s)
{
    D("%s: called", __FUNCTION__);

    if(s->fd != -1) {
        // close connection to the QEMU GPS daemon
        epoll_deregister( epoll_nmeafd, s->fd );
        close( s->fd );
        s->fd = -1;
    }
}

static void
gps_state_init( GpsState*  state )
{
    int    ret;
    int    done = 0;

	D("%s: called", __FUNCTION__);

    state->init       = STATE_INIT;
    state->control[0] = -1;
    state->control[1] = -1;
    state->fd         = -1;

    if (sem_init(&state->fix_sem, 0, 1) != 0) {
        E("gps semaphore initialization failed! errno = %d", errno);
        return;
    }

    if (property_get("ro.kernel.android.gps", prop, "/dev/ttymxc2") == 0) {
        E("no kernel-provided gps device name (not hosted)");
        E("please set ro.kernel.android.gps property");
        return;
    }

    if ( socketpair( AF_LOCAL, SOCK_STREAM, 0, state->control ) < 0 ) {
        E("could not create thread control socket pair: %s", strerror(errno));
        goto Fail;
    }

	state->thread = state->callbacks.create_thread_cb("locosys_gps", gps_state_thread, state);
    if (!state->thread) {
        E("could not create gps thread: %s", strerror(errno));
        goto Fail;
    }

    state->callbacks.set_capabilities_cb(GPS_CAPABILITY_SCHEDULING);

	D("gps state initialized");
    return;

Fail:
    gps_state_done( state );
}


/*****************************************************************/
/*****************************************************************/
/*****                                                       *****/
/*****       I N T E R F A C E                               *****/
/*****                                                       *****/
/*****************************************************************/
/*****************************************************************/
static int
locosys_gps_init(GpsCallbacks* callbacks)
{
    GpsState*  s = _gps_state;

	D("gps state initializing %d",s->init);

    s->callbacks = *callbacks;
    if (!s->init) {
        gps_state_init(s);
    }

    if (!g_gpscallback) {
        g_gpscallback = callbacks;
    }

    GPS_STATUS_CB(s->callbacks, GPS_STATUS_ENGINE_ON);

    return 0;
}

static void
locosys_gps_cleanup(void)
{
    GpsState*  s = _gps_state;

	D("%s: called", __FUNCTION__);

    if (s->init) {
        gps_state_done(s);
    }

    GPS_STATUS_CB(s->callbacks, GPS_STATUS_ENGINE_OFF);
}

static int
locosys_gps_start()
{
    GpsState*  s = _gps_state;

	D("%s: called", __FUNCTION__ );

	if(gps_checkstate(s) == -1) {
		E("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
	}

    gps_state_lock_fix(s);
    gps_opentty(s);
    gps_state_unlock_fix(s);
	gps_state_start(s);
    GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_BEGIN);

    return 0;
}


static int
locosys_gps_stop()
{
    GpsState*  s = _gps_state;

	D("%s: called", __FUNCTION__ );

	if(gps_checkstate(s) == -1) {
		E("%s: called with uninitialized state !!", __FUNCTION__);
		return -1;
	}

	//close LM first
    gps_state_lock_fix(s);
    gps_closetty(s);
    gps_state_unlock_fix(s);
	gps_state_stop(s);
	GPS_STATUS_CB(s->callbacks, GPS_STATUS_SESSION_END);
    return 0;
}

static int
locosys_gps_inject_time(GpsUtcTime time, int64_t timeReference, int uncertainty)
{
    return 0;
}

static void
locosys_gps_delete_aiding_data(GpsAidingData flags)
{
}

static int
locosys_gps_inject_location(double latitude, double longitude, float accuracy)
{
    return 0;
}

static int
locosys_gps_set_position_mode(GpsPositionMode mode, GpsPositionRecurrence recurrence,
        uint32_t min_interval, uint32_t preferred_accuracy, uint32_t preferred_time)
{
    GpsState* s = _gps_state;

    // only standalone is supported for now.
    if (mode != GPS_POSITION_MODE_STANDALONE) {
        E("%s: set GPS POSITION mode error! (mode:%d) ", __FUNCTION__, mode);
        E("Set as standalone mode currently! ");
    }

    if (!s->init) {
        E("%s: called with uninitialized state !!", __FUNCTION__);
        return -1;
    }

    s->fix_freq = min_interval/1000;
    if (0 == s->fix_freq) {
        s->fix_freq = 1;
    }

    D("gps fix frquency set to %d sec", s->fix_freq);

    return 0;
}

static const void*
locosys_gps_get_extension(const char* name)
{
    // no extensions supported
    return NULL;
}

int gps_checkstate(GpsState *s)
{
    if (!s->init) {
        if (g_gpscallback) {
            locosys_gps_init(g_gpscallback);
        }

        if (!s->init) {
            ALOGE("%s: still called with uninitialized state !!", __FUNCTION__);
            return -1;
        }
    }

    return 0;
}

static const GpsInterface  locosysGpsInterface = {
    .size =sizeof(GpsInterface),
    .init = locosys_gps_init,
    .start = locosys_gps_start,
    .stop = locosys_gps_stop,
    .cleanup = locosys_gps_cleanup,
    .inject_time = locosys_gps_inject_time,
    .inject_location = locosys_gps_inject_location,
    .delete_aiding_data = locosys_gps_delete_aiding_data,
    .set_position_mode = locosys_gps_set_position_mode,
    .get_extension = locosys_gps_get_extension,
};

const GpsInterface* gps_get_hardware_interface()
{
    return &locosysGpsInterface;
}

