/*
 * simple sketch for testing the speed and accuracy of the bearing offset functions
 * send the sketch a valid NMEA sentence via the serial port, then optionally:
 * update distance by sending: d10000 (number denotes distance)
 * update the angular step by sending: a5 (number denotes angle in degrees)
 * then run the test by sending: CALC
 * output can either be NMEA sentences for plotting using various tools such as http://nmeagen.org
 * or lat and long as decimal numbers
 */

#include <TinyGPS++.h>
#include <Gps64.h>

TinyGPSPlus gps;

void setup() {
	Serial.begin(57600);
	Serial.println("Started");
}

char scratch;
char buf[128];
uint8_t index;

void loop() {
	while (Serial.available() > 0) {
		scratch = Serial.read();
		buf[index] = scratch;
		gps.encode(scratch);
		index++;
		if ((scratch == '\r') || (scratch == '\n')) {
			buf[index] = (char)0;
			check_command();
			index = 0;
		};
		if (index > 127) {
			index = 0;
		}
	};
	digitalWrite(LED_BUILTIN, gps.location.isValid());
};

double distance = 1000000;
double angle_step = 10;

void check_command() {
	if (buf[0] == 'd') {
		distance = atol(&buf[1]);
	} else if (buf[0] == 'a') {
		angle_step = atol(&buf[1]);
	} else if (strncmp(buf, "CALC", 4) == 0) {
		calculate();
	}
}

GPS_DATA fix_lat;
GPS_DATA fix_lon;

GPS_DATA dst_lat;
GPS_DATA dst_lon;


void calculate() {
	if (gps.location.isValid()) {
		fix_lat.From(gps.location.rawLat());
		fix_lon.From(gps.location.rawLng());
//		Serial.println(distance);
//		Serial.println(angle_step);
//		Serial.print("Centre,");
	
		generate_sentence(fix_lat, fix_lon);
//		generate_output(fix_lat, fix_lon);
		uint32_t start = millis();
		for (double angle=0; angle<=360; angle += angle_step) {
			desto(fix_lat, fix_lon, angle, distance, dst_lat, dst_lon);
/*			Serial.print("lat in:"); Serial.print(fix_lat.deg); Serial.println(fix_lat.minSec);
			Serial.print("lon in:"); Serial.print(fix_lon.deg); Serial.println(fix_lon.minSec);

			Serial.print("lat out:"); Serial.print(dst_lat.deg); Serial.println(dst_lat.minSec);
			Serial.print("lon out:"); Serial.print(dst_lon.deg); Serial.println(dst_lon.minSec);//*/
//			Serial.print(angle);
//			Serial.print(',');
			generate_sentence(dst_lat, dst_lon);
//			generate_output(dst_lat, dst_lon);
		}
		uint32_t stop = millis();
		Serial.print("time taken:");
		Serial.println(stop-start);
	}
}

bool gpsdata_to_dms(GPS_DATA data, int16_t &degs, int16_t &mins, int16_t &secs) {
	degs = (data.deg >= 0) ? data.deg : data.deg*-1; // remove negative
	mins = ((data.minSec/100) * 6) / 1000000;
	secs = (((data.minSec/100) * 6) / 1000) - (mins * 1000);
	if (mins < 0) mins = mins*-1; // remove negative
	if (secs < 0) secs = secs*-1; // remove negative
	return ((data.deg > 0) || ((data.deg == 0) && (data.minSec > 0))); // minSec can only be negative when degrees = 0
}

void generate_output(GPS_DATA latitude, GPS_DATA longitude) {
	int16_t lat_d, lat_m, lat_s, lon_d, lon_m, lon_s;
	char lat_h, lon_h;
	lat_h = gpsdata_to_dms(latitude, lat_d, lat_m, lat_s) ? 'N' : 'S';
	lon_h = gpsdata_to_dms(longitude, lon_d, lon_m, lon_s) ? 'E' : 'W';

	lat_s = lat_s / 10;
	lon_s = lon_s / 10;

	static char sentence_buf[64];
	memset(sentence_buf, 0, 64);
	sprintf(sentence_buf, "Latitude: %02u°%02u'%02u\"%c Longitude: %03u°%02u'%02u\"%c", lat_d, lat_m, lat_s, lat_h, lon_d, lon_m, lon_s, lon_h);
	Serial.println(sentence_buf);
}

int NMEAchecksum(const char *s) {
    int c = 0;
    while(*s)
        c ^= *s++;
    return c;
}

void generate_sentence(GPS_DATA latitude, GPS_DATA longitude) {
	int16_t lat_d, lat_m, lat_s, lon_d, lon_m, lon_s;
	char lat_h, lon_h;
	lat_h = gpsdata_to_dms(latitude, lat_d, lat_m, lat_s) ? 'N' : 'S';
	lon_h = gpsdata_to_dms(longitude, lon_d, lon_m, lon_s) ? 'E' : 'W';
	
	static uint8_t offset;
	static char sentence_buf[64];
	offset = 0;
	memset(sentence_buf, 0, 64);
	
	offset = sprintf(sentence_buf, "$GPGGA,012345,%02u%02u.%03u,%c,%03u%02u.%03u,%c,1,12,1.0,0.0,M,0.0,M,,", lat_d, lat_m, lat_s, lat_h, lon_d, lon_m, lon_s, lon_h);
	int chk = NMEAchecksum(&sentence_buf[1]); // offset to skip dollar sign
	offset += sprintf(&sentence_buf[offset], "*%02X", chk);
	Serial.println(sentence_buf);
}
