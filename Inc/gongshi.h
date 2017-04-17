#ifndef GONGSHI_H
#define GONGSHI_H


float getDistanceVer2(float lat1, char latitude1_flag,float lng1, char longitude1_flag, float lat2,char latitude2_flag, float lng2,char longitude2_flag);

float get_angle(float latitude1, char latitude1_flag,float longitude1, char longitude1_flag, float latitude2, char latitude2_flag, float longitude2, char longitude2_flag);
float getDistanceVer1(float lat1, float lng1, float lat2, float lng2);
float GetAngle(float longitude1, char longitude1_flag, float latitude1, char latitude1_flag,
                 float longitude2, char longitude2_flag, float latitude2, char latitude2_flag);
int sunriset( int year, int month, int day, float lon, float lat, 
                  float altit, int upper_limb, float *trise, float *tset ) ;
void moon_test(float dbLon,float dbLat,int year,int month,int day,float TimeZone);

#endif
