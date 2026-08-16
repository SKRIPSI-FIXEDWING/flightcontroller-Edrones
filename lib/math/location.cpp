
/*
 *  this module deals with calculations involving struct Location
 */
#include "AP_Math.h"
#include "location.h"
// #include <complex.h>

// return horizontal distance between two positions in cm
float get_horizontal_distance_cm(const Vector3f &origin, const Vector3f &destination)
{
    return norm2(destination.x-origin.x,destination.y-origin.y);
}

// return bearing in centi-degrees between two positions
float get_bearing_cd(const Vector3f &origin, const Vector3f &destination)
{
    float bearing = atan2f(destination.y-origin.y, destination.x-origin.x) * DEGX100;
    if (bearing < 0) {
        bearing += 36000.0f;
    }
    return bearing;
}

// return true when lat and lng are within range
bool check_lat(float lat)
{
    return fabsf(lat) <= 90;
}
bool check_lng(float lng)
{
    return fabsf(lng) <= 180;
}
bool check_lat(int32_t lat)
{
    return labs(lat) <= 90*1e7;
}
bool check_lng(int32_t lng)
{
    return labs(lng) <= 180*1e7;
}
bool check_latlng(float lat, float lng)
{
    return check_lat(lat) && check_lng(lng);
}
bool check_latlng(int32_t lat, int32_t lng)
{
    return check_lat(lat) && check_lng(lng);
}

