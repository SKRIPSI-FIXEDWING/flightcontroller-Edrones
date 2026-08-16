/*
 * Locations.cpp
 */

#include "Locations.h"

// #include <AP_AHRS/AP_AHRS.h>
// #include <AP_Terrain/AP_Terrain.h>

// AP_Terrain *Location::_terrain = nullptr;

/// constructors
Locations::Locations()
{
    zero();
}

const Locations definitely_zero{};
bool Locations::is_zero(void) const
{
    return !memcmp(this, &definitely_zero, sizeof(*this));
}

void Locations::zero(void)
{
    memset(this, 0, sizeof(*this));
}

Locations::Locations(int32_t latitude, int32_t longitude, int32_t alt_in_cm, AltFrame frame)
{
    zero();
    lat = latitude;
    lng = longitude;
    set_alt_cm(alt_in_cm, frame);
}

Locations::Locations(const Vector3f &ekf_offset_neu)
{
    // store alt and alt frame
    set_alt_cm(ekf_offset_neu.z, AltFrame::ABOVE_ORIGIN);

    // calculate lat, lon
    Locations ekf_origin;
    // if (AP::ahrs().get_origin(ekf_origin))
    // {
    //     lat = ekf_origin.lat;
    //     lng = ekf_origin.lng;
    //     offset(ekf_offset_neu.x / 100.0f, ekf_offset_neu.y / 100.0f);
    // }
}

void Locations::set_alt_cm(int32_t alt_cm, AltFrame frame)
{
    alt = alt_cm;
    relative_alt = false;
    terrain_alt = false;
    origin_alt = false;
    switch (frame)
    {
    case AltFrame::ABSOLUTE:
        // do nothing
        break;
    case AltFrame::ABOVE_HOME:
        relative_alt = true;
        break;
    case AltFrame::ABOVE_ORIGIN:
        origin_alt = true;
        break;
    case AltFrame::ABOVE_TERRAIN:
        // we mark it as a relative altitude, as it doesn't have
        // home alt added
        relative_alt = true;
        terrain_alt = true;
        break;
    }
}

// converts altitude to new frame
bool Locations::change_alt_frame(AltFrame desired_frame)
{
    int32_t new_alt_cm;
    if (!get_alt_cm(desired_frame, new_alt_cm))
    {
        return false;
    }
    set_alt_cm(new_alt_cm, desired_frame);
    return true;
}

// get altitude frame
Locations::AltFrame Locations::get_alt_frame() const
{
    if (terrain_alt)
    {
        return AltFrame::ABOVE_TERRAIN;
    }
    if (origin_alt)
    {
        return AltFrame::ABOVE_ORIGIN;
    }
    if (relative_alt)
    {
        return AltFrame::ABOVE_HOME;
    }
    return AltFrame::ABSOLUTE;
}

/// get altitude in desired frame
bool Locations::get_alt_cm(AltFrame desired_frame, int32_t &ret_alt_cm) const
{
    Locations::AltFrame frame = get_alt_frame();

    // shortcut if desired and underlying frame are the same
    if (desired_frame == frame)
    {
        ret_alt_cm = alt;
        return true;
    }

    // check for terrain altitude
    float alt_terr_cm = 0;
    if (frame == AltFrame::ABOVE_TERRAIN || desired_frame == AltFrame::ABOVE_TERRAIN)
    {
        return false;
    }

    // convert alt to absolute
    int32_t alt_abs = 0;
    switch (frame)
    {
    case AltFrame::ABSOLUTE:
        alt_abs = alt;
        break;
    case AltFrame::ABOVE_HOME:
        return false;
    case AltFrame::ABOVE_ORIGIN:
        return false;
    case AltFrame::ABOVE_TERRAIN:
        alt_abs = alt + alt_terr_cm;
        break;
    }

    // convert absolute to desired frame
    switch (desired_frame)
    {
    case AltFrame::ABSOLUTE:
        ret_alt_cm = alt_abs;
        return true;
    case AltFrame::ABOVE_HOME:
        return false;
    case AltFrame::ABOVE_ORIGIN:
        return false;
    case AltFrame::ABOVE_TERRAIN:
        ret_alt_cm = alt_abs - alt_terr_cm;
        return true;
    }
    return false;
}

bool Locations::get_vector_xy_from_origin_NE(Vector2f &vec_ne) const
{
    // Locations ekf_origin;
    // if (!AP::ahrs().get_origin(ekf_origin))
    // {
    //     return false;
    // }
    // vec_ne.x = (lat - ekf_origin.lat) * LATLON_TO_CM;
    // vec_ne.y = (lng - ekf_origin.lng) * LATLON_TO_CM * ekf_origin.longitude_scale();
    return true;
}

bool Locations::get_vector_from_origin_NEU(Vector3f &vec_neu) const
{
    // convert lat, lon
    Vector2f vec_ne;
    if (!get_vector_xy_from_origin_NE(vec_ne))
    {
        return false;
    }
    vec_neu.x = vec_ne.x;
    vec_neu.y = vec_ne.y;

    // convert altitude
    int32_t alt_above_origin_cm = 0;
    if (!get_alt_cm(AltFrame::ABOVE_ORIGIN, alt_above_origin_cm))
    {
        return false;
    }
    vec_neu.z = alt_above_origin_cm;

    return true;
}

// return distance in meters between two Locationss
float Locations::get_distance(const struct Locations &loc2) const
{
    float dlat = (float)(loc2.lat - lat);
    float dlng = ((float)(loc2.lng - lng)) * loc2.longitude_scale();
    return norm2(dlat, dlng) * LOCATION_SCALING_FACTOR;
}

/*
  return the distance in meters in North/East plane as a N/E vector
  from loc1 to loc2
 */
Vector2f Locations::get_distance_NE(const Locations &loc2) const
{
    return Vector2f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - lng) * LOCATION_SCALING_FACTOR * longitude_scale());
}

// return the distance in meters in North/East/Down  5plane as a N/E/D vector to loc2
Vector3f Locations::get_distance_NED(const Locations &loc2) const
{
    return Vector3f((loc2.lat - lat) * LOCATION_SCALING_FACTOR,
                    (loc2.lng - lng) * LOCATION_SCALING_FACTOR * longitude_scale(),
                    (alt - loc2.alt) * 0.01f);
}

// extrapolate latitude/longitude given distances (in meters) north and east
void Locations::offset(float ofs_north, float ofs_east)
{
    // use is_equal() because is_zero() is a local class conflict and is_zero() in AP_Math does not belong to a class
    if (!is_equal(ofs_north, 0.0f) || !is_equal(ofs_east, 0.0f))
    {
        int32_t dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / longitude_scale();
        lat += dlat;
        lng += dlng;
    }
}

/*
 *  extrapolate latitude/longitude given bearing and distance
 * Note that this function is accurate to about 1mm at a distance of
 * 100m. This function has the advantage that it works in relative
 * positions, so it keeps the accuracy even when dealing with small
 * distances and floating point numbers
 */
void Locations::offset_bearing(float bearing, float distance)
{
    const float ofs_north = cosf(radians(bearing)) * distance;
    const float ofs_east = sinf(radians(bearing)) * distance;
    offset(ofs_north, ofs_east);
}

float Locations::longitude_scale() const
{
    float scale = cosf(lat * (1.0e-7f * DEG_TO_RAD));
    return MAX(scale, 0.01f);
}

/*
 * convert invalid waypoint with useful data. return true if Locations changed
 */
bool Locations::sanitize(const Locations &defaultLoc)
{
    bool has_changed = false;
    // convert lat/lng=0 to mean current point
    if (lat == 0 && lng == 0)
    {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    // convert relative alt=0 to mean current alt
    if (alt == 0 && relative_alt)
    {
        relative_alt = false;
        alt = defaultLoc.alt;
        has_changed = true;
    }

    // limit lat/lng to appropriate ranges
    if (!check_latlng())
    {
        lat = defaultLoc.lat;
        lng = defaultLoc.lng;
        has_changed = true;
    }

    return has_changed;
}

// make sure we know what size the Locations object is:
assert_storage_size<Locations, 16> _assert_storage_size_Locations;

// return bearing in centi-degrees from Locations to loc2
int32_t Locations::get_bearing_to(const struct Locations &loc2) const 
{
    const int32_t off_x = loc2.lng - lng;
    const int32_t off_y = (loc2.lat - lat) / loc2.longitude_scale();
    int32_t bearing = 9000 + atan2f(-off_y, off_x) * DEGX100;
    if (bearing < 0) 
    {
        bearing += 36000;
    }
    return bearing; 
}

/*
  return true if lat and lng match. Ignores altitude and options
 */
bool Locations::same_latlon_as(const Locations &loc2) const
{
    return (lat == loc2.lat) && (lng == loc2.lng);
}

// return true when lat and lng are within range
bool Locations::check_latlng() const
{
    return check_lat(lat) && check_lng(lng);
}

// see if Locations is past a line perpendicular to
// the line between point1 and point2 and passing through point2.
// If point1 is our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool Locations::past_interval_finish_line(const Locations &point1, const Locations &point2) const
{
    return this->line_path_proportion(point1, point2) >= 1.0f;
}

/*
  return the proportion we are along the path from point1 to
  point2, along a line parallel to point1<->point2.

  This will be more than 1 if we have passed point2
 */
float Locations::line_path_proportion(const Locations &point1, const Locations &point2) const
{
    const Vector2f vec1 = point1.get_distance_NE(point2);
    const Vector2f vec2 = point1.get_distance_NE(*this);
    const float dsquared = sq(vec1.x) + sq(vec1.y);
    if (dsquared < 0.001f)
    {
        // the two points are very close together
        return 1.0f;
    }
    return (vec1 * vec2) / dsquared;
}

void Locations::point_use_bearing(Locations &to ,float bearing,float distance)
{
	const float ofs_north = cosf(DEG_TO_RAD*(bearing)) * distance;
    const float ofs_east = sinf(DEG_TO_RAD*(bearing)) * distance;
    if ((ofs_north!=0.0f) || (ofs_east!=0.0f))
    {
        int32_t dlat = ofs_north * this->LOCATION_SCALING_FACTOR_INV;
        int32_t dlng = (ofs_east * this->LOCATION_SCALING_FACTOR_INV) / this->longitude_scale();
        to.lat= lat + dlat;
        to.lng= lng + dlng;
	}


    // 	const float ofs_north = cosf(DEG_TO_RAD*(bearing)) * distance;
    // const float ofs_east = sinf(DEG_TO_RAD*(bearing)) * distance;
    // if ((ofs_north!=0.0f) || (ofs_east!=0.0f))
    // {
    //     int dlat = ofs_north * LOCATION_SCALING_FACTOR_INV;
    //     int dlng = (ofs_east * LOCATION_SCALING_FACTOR_INV) / thislongitude_scale();
    //     dlat = dlat + th
    //     dlng = dlng +
    //     to.lat= lat + dlat;
    //     to.lng= lng + dlng;
	// }
}