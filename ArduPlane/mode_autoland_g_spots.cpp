#include "mode.h"
#include "Plane.h"
#include <cmath>
#include <algorithm>

bool ModeAUTOLAND_G_SPOTS::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO, "GSPOT: ENTERed GSPOTs mode");
#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    if (plane.previous_mode == &plane.mode_guided && quadplane.guided_wait_takeoff_on_mode_enter)
    {
        if (!plane.mission.starts_with_takeoff_cmd())
        {
            gcs().send_text(MAV_SEVERITY_ERROR, "Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }

    if (plane.quadplane.available() && plane.quadplane.enable == 2)
    {
        plane.auto_state.vtol_mode = true;
    }
    else
    {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif

    /*
    ________________ Locations in AP_AHRS.h and Location.h _______________________
    */
    // Get home point (location where the plane was ARMed! (not turned on or safety switch))
    Location gspot_loc_home{AP::ahrs().get_home()};
    gcs().send_text(MAV_SEVERITY_INFO, "Home location at: %f Latitude, %f Longitude, %f Altitude", gspot_loc_home.lat * 1.0e-7, gspot_loc_home.lng * 1.0e-7, gspot_loc_home.alt * 1.0e-2);
    const int32_t gspot_WP5_alt = plane.get_RTL_altitude_cm() - gspot_loc_home.alt;                                // [cm] WP5 altidude is equal to rtl alt (ALT_HOLD_RTL parameter)
    Location gspot_loc_WP5{gspot_loc_home.lat, gspot_loc_home.lng, gspot_WP5_alt, Location::AltFrame::ABOVE_HOME}; // Location object WP5_alt above home, used for dummy WPinf, WP5 and WP4

    /*
    ________________ Mission functions found in AP_Mission.h/.cpp _______________________
    */
    // Delete auto mission
    plane.mission.clear();
    // plane.mission.start();

    // Add dummy waypoint WPinf because it somehow does not take the first WP into the mission
    AP_Mission::Mission_Command gspot_cmd_WPinf;
    gspot_cmd_WPinf.id = MAV_CMD_NAV_WAYPOINT;
    gspot_cmd_WPinf.content.location = gspot_loc_WP5;
    plane.mission.add_cmd(gspot_cmd_WPinf);

    // Add loiter turns WP6
    AP_Mission::Mission_Command gspot_cmd_WP6;
    gspot_cmd_WP6.id = MAV_CMD_NAV_LOITER_TURNS;
    // gspot_cmd_WP6.p1 = plane.g.landa_ltr_turns; // Loiter turns in [int]
    gspot_cmd_WP6.p1 = std::min(int(255), int(plane.g.landa_ltr_turns)); // number of loiter turns
    gspot_cmd_WP6.p1 |= (abs(plane.aparm.loiter_radius)<<8);                  // loiter radius
    gspot_cmd_WP6.content.location = gspot_loc_WP5;
    gspot_cmd_WP6.content.location.loiter_ccw = (plane.aparm.loiter_radius < 0); // set cw or ccw by neg/pos
    gspot_cmd_WP6.content.location.loiter_xtrack = 0;               // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
    plane.mission.add_cmd(gspot_cmd_WP6);

    // Add loiter time WP5
    AP_Mission::Mission_Command gspot_cmd_WP5;
    gspot_cmd_WP5.id = MAV_CMD_NAV_LOITER_TIME;
    gspot_cmd_WP5.p1 = plane.g.landa_ltr_time; // Loiter time in [s] // loiter time in seconds uses all 16 bits, 8bit seconds is too small. No room for radius.
    gspot_cmd_WP5.content.location = gspot_loc_WP5;
    gspot_cmd_WP5.content.location.loiter_ccw = (plane.aparm.loiter_radius < 0); // set cw or ccw by neg/pos
    gspot_cmd_WP5.content.location.loiter_xtrack = 0;               // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
    plane.mission.add_cmd(gspot_cmd_WP5);

    // // Add RTL WP4
    // AP_Mission::Mission_Command gspot_cmd_WP4;
    // gspot_cmd_WP4.id = MAV_CMD_NAV_WAYPOINT;
    // gspot_cmd_WP4.content.location = gspot_loc_WP5;
    // plane.mission.add_cmd(gspot_cmd_WP4);

    // Set bool for next waypoints (see update)
    gspot_land_mission_written = false;

    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed())
    {
        if (hal.util->persistent_data.waypoint_num != 0)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeAUTOLAND_G_SPOTS::_exit()
{
    gcs().send_text(MAV_SEVERITY_INFO, "GSPOT: EXITing GSPOTs mode");
    gspot_land_mission_written = false;
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING)
    {
        plane.mission.stop();

        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id))
        {
            restart = false;
        }
#endif
        if (restart)
        {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAUTOLAND_G_SPOTS::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING)
    {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto GSPOT without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto())
    {
        plane.quadplane.control_auto();
        return;
    }
#endif

    /*
    ____________ START: Create land mission waypoints based on windspeed ______________
    */

    if (nav_cmd_id == MAV_CMD_NAV_LOITER_TIME && !gspot_land_mission_written)
    {
        // Get wind info
        Vector3f gspot_wind = AP::ahrs().wind_estimate();
        gspot_wind_heading_deg_cw_from_north = degrees(atan2f(-gspot_wind.y, -gspot_wind.x));
        gspot_wind_vel_total_mps = sqrt(pow(gspot_wind.x, 2) + pow(gspot_wind.y, 2));
        if (gspot_wind_heading_deg_cw_from_north < 0.)
        {
            gspot_wind_heading_deg_cw_from_north = 360 + gspot_wind_heading_deg_cw_from_north;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "Wind speed: %f [m/s]; Wind direction: %f [deg] from north cw", gspot_wind_vel_total_mps, gspot_wind_heading_deg_cw_from_north);

        // Set land flap percentage
        float gspot_wind_head_total = fabs(cos(radians(fabs(gspot_wind_heading_deg_cw_from_north - degrees(plane.initial_armed_bearing))))) * gspot_wind_vel_total_mps;
        float gspot_land_wind_max = plane.g.landa_flapmaxwnd;
        if (gspot_wind_head_total < gspot_land_wind_max)
        {
            gspot_land_flap_percent = fabs(((std::min(gspot_wind_head_total, std::max(float(0), (gspot_wind_head_total - plane.g.landa_flapminwnd))) / (gspot_land_wind_max - plane.g.landa_flapminwnd)) * 100) - 100);
        } // approach direction does not matter, using abs(cos(...)) for that
        else
        {
            gspot_land_flap_percent = 0;
        }
        gcs().send_text(MAV_SEVERITY_INFO, "GSPOT: Landing headwind vel: %f [m/s], Flap will be set at: %i [perc]", gspot_wind_head_total, gspot_land_flap_percent);

        // Define approach waypoint location by distance and heading from touchdownpoint
        const int16_t gspot_appr_alt = abs(plane.g.landa_appr_alt) * 100;                                                                                                     // [cm] WP2/approach point altidude
        int16_t gspot_appr_dist = std::max((abs(plane.g.landa_appr_dist) / 100), abs(plane.g.landa_appr_dist) - abs(int(gspot_wind_head_total * plane.g.landa_appr_scale)));  // [m] Horizontal distance of how far away the last waypoint in the air is (approach waypoint), scaled by headwind and minimum of LANDA_APPR_DIST/100
        int32_t gspot_latitude2_t, gspot_longitude2_t;                                                                                                                        // [deg*1e7] Latitude and Longitude of WP2/approach point in 1e7 degree, as used in Location type
        float gspot_heading_runway_rad = (plane.g.landa_clout_enbl == 1 && !(fabs(plane.climb_out_bearing) <= 1e-9)) ? plane.climb_out_bearing : plane.initial_armed_bearing; // [rad] Heading of the runway cw from true north, direction as was armed or if LANDA_CLOUT_ENBL is set to 1 it will use the auto takeoff heading
        Location gspot_loc_home{AP::ahrs().get_home()};                                                                                                                       // Get home location
        gcs().send_text(MAV_SEVERITY_INFO, "GSPOT: Approach point to land point hor. distance set at %i [m]", gspot_appr_dist);

        // if the wind is coming from the back turn into the wind for landing
        if (fabs(gspot_wind_heading_deg_cw_from_north - degrees(plane.initial_armed_bearing)) > 90 + plane.g.landa_wnd_margin)
        {
            gspot_appr_dist = -gspot_appr_dist;
        }

        // Calculate approach waypoint latitude and longitude
        ModeAUTOLAND_G_SPOTS::gspot_calc_lat_from_latlngdistheading(
            gspot_loc_home.lat, gspot_loc_home.lng,
            -gspot_appr_dist, gspot_heading_runway_rad,
            &gspot_latitude2_t, &gspot_longitude2_t);
        Location gspot_loc_WP4{gspot_latitude2_t, gspot_longitude2_t, (plane.get_RTL_altitude_cm() - gspot_loc_home.alt), Location::AltFrame::ABOVE_HOME};
        Location gspot_loc_WP2{gspot_latitude2_t, gspot_longitude2_t, gspot_appr_alt, Location::AltFrame::ABOVE_HOME};
        Location gspot_loc_WP1{gspot_loc_home.lat, gspot_loc_home.lng, 0, Location::AltFrame::ABOVE_HOME};

        // Add RTL WP4
        AP_Mission::Mission_Command gspot_cmd_WP4;
        gspot_cmd_WP4.id = MAV_CMD_NAV_WAYPOINT;
        gspot_cmd_WP4.p1 = (0 << 8) | (plane.g.waypoint_radius & 0x00FF);             // param 3 pass by distance in meters is held in high p1 AND param 2 is acceptance radius in meters is held in low p1
        gspot_cmd_WP4.content.location = gspot_loc_WP4;
        plane.mission.add_cmd(gspot_cmd_WP4);

        // Add loiter to alt WP3
        AP_Mission::Mission_Command gspot_cmd_WP3;
        gspot_cmd_WP3.id = MAV_CMD_NAV_LOITER_TO_ALT;
        gspot_cmd_WP3.p1 = abs(plane.aparm.loiter_radius);
        gspot_cmd_WP3.content.location = gspot_loc_WP2;
        gspot_cmd_WP3.content.location.loiter_ccw = (plane.aparm.loiter_radius < 0); // set cw or ccw by neg/pos
        gspot_cmd_WP3.content.location.loiter_xtrack = 0;               // 0 to xtrack from center of waypoint, 1 to xtrack from tangent exit location
        plane.mission.add_cmd(gspot_cmd_WP3);

        // Add final approach waypoint WP2
        AP_Mission::Mission_Command gspot_cmd_WP2;
        gspot_cmd_WP2.id = MAV_CMD_NAV_WAYPOINT;
        gspot_cmd_WP2.p1 = (0 << 8) | (plane.g.waypoint_radius & 0x00FF);             // param 3 pass by distance in meters is held in high p1 AND param 2 is acceptance radius in meters is held in low p1
        gspot_cmd_WP2.content.location = gspot_loc_WP2;
        plane.mission.add_cmd(gspot_cmd_WP2);

        // Add landing point WP1
        AP_Mission::Mission_Command gspot_cmd_WP1;
        gspot_cmd_WP1.id = MAV_CMD_NAV_LAND;
        gspot_cmd_WP1.p1 = (plane.get_RTL_altitude_cm() - gspot_loc_home.alt) / 100; // set abort altitude to rtl altitude
        gspot_cmd_WP1.content.location = gspot_loc_WP1;                              // made new waypoint with relative altitude (instead of gspot_loc_home with total altitude) to ensure that the abort altitude is above home)
        plane.mission.add_cmd(gspot_cmd_WP1);
        // bool WP1_sent = plane.mission.add_cmd(gspot_cmd_WP1);
        // if (WP1_sent){gcs().send_text(MAV_SEVERITY_INFO, "WP1 sent succesfully!");}
        // // Felicidades, aquí está el rhinoceronte ;)
        // else {gcs().send_text(MAV_SEVERITY_INFO, "WP1 sending ERROR");}

        gspot_land_mission_written = true;
    }

    /*
    ____________ END: Create land mission waypoints based on windspeed ______________
    */

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND))
    {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    }
    else if (nav_cmd_id == MAV_CMD_NAV_LAND)
    {
        static bool gspot_runonce_lnd_msg = true;
        if (gspot_runonce_lnd_msg)
        {
            gcs().send_text(MAV_SEVERITY_INFO, "GSPOT: Auto landing on the GSPOT");
            gspot_runonce_lnd_msg = false;
        }

        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit * 100UL);

        if (plane.landing.is_throttle_suppressed())
        {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        }
        else
        {
            plane.calc_throttle();
        }

#if AP_SCRIPTING_ENABLED
    }
    else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME)
    {
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plane.nav_roll_cd = plane.ahrs.roll_sensor;
        plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    }
    else
    {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT)
        {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeAUTOLAND_G_SPOTS::navigate()
{
    if (AP::ahrs().home_is_set())
    {
        plane.mission.update();
    }
}

bool ModeAUTOLAND_G_SPOTS::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
    return (!plane.nav_scripting_active());
#endif
    return true;
}

bool ModeAUTOLAND_G_SPOTS::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
    return (!plane.nav_scripting_active());
#endif
    return true;
}

void ModeAUTOLAND_G_SPOTS::gspot_calc_lat_from_latlngdistheading(
    int32_t gspot_latitude1_deg, int32_t gspot_longitude1_deg, int gspot_distance, double gspot_heading_rad,
    int32_t *gspot_latitude2_deg, int32_t *gspot_longitude2_deg)
{ /*
  ______ Calculates the latitude and longitude of a terminal point, _________________________
  ______ given the original point latitude, longitude, horizontal distance and heading (clockwise from north) ___________________________
  For reference of calculation see:
  http://www.movable-type.co.uk/scripts/latlong.html
  Note that the input and output values for long and lat deg are in 1e7 deg!, all other vars require SI (m, rad)
  */

    const double gspot_delta = gspot_distance / (6.3781 * 1.0e+6);
    double gspot_latitude2_rad = asin(sin(radians(gspot_latitude1_deg) * (1.0e-7)) * cos(gspot_delta) +
                                      cos(radians(gspot_latitude1_deg) * (1.0e-7)) * sin(gspot_delta) * cos(gspot_heading_rad));
    double gspot_longitude2_rad = (radians(gspot_longitude1_deg) * (1.0e-7)) +
                                  atan2(sin(gspot_heading_rad) * sin(gspot_delta) * cos(radians(gspot_latitude1_deg) * (1.0e-7)),
                                        cos(gspot_delta) - sin(radians(gspot_latitude1_deg) * (1.0e-7)) * sin(gspot_latitude2_rad));
    *gspot_latitude2_deg = int(degrees(gspot_latitude2_rad) * 1.0e7);
    *gspot_longitude2_deg = int(((fmodf(degrees(gspot_longitude2_rad) + 540, 360)) - 180) * 1e7);
}
