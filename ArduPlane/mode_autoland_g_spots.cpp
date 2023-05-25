#include "mode.h"
#include "Plane.h"
#include <cmath>


bool ModeAUTOLAND_G_SPOTS::_enter()
{
    gcs().send_text(MAV_SEVERITY_INFO,"GSPOT: ENTERed GSPOTs mode");
#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    if (plane.previous_mode == &plane.mode_guided && quadplane.guided_wait_takeoff_on_mode_enter) {
        if (!plane.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif

    /* 
    ________________ Locations in AP_AHRS.h and Location.h _______________________
    */
   // Get home point (location where the plane was ARMed! (not turned on or safety switch))
    Location gspot_loc_home{ AP::ahrs().get_home() };
    gcs().send_text(MAV_SEVERITY_INFO, "Home location at: %f Latitude, %f Longitude, %f Altitude", gspot_loc_home.lat* 1.0e-7, gspot_loc_home.lng* 1.0e-7, gspot_loc_home.alt* 1.0e-2);

    /* 
    ________________ Mission functions found in AP_Mission.h/.cpp _______________________
    */
    // Delete auto mission
    // plane.mission.clear();
    // plane.mission.start();
    // Create new autoland mission

    // Define approach WP2 location by distance and heading from touchdownpoint
    const int32_t   gspot_WP2_alt              = 3000;    // [cm] WP2 altidude
    const double    gspot_heading_runway_deg   = -193;     // [deg] Heading of the runway, direction as was armed
    const double    gspot_heading_runway_rad   = 2*M_PI*(gspot_heading_runway_deg/360); // [rad] above
    const int       gspot_dist_approach        = 300;      // [m] Horizontal distance of how far away the last waypoint in the air is (approach waypoint)
    int32_t gspot_latitude2_t, gspot_longitude2_t;         // [deg*1e7] Latitude and Longitude of WP2 in 1e7 degree, as used in Location type

    //Calculate approach waypoint (WP2) latitude and longitude
    ModeAUTOLAND_G_SPOTS::gspot_calc_lat_from_latlngdistheading(
                                gspot_loc_home.lat, gspot_loc_home.lng, 
                                gspot_dist_approach, gspot_heading_runway_rad,
                                &gspot_latitude2_t, &gspot_longitude2_t);
    gcs().send_text(MAV_SEVERITY_INFO, "WP at %i, %i", gspot_latitude2_t, gspot_longitude2_t);
    Location gspot_loc_WP2 {gspot_latitude2_t, gspot_longitude2_t, gspot_WP2_alt, Location::AltFrame::ABOVE_HOME};

    // Add loiter to alt WP3
    AP_Mission::Mission_Command gspot_cmd_WP3;
    gspot_cmd_WP3.id = MAV_CMD_NAV_LOITER_TO_ALT;
    gspot_cmd_WP3.content.location = gspot_loc_WP2;
    plane.mission.add_cmd(gspot_cmd_WP3);


    // Add final approach waypoint WP2
    AP_Mission::Mission_Command gspot_cmd_WP2;
    gspot_cmd_WP2.id = MAV_CMD_NAV_WAYPOINT;
    gspot_cmd_WP2.content.location = gspot_loc_WP2;
    plane.mission.add_cmd(gspot_cmd_WP2);

    // Add landing point WP1
    AP_Mission::Mission_Command gspot_cmd_WP1;
    gspot_cmd_WP1.id = MAV_CMD_NAV_LAND;
    gspot_cmd_WP1.content.location = gspot_loc_home;

    bool wait_for_mission_sent = plane.mission.add_cmd(gspot_cmd_WP1);
    if (wait_for_mission_sent){gcs().send_text(MAV_SEVERITY_INFO, "WP sent succesfully!");}
    // Felicidades, aquí está el rhinoceronte ;)
    else {gcs().send_text(MAV_SEVERITY_INFO, "WP sending ERROR");}    
    
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc;
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
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
    gcs().send_text(MAV_SEVERITY_INFO,"GSPOT: EXITing GSPOTs mode");
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeAUTOLAND_G_SPOTS::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto GSPOT without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }
#endif

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_Vehicle::FixedWing::FLIGHT_ABORT_LAND)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        gcs().send_text(MAV_SEVERITY_INFO,"GSPOT: Auto landing on the GSPOT");
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plane.calc_throttle();
        }

#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plane.nav_roll_cd = plane.ahrs.roll_sensor;
        plane.nav_pitch_cd = plane.ahrs.pitch_sensor;
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, plane.nav_scripting.throttle_pct);
#endif
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();
    }
}

void ModeAUTOLAND_G_SPOTS::navigate()
{
    if (AP::ahrs().home_is_set()) {
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
                                int32_t* gspot_latitude2_deg, int32_t* gspot_longitude2_deg)
{   /*
    ______ Calculates the latitude and longitude of a terminal point, _________________________
    ______ given the original point latitude, longitude, horizontal distance and heading (clockwise from north) ___________________________
    For reference of calculation see:
    http://www.movable-type.co.uk/scripts/latlong.html 
    Note that the input and output values for long and lat deg are in 1e7 deg!, all other vars require SI (m, rad)
    */
    const double gspot_delta = gspot_distance/(6.3781*1.0e+6);
    double gspot_latitude2_rad    = asin(sin((2*M_PI*(gspot_latitude1_deg*(1.0e-7))/360))*cos(gspot_delta) + 
                                cos((2*M_PI*(gspot_latitude1_deg*(1.0e-7))/360))*sin(gspot_delta)*cos(gspot_heading_rad));
    double gspot_longitude2_rad   = (2*M_PI*(gspot_longitude1_deg*(1.0e-7))/360) + 
                                atan2(sin(gspot_heading_rad)*sin(gspot_delta)*cos((2*M_PI*(gspot_latitude1_deg*(1.0e-7))/360)), 
                                cos(gspot_delta)-sin((2*M_PI*(gspot_latitude1_deg*(1.0e-7))/360))*sin(gspot_latitude2_rad));
    *gspot_latitude2_deg  = int((gspot_latitude2_rad*360/(2*M_PI))*1.0e7);
    *gspot_longitude2_deg = int(((fmodf((gspot_longitude2_rad*360/(2*M_PI))+ 540, 360))-180)*1e7);
    // *gspot_longitude2_deg = ((int((gspot_longitude2_rad*360/(2*M_PI))*1.0e7) + 540*int(1.0e7))%int(360*1.0e7)-180*1.0e7);
}