///*******************************************************************************
// *  Copyright: Balint Aron Uveges, 2022                                        *
// *  Developed at Pazmany Peter Catholic University,                            *
// *               Faculty of Information Technology and Bionics                 *
// *  Author(s): Balint Aron Uveges                                              *
// *  This file is distributed under the terms in the attached LICENSE file.     *
// *                                                                             *
// *******************************************************************************/

package physicalProcess.WildFirePhysicalProcess;

// The physical process module simulates a physical process that could be measured/sampled
// by a sensing device on the nodes. Different sensing devices (e.g temperature, pressure,
// light, acceleration) would be represented by distinct PhysicalProcess modules. A node
// simply ask the process for a sample which is returned as soon as it is calculated.

simple WildFirePhysicalProcess like physicalProcess.iPhysicalProcess {
parameters:
	bool    collectTraceInfo = default (false);
    bool    enabled          = default (true);
    int     wf_start_x_coord = default (0);
    int     wf_start_y_coord = default (0);
    int     wf_start_x_offset= default (1);
    int     wf_start_y_offset= default (1);
    bool    no_map_file      = default (false);
    string  map_file;
    int     map_scale        = default (90);
    int     ca_step_period;
    int     ca_start_timer;
    int     step_limit       = default(0);       // - how many steps are performed
    double  p_h              = default(0.58);    // - p_h    - corrective probability coefficent
    double  c_1              = default(0.045);   // - c_1    - wind model's first coefficient - affects speed
    double  c_2              = default(0.131);   // - c_2    - wind model's second coefficient - affects angle
    double  a                = default(0.078);   // - a      - slope model's coefficient
    double  w_a              = default(0);       // - w_a    - wind angle (North: 0, East: Pi/2, South: Pi, West: 3PI/2), rad
    double  w_s              = default(8.1);     // - w_s    - wind speed, m/s
    double  p_mois           = default(0.0);     // - p_mois - fuel moisture
    double  l                = default(90);      // - l      - cell's side length, m
    bool    sp               = default(false);   // - sp     - fire spotting enabled/disabled
    int     seed             = default(0);       // - seed   - random seed
    bool    spatial_sense    = default(false);   // - sense neighbouring cells
    int     sense_distance   = default(5);       // - how far the sensor reaches
    double  sense_attn       = default(1);       // - how fast sense attenuated
    bool    plane_to_yaml    = default(false);   // - serialize WF plane to yaml after every iteration
    string  yp_coding        = default("digit"); // - how to code yaml plane, digit or enum
	string  description      = default ("CA based wildfire");
    double  rad_res          = default(0.01);    // - radius resolution
    double  look_rad         = default(8);       // - radius looking 
    bool    sel_all_cell     = default(false);   // - how to select cells while looking around
    string  sensing_model    = default("spatial_sense"); // spatial_sense, disk_model, prob_model,
                                                         // prob_spatial_sense, distance_disk_model
    double  r_u              = default(0);       // - R_u, uncertaininty radius
    double  lambda           = default(1);       // - lambda, for exp. probability
    double  gamma            = default(1);       // - gamma, for exp. probability
    bool    plane_at_finish  = default(false);
    double  c_w_a            = default(0);       // - Influence of wind on sensing, wind angle wise
    double  c_w_s            = default(0);       // - influence of wind on sensing, wind speed wise
    double  c_w_l            = default(0);       // - Wind speed in m/s, that must be exceeded to have effect
    double  c_w_m            = default(0);       // - Maximum wind speed, that if exceeded, no additional change happen
    bool    sv_to_yaml       = default(false);   // - Sensed value to yaml

 gates:
	output toNode[];
	input fromNode[];
}
