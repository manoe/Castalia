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
    int     wf_start_x_coord = default (0);
    int     wf_start_y_coord = default (0);
    bool    no_map_file      = default (false);
    string  map_file;
    int     map_scale        = default (90);
    int     ca_step_period;
    int     ca_start_timer;
    double  p_h              = default(0.58);    // - p_h  - corrective probability coefficent
    double  c_1              = default(0.045);   // - c_1  - wind model's first coefficient - affects speed
    double  c_2              = default(0.131);   // - c_2  - wind model's second coefficient - affects angle
    double  a                = default(0.078);   // - a    - slope model's coefficient
    double  w_a              = default(0);       // - w_a  - wind angle (North: 0, East: Pi/2, South: Pi, West: 3PI/2), rad
    double  w_s              = default(8.1);     // - w_s  - wind speed, m/s
    double  l                = default(90);      // - l    - cell's side length, m
    bool    sp               = default(false);   // - sp   - fire spotting enabled/disabled
    int     seed             = default(0);       // - seed - random seed
    bool    spatial_sense    = default(false);   // - sense neighbouring cells
    int     sense_distance   = default(5);       // - how far the sensor reaches
    double  sense_attn       = default(1);       // - how fast sense attenuated
    bool    plane_to_yaml    = default(false);   // - serialize WF plane to yaml after every iteration
    string  yp_coding        = default("digit"); // - how to code yaml plane, digit or enum
	string  description      = default ("CA based wildfire");

 gates:
	output toNode[];
	input fromNode[];
}
