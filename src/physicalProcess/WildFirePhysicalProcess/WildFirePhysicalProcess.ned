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
    string  map_file;
    int     map_scale = default (90);
    int     ca_step_period;
    int     ca_start_timer;

	string  description = default ("CA based wildfire");

 gates:
	output toNode[];
	input fromNode[];
}
