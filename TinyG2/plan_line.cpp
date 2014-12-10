/*
 * plan_line.c - acceleration managed line planning and motion execution
 * This file is part of the TinyG project
 *
 * Copyright (c) 2010 - 2014 Alden S. Hart, Jr.
 * Copyright (c) 2012 - 2014 Rob Giseburt
 *
 * This file ("the software") is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License, version 2 as published by the
 * Free Software Foundation. You should have received a copy of the GNU General Public
 * License, version 2 along with the software.  If not, see <http://www.gnu.org/licenses/>.
 *
 * As a special exception, you may use this file as part of a software library without
 * restriction. Specifically, if other files instantiate templates or use macros or
 * inline functions from this file, or you compile this file and link it with  other
 * files to produce an executable, this file does not by itself cause the resulting
 * executable to be covered by the GNU General Public License. This exception does not
 * however invalidate any other reasons why the executable file might be covered by the
 * GNU General Public License.
 *
 * THE SOFTWARE IS DISTRIBUTED IN THE HOPE THAT IT WILL BE USEFUL, BUT WITHOUT ANY
 * WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT
 * SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
 * OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "tinyg2.h"
#include "config.h"
#include "canonical_machine.h"
#include "planner.h"
#include "stepper.h"
#include "report.h"
#include "util.h"
#include "spindle.h"

// aline planner routines / feedhold planning
static void _calc_move_times(GCodeState_t *gms, const float axis_length[], const float axis_square[]);
static void _plan_block_list(mpBuf_t *bf);
static float _get_junction_vmax(const float a_unit[], const float b_unit[]);
static float _compute_next_segment_velocity();

/* Runtime-specific setters and getters
 *
 * mp_zero_segment_velocity() 		- correct velocity in last segment for reporting purposes
 * mp_get_runtime_velocity() 		- returns current velocity (aggregate)
 * mp_get_runtime_machine_position()- returns current axis position in machine coordinates
 * mp_set_runtime_work_offset()		- set offsets in the MR struct
 * mp_get_runtime_work_position() 	- returns current axis position in work coordinates
 *									  that were in effect at move planning time
 */

void mp_zero_segment_velocity() { mr.segment_velocity = 0;}
float mp_get_runtime_velocity(void) { return (mr.segment_velocity);}
float mp_get_runtime_absolute_position(uint8_t axis) { return (mr.position[axis]);}
void mp_set_runtime_work_offset(float offset[]) { copy_vector(mr.gm.work_offset, offset);}
float mp_get_runtime_work_position(uint8_t axis) { return (mr.position[axis] - mr.gm.work_offset[axis]);}

/*
 * mp_get_runtime_busy() - return TRUE if motion control busy (i.e. robot is moving)
 *
 *	Use this function to sync to the queue. If you wait until it returns
 *	FALSE you know the queue is empty and the motors have stopped.
 */

uint8_t mp_get_runtime_busy()
{
	if ((st_runtime_isbusy() == true) || (mr.move_state == MOVE_RUN)) return (true);
	return (false);
}

static void _swap_replanned_trapezoid(mpBuf_t *bf) {
    bf->entry_velocity = bf->replanned_entry_velocity;
    bf->cruise_velocity = bf->replanned_cruise_velocity;
    bf->exit_velocity = bf->replanned_exit_velocity;
    bf->head_length = bf->replanned_head_length;
    bf->body_length = bf->replanned_body_length;
    bf->tail_length = bf->replanned_tail_length;
}

static void _mr_swap_replanned_trapezoid() {
    mr.entry_velocity = mr.replanned_entry_velocity;
    mr.cruise_velocity = mr.replanned_cruise_velocity;
    mr.exit_velocity = mr.replanned_exit_velocity;
    mr.head_length = mr.replanned_head_length;
    mr.body_length = mr.replanned_body_length;
    mr.tail_length = mr.replanned_tail_length;
}

static void _request_replan() { mr.replan_state = REPLAN_REQUESTED; }
static void _request_replan_swap(mpBuf_t *replan_bp0, mpBuf_t *replan_hold_buffer) {
    mr.replan_bp0 = replan_bp0;
    mr.replan_hold_buffer = replan_hold_buffer;
    mr.replan_state = REPLAN_SWAP_REQUESTED;
}
static void _finish_replan() { mr.replan_state = REPLAN_OFF; }

static mpBuf_t scratchBuf;

void mp_check_for_replan()
{
    if(mr.replan_state != REPLAN_SWAP_REQUESTED) {
        mr.replan_interrupted = 1;
        return;
    }
    
    /* If the replan was interrupted by exec, and involves a change in mr, or if bp0 changed, reject it... */
    bool mrChanged = fp_NE(mr.entry_velocity, mr.replanned_entry_velocity) ||
                    fp_NE(mr.cruise_velocity, mr.replanned_cruise_velocity) ||
                    fp_NE(mr.exit_velocity, mr.replanned_exit_velocity) ||
                    fp_NE(mr.head_length + mr.body_length + mr.tail_length, mr.replanned_head_length + mr.replanned_body_length + mr.replanned_tail_length);
    if(mr.replan_interrupted && (mrChanged || mr.replan_bp0 != mb.r)) {
        mr.replan_state = REPLAN_REQUESTED;
        return;
    }
    
    /* If we need to swap bp0 in somewhere weird, or adjust lengths, do it now...
     * This is as a result of mp_plan_hold_callback */
    if(mr.replan_hold_buffer != NULL) {
        mr.replan_bp0->length = mr.replan_bp0_length;
        mr.replan_bp0->delta_vmax = mp_get_target_velocity(0, mr.replan_bp0->length, mr.replan_bp0);
        mr.replan_bp0->move_state = MOVE_NEW;
        if(mr.replan_hold_buffer != mr.replan_bp0) {
            //If the hold buffer exists and is not bp0, then the hold buffer has been split into
            //"hold buffer" (decel to new hold point) and "bp0" (accel back to original target)
            copy_vector(mr.replan_bp0->gm.target, mr.replan_hold_buffer->gm.target);
            for(uint8_t axis = 0; axis < AXES; axis++) {
                mr.replan_hold_buffer->gm.target[axis] -= mr.replan_hold_buffer->unit[axis] * (mr.replan_hold_buffer->head_length + mr.replan_hold_buffer->body_length + mr.replan_hold_buffer->tail_length);
            }
            copy_vector(mr.replan_bp0->unit, mr.replan_hold_buffer->unit);
            mr.replan_hold_buffer->length = mr.replan_hold_buffer_length;
            mr.replan_hold_buffer->delta_vmax = mp_get_target_velocity(0, mr.replan_hold_buffer->length, mr.replan_hold_buffer);
            /* shift stuff to make room for bp0... */
            mp_copy_buffer(&scratchBuf, mr.replan_bp0);
            mpBuf_t *bp = mr.replan_bp0;
            do {
                mp_copy_buffer(bp, bp->nx);
                bp = bp->nx;
            } while(bp != mr.replan_hold_buffer);
            mp_copy_buffer(mr.replan_hold_buffer, &scratchBuf);
        }
    }
    
    /* Get exec_aline to reset the segment processing, if needed */
    if(mr.move_state == MOVE_RUN && (
         fp_NE(mr.replanned_head_length, mr.head_length) ||
         fp_NE(mr.replanned_body_length, mr.body_length) ||
         fp_NE(mr.replanned_tail_length, mr.tail_length))) {
        _mr_swap_replanned_trapezoid();
        
        mr.section = SECTION_HEAD;
        mr.section_state = SECTION_NEW;
        for (uint8_t axis=0; axis<AXES; axis++) {
			mr.waypoint[SECTION_HEAD][axis] = mr.position[axis] + mr.unit[axis] * mr.head_length;
			mr.waypoint[SECTION_BODY][axis] = mr.position[axis] + mr.unit[axis] * (mr.head_length + mr.body_length);
			mr.waypoint[SECTION_TAIL][axis] = mr.position[axis] + mr.unit[axis] * (mr.head_length + mr.body_length + mr.tail_length);
		}
        copy_vector(mr.target, mr.waypoint[SECTION_TAIL]);
    }

    mpBuf_t *bp = mr.replan_bp0;
    do {
        _swap_replanned_trapezoid(bp);
        bp = mp_get_next_buffer(bp);
    } while((bp->move_state != MOVE_OFF) && (bp != mr.replan_bp0));
    
    if(mr.replan_hold_buffer != NULL && cm.hold_state == FEEDHOLD_PLAN) {
        cm.hold_state = FEEDHOLD_DECEL;
        _request_replan(); //to smooth out stuff on the other side of the hold...
    } else
        _finish_replan();
}

/****************************************************************************************
 * mp_aline() - plan a line with acceleration / deceleration
 *
 *	This function uses constant jerk motion equations to plan acceleration and deceleration
 *	The jerk is the rate of change of acceleration; it's the 1st derivative of acceleration,
 *	and the 3rd derivative of position. Jerk is a measure of impact to the machine.
 *	Controlling jerk smooths transitions between moves and allows for faster feeds while
 *	controlling machine oscillations and other undesirable side-effects.
 *
 * 	Note All math is done in absolute coordinates using single precision floating point (float).
 *
 *	Note: Returning a status that is not STAT_OK means the endpoint is NOT advanced. So lines
 *	that are too short to move will accumulate and get executed once the accumulated error
 *	exceeds the minimums.
 */

stat_t mp_aline(GCodeState_t *gm_in)
{
	mpBuf_t *bf; 						// current move pointer
	float exact_stop = 0;				// preset this value OFF
	float junction_velocity;

	// compute some reused terms
	float axis_length[AXES];
	float axis_square[AXES];
	float length_square = 0;

	for (uint8_t axis=0; axis<AXES; axis++) {
		axis_length[axis] = gm_in->target[axis] - mm.position[axis];
		axis_square[axis] = square(axis_length[axis]);
		length_square += axis_square[axis];
	}
	float length = sqrt(length_square);

	// exit if the move has zero movement. At all.
	if (fp_ZERO(length)) {
//	if (fp_ZERO(axis_length[AXIS_X]) && fp_ZERO(axis_length[AXIS_Y]) && fp_ZERO(axis_length[AXIS_Z]) &&
//		fp_ZERO(axis_length[AXIS_A]) && fp_ZERO(axis_length[AXIS_B]) && fp_ZERO(axis_length[AXIS_C])) {
		sr_request_status_report(SR_REQUEST_IMMEDIATE_FULL);
		return (STAT_MINIMUM_LENGTH_MOVE);
	}

	// If _calc_move_times() says the move will take less than the minimum move time get a more
	// accurate time estimate based on starting velocity and acceleration. The time of the move
	// is determined by its initial velocity (Vi) and how much acceleration will be occur. For
	// this we need to look at the exit velocity of the previous block. There are 3 possible cases:
	//	(1) There is no previous block. Vi = 0
	//	(2) Previous block is optimally planned. Vi = previous block's exit_velocity
	//	(3) Previous block is not optimally planned. Vi <= previous block's entry_velocity + delta_velocity

	_calc_move_times(gm_in, axis_length, axis_square);						// set move time and minimum time in the state

	if (gm_in->move_time < MIN_BLOCK_TIME) {
/*		float delta_velocity = pow(length, 0.66666666) * mm.cbrt_jerk;		// max velocity change for this move - uses jerk from previous move
		float entry_velocity = 0;											// pre-set as if no previous block
		if ((bf = mp_get_run_buffer()) != NULL) {
			if (bf->replannable == true) {									// not optimally planned
				entry_velocity = bf->entry_velocity + bf->delta_vmax;
			} else {														// optimally planned
				entry_velocity = bf->exit_velocity;
			}
		}
		float move_time = (2 * length) / (2*entry_velocity + delta_velocity);// compute execution time for this move
		if (move_time < MIN_BLOCK_TIME) {
			sr_request_status_report(SR_REQUEST_IMMEDIATE_FULL);
*/
			return (STAT_MINIMUM_TIME_MOVE);
//		}
	}

	// get a cleared buffer and setup move variables
	if ((bf = mp_get_write_buffer()) == NULL) {							// never supposed to fail
		return(cm_hard_alarm(STAT_BUFFER_FULL_FATAL));
	}
	bf->bf_func = mp_exec_aline;										// register the callback to the exec function
	bf->length = length;
	memcpy(&bf->gm, gm_in, sizeof(GCodeState_t));						// copy model state into planner buffer

	// Compute the unit vector and find the right jerk to use (combined operations)
	// To determine the jerk value to use for the block we want to find the axis for which
	// the jerk cannot be exceeded - the 'jerk-limit' axis. This is the axis for which
	// the time to decelerate from the target velocity to zero would be the longest.
	//
	//	We can determine the "longest" deceleration in terms of time or distance.
	//
	//  The math for time-to-decelerate T from speed S to speed E with constant
	//  jerk J is:
	//
	//		T = 2*sqrt((S-E)/J[n])
	//
	//	Since E is always zero in this calculation, we can simplify:
	//		T = 2*sqrt(S/J[n])
	//
	//	The math for distance-to-decelerate l from speed S to speed E with constant
	//  jerk J is:
	//
	//		l = (S+E)*sqrt((S-E)/J)
	//
	//	Since E is always zero in this calculation, we can simplify:
	//		l = S*sqrt(S/J)
	//
	//  The final value we want is to know which one is *longest*, compared to the others,
	//	so we don't need the actual value. This means that the scale of the value doesn't
	//	matter, so for T we can remove the "2 *" and For L we can remove the "S*".
	//	This leaves both to be simply "sqrt(S/J)". Since we don't need the scale,
	//	it doesn't matter what speed we're coming from, so S can be replaced with 1.
	//
	//  However, we *do* need to compensate for the fact that each axis contributes
	//	differently to the move, so we will scale each contribution C[n] by the
	//	proportion of the axis movement length D[n] to the total length of the move L.
	//	Using that, we construct the following, with these definitions:
	//
	//		J[n] = max_jerk for axis n
	//		D[n] = distance traveled for this move for axis n
	//		L = total length of the move in all axes
	//		C[n] = "axis contribution" of axis n
	//
	// For each axis n: C[n] = sqrt(1/J[n]) * (D[n]/L)
	//
	//	Keeping in mind that we only need a rank compared to the other axes, we can further
	//	optimize the calculations::
	//
	//	Square the expression to remove the square root:
	//		C[n]^2 = (1/J[n]) * (D[n]/L)^2	(We don't care the C is squared, we'll use it that way.)
	//
	//	Re-arranged to optimize divides for pre-calculated values, we create a value
	//  M that we compute once:
	//		M = (1/(L^2))
	//  Then we use it in the calculations for every axis:
	//		C[n] = (1/J[n]) * D[n]^2 * M
	//
	//  Also note that we already have (1/J[n]) calculated for each axis, which simplifies it further.
	//
	// Finally, the selected jerk term needs to be scaled by the reciprocal of the absolute value
	// of the jerk-limit axis's unit vector term. This way when the move is finally decomposed into
	// its constituent axes for execution the jerk for that axis will be at it's maximum value.

	float C;					// contribution term. C = T * a
	float maxC = 0;
	float recip_L2 = 1/length_square;

	for (uint8_t axis=0; axis<AXES; axis++) {
		if (fp_NOT_ZERO(axis_length[axis])) {
			bf->unit[axis] = axis_length[axis] / bf->length;			// compute unit vector term (zeros are already zero)
			C = axis_square[axis] * recip_L2 * cm.a[axis].recip_jerk;	// squaring axis_length ensures it's positive
			if (C > maxC) {
				maxC = C;
				bf->jerk_axis = axis;									// also needed for junction vmax calculation
			}
		}
	}
	// set up and pre-compute the jerk terms needed for this round of planning
//	bf->jerk = cm.a[bf->jerk_axis].recip_jerk * fabs(bf->unit[bf->jerk_axis]);// scale the jerk
	bf->jerk = cm.a[bf->jerk_axis].jerk_max * JERK_MULTIPLIER / fabs(bf->unit[bf->jerk_axis]);	// scale the jerk

	if (fabs(bf->jerk - mm.jerk) > JERK_MATCH_TOLERANCE) {	// specialized comparison for tolerance of delta
		mm.jerk = bf->jerk;									// used before this point next time around
		mm.recip_jerk = 1/bf->jerk;							// compute cached jerk terms used by planning
		mm.cbrt_jerk = cbrt(bf->jerk);
	}
	bf->recip_jerk = mm.recip_jerk;
	bf->cbrt_jerk = mm.cbrt_jerk;

	// finish up the current block variables
	if (cm_get_path_control(MODEL) != PATH_EXACT_STOP) { 	// exact stop cases already zeroed
		exact_stop = 8675309;								// an arbitrarily large floating point number
	}
	bf->cruise_vmax = bf->length / bf->gm.move_time;		// target velocity requested
	junction_velocity = _get_junction_vmax(bf->pv->unit, bf->unit);
	bf->entry_vmax = min3(bf->cruise_vmax, junction_velocity, exact_stop);
	bf->delta_vmax = mp_get_target_velocity(0, bf->length, bf);
	bf->exit_vmax = min3(bf->cruise_vmax, (bf->entry_vmax + bf->delta_vmax), exact_stop);
	bf->braking_velocity = bf->delta_vmax;
    
    /* Initialize the buffer to plan to/from zero, and then ask for a more optimal plan */
    bf->replanned_entry_velocity = 0;
    bf->replanned_exit_velocity = 0;
    bf->replanned_cruise_velocity = bf->cruise_vmax;
    mp_calculate_trapezoid(bf);
    _swap_replanned_trapezoid(bf);

	// Note: these next lines must remain in exact order. Position must update before committing the buffer.
	copy_vector(mm.position, bf->gm.target);	// set the planner position
	mp_commit_write_buffer(MOVE_TYPE_ALINE); 	// commit current block (must follow the position update)
    
    _request_replan();
	return (STAT_OK);
}

/***** ALINE HELPERS *****
 * _calc_move_times()
 * _plan_block_list()
 * _get_junction_vmax()
 */

/*
 * _calc_move_times() - compute optimal and minimum move times into the gcode_state
 *
 *	"Minimum time" is the fastest the move can be performed given the velocity constraints on each
 *	participating axis - regardless of the feed rate requested. The minimum time is the time limited
 *	by the rate-limiting axis. The minimum time is needed to compute the optimal time and is
 *	recorded for possible feed override computation..
 *
 *	"Optimal time" is either the time resulting from the requested feed rate or the minimum time if
 *	the requested feed rate is not achievable. Optimal times for traverses are always the minimum time.
 *
 *	The gcode state must have targets set prior by having cm_set_target(). Axis modes are taken into
 *	account by this.
 *
 *	The following times are compared and the longest is returned:
 *	  -	G93 inverse time (if G93 is active)
 *	  -	time for coordinated move at requested feed rate
 *	  -	time that the slowest axis would require for the move
 *
 *	Sets the following variables in the gcode_state struct
 *	  - move_time is set to optimal time
 *	  - minimum_time is set to minimum time
 */
/* --- NIST RS274NGC_v3 Guidance ---
 *
 *	The following is verbatim text from NIST RS274NGC_v3. As I interpret A for moves that
 *	combine both linear and rotational movement, the feed rate should apply to the XYZ
 *	movement, with the rotational axis (or axes) timed to start and end at the same time
 *	the linear move is performed. It is possible under this case for the rotational move
 *	to rate-limit the linear move.
 *
 * 	2.1.2.5 Feed Rate
 *
 *	The rate at which the controlled point or the axes move is nominally a steady rate
 *	which may be set by the user. In the Interpreter, the interpretation of the feed
 *	rate is as follows unless inverse time feed rate mode is being used in the
 *	RS274/NGC view (see Section 3.5.19). The canonical machining functions view of feed
 *	rate, as described in Section 4.3.5.1, has conditions under which the set feed rate
 *	is applied differently, but none of these is used in the Interpreter.
 *
 *	A. 	For motion involving one or more of the X, Y, and Z axes (with or without
 *		simultaneous rotational axis motion), the feed rate means length units per
 *		minute along the programmed XYZ path, as if the rotational axes were not moving.
 *
 *	B.	For motion of one rotational axis with X, Y, and Z axes not moving, the
 *		feed rate means degrees per minute rotation of the rotational axis.
 *
 *	C.	For motion of two or three rotational axes with X, Y, and Z axes not moving,
 *		the rate is applied as follows. Let dA, dB, and dC be the angles in degrees
 *		through which the A, B, and C axes, respectively, must move.
 *		Let D = sqrt(dA^2 + dB^2 + dC^2). Conceptually, D is a measure of total
 *		angular motion, using the usual Euclidean metric. Let T be the amount of
 *		time required to move through D degrees at the current feed rate in degrees
 *		per minute. The rotational axes should be moved in coordinated linear motion
 *		so that the elapsed time from the start to the end of the motion is T plus
 *		any time required for acceleration or deceleration.
 */
static void _calc_move_times(GCodeState_t *gms, const float axis_length[], const float axis_square[])
										// gms = Gcode model state
{
	float inv_time=0;				// inverse time if doing a feed in G93 mode
	float xyz_time=0;				// coordinated move linear part at requested feed rate
	float abc_time=0;				// coordinated move rotary part at requested feed rate
	float max_time=0;				// time required for the rate-limiting axis
	float tmp_time=0;				// used in computation
	gms->minimum_time = 8675309;	// arbitrarily large number

	// compute times for feed motion
	if (gms->motion_mode != MOTION_MODE_STRAIGHT_TRAVERSE) {
		if (gms->feed_rate_mode == INVERSE_TIME_MODE) {
			inv_time = gms->feed_rate;	// NB: feed rate was un-inverted to minutes by cm_set_feed_rate()
			gms->feed_rate_mode = UNITS_PER_MINUTE_MODE;
		} else {
			// compute length of linear move in millimeters. Feed rate is provided as mm/min
			xyz_time = sqrt(axis_square[AXIS_X] + axis_square[AXIS_Y] + axis_square[AXIS_Z]) / gms->feed_rate;

			// if no linear axes, compute length of multi-axis rotary move in degrees. Feed rate is provided as degrees/min
			if (fp_ZERO(xyz_time)) {
				abc_time = sqrt(axis_square[AXIS_A] + axis_square[AXIS_B] + axis_square[AXIS_C]) / gms->feed_rate;
			}
		}
	}
	for (uint8_t axis = AXIS_X; axis < AXES; axis++) {
		if (gms->motion_mode == MOTION_MODE_STRAIGHT_TRAVERSE) {
			tmp_time = fabs(axis_length[axis]) / cm.a[axis].velocity_max;
		} else { // MOTION_MODE_STRAIGHT_FEED
			tmp_time = fabs(axis_length[axis]) / cm.a[axis].feedrate_max;
		}
		max_time = max(max_time, tmp_time);

		if (tmp_time > 0) { 	// collect minimum time if this axis is not zero
			gms->minimum_time = min(gms->minimum_time, tmp_time);
		}
	}
	gms->move_time = max4(inv_time, max_time, xyz_time, abc_time);
}

/* _plan_block_list() - plans the entire block list
 *
 *	The block list is the circular buffer of planner buffers (bf's). The block currently
 *	being planned is the "bf" block. The "first block" is the next block to execute;
 *	queued immediately behind the currently executing block, aka the "running" block.
 *	In some cases there is no first block because the list is empty or there is only
 *	one block and it is already running.
 *
 *	_plan_block_list() plans all blocks between and including the (effective) first block
 *	and the bf. It sets entry, exit and cruise v's from vmax's then calls trapezoid generation.
 *
 *	Variables that must be provided in the mpBuffers that will be processed:
 *
 *	  bf (function arg)		- end of block list (last block in time)
 *	  bf->move_type			- typically MOVE_TYPE_ALINE. Other move_types should be set to
 *							  length=0, entry_vmax=0 and exit_vmax=0 and are treated
 *							  as a momentary stop (plan to zero and from zero).
 *
 *	  bf->length			- provides block length
 *	  bf->entry_vmax		- used during forward planning to set entry velocity
 *	  bf->cruise_vmax		- used during forward planning to set cruise velocity
 *	  bf->exit_vmax			- used during forward planning to set exit velocity
 *	  bf->delta_vmax		- used during forward planning to set exit velocity
 *
 *	  bf->recip_jerk		- used during trapezoid generation
 *	  bf->cbrt_jerk			- used during trapezoid generation
 *
 *	Variables that will be set during processing:
 *
 *	  bf->braking_velocity	- set during backward planning
 *	  bf->entry_velocity	- set during forward planning
 *	  bf->cruise_velocity	- set during forward planning
 *	  bf->exit_velocity		- set during forward planning
 *
 *	  bf->head_length		- set during trapezoid generation
 *	  bf->body_length		- set during trapezoid generation
 *	  bf->tail_length		- set during trapezoid generation
 *
 *	Variables that are ignored but here's what you would expect them to be:
 *	  bf->move_state		- NEW for all blocks but the earliest
 *	  bf->target[]			- block target position
 *	  bf->unit[]			- block unit vector
 *	  bf->time				- gets set later
 *	  bf->jerk				- source of the other jerk variables. Used in mr.
 */
/* Notes:
 *	[1] The mr_flag is used to tell replan to account for mr buffer's exit velocity (Vx)
 *		mr's Vx is always found in the provided bf buffer. Used to replan feedholds
 */
/* Normal case:
 *      Add a buffer.  Initialize it to plan from stop and to stop.  Set "replan requested".
 *      Replanning callback fires, tries to compute trapezoids for [bp0, bf]. set "replan swap requested".
 *      Exec either accepts new plan, sets "replan off"; or rejects new plan, sets "replan requested".
 *
 * Feedhold case:
 *      Update mr and bp1,bp2...bpX to be tail segments until we plan to a halt.
 *      Update bp0 to plan back from halt to original bpX endpoint. set "replan swap requested".
 *      If exec rejects the plan, we try again.
 *      If exec accepts the plan, sets replan off, feedhold decel, and requests a replan to optimally plan post-hold stuff...
 *
 * Feedrate override case:
 *      Set "replan requested" as in normal case, but first adjust the vmaxes appropriately.  TODO...
 *
 */
static void _plan_block_list(mpBuf_t *bf)
{
	mpBuf_t *bp = bf;

	// Backward planning pass. Find first block and update the braking velocities.
	// At the end *bp points to the buffer before the first block.
	while ((bp = mp_get_prev_buffer(bp)) != bf && (bp->move_state == MOVE_NEW || bp->move_state == MOVE_RUN)) {
		bp->braking_velocity = min(bp->nx->entry_vmax, bp->nx->braking_velocity) + bp->delta_vmax;
	}

	// forward planning pass - recomputes trapezoids in the list from the first block to the bf block.
	while ((bp = mp_get_next_buffer(bp)) != bf) {
        if (bp->move_state == MOVE_RUN) {
            /* Only replan the running block if the exit velocity has changed... */
            float target_exit_velocity = min4( bp->exit_vmax,
                                               bp->nx->entry_vmax,
                                               bp->nx->braking_velocity,
                                               (bp->entry_velocity + bp->delta_vmax));
            if(fp_NE(target_exit_velocity, bp->exit_velocity)) {
                /* Currently running block... take the next segment velocity, and recompute the length */
                bp->replanned_entry_velocity = _compute_next_segment_velocity();
                bp->length = get_axis_vector_length(mr.target, mr.position);
                bp->replanned_cruise_velocity = bp->cruise_vmax;
                bp->replanned_exit_velocity = target_exit_velocity;
                mp_calculate_trapezoid(bp);
            }
            continue;
        } else {
            if(bp->pv == bf || bp->pv->move_state == MOVE_OFF) {
                /* First block, but nothing's currently running */
                bp->replanned_entry_velocity = mr.exit_velocity;
            } else {
                /* bp+X where X > 0 */
                bp->replanned_entry_velocity = bp->pv->replanned_exit_velocity;
            }
            bp->replanned_exit_velocity = min4( bp->exit_vmax,
                                                bp->nx->entry_vmax,
                                                bp->nx->braking_velocity,
                                               (bp->replanned_entry_velocity + bp->delta_vmax));
        
            if(fp_EQ(bp->replanned_entry_velocity, bp->entry_velocity) &&
               fp_EQ(bp->replanned_exit_velocity, bp->exit_velocity) &&
               fp_EQ(bp->head_length + bp->body_length + bp->tail_length, bp->length)) {
                bp->replanned_head_length = bp->head_length;
                bp->replanned_body_length = bp->body_length;
                bp->replanned_tail_length = bp->tail_length;
            } else {
                bp->replanned_cruise_velocity = bp->cruise_vmax;
                mp_calculate_trapezoid(bp);
            }
        }
    }
	// finish up the last block move
	bp->replanned_entry_velocity = bp->pv->replanned_exit_velocity;
	bp->replanned_exit_velocity = 0;
    
    if(fp_EQ(bp->replanned_entry_velocity, bp->entry_velocity) &&
       fp_EQ(bp->replanned_exit_velocity, bp->exit_velocity) &&
       fp_EQ(bp->head_length + bp->body_length + bp->tail_length, bp->length)) {
        bp->replanned_head_length = bp->head_length;
        bp->replanned_body_length = bp->body_length;
        bp->replanned_tail_length = bp->tail_length;
    } else {
        bp->replanned_cruise_velocity = bp->cruise_vmax;
        mp_calculate_trapezoid(bp);
    }
}

/*
 * _get_junction_vmax() - Sonny's algorithm - simple
 *
 *  Computes the maximum allowable junction speed by finding the velocity that will yield
 *	the centripetal acceleration in the corner_acceleration value. The value of delta sets
 *	the effective radius of curvature. Here's Sonny's (Sungeun K. Jeon's) explanation
 *	of what's going on:
 *
 *	"First let's assume that at a junction we only look a centripetal acceleration to simply
 *	things. At a junction of two lines, let's place a circle such that both lines are tangent
 *	to the circle. The circular segment joining the lines represents the path for constant
 *	centripetal acceleration. This creates a deviation from the path (let's call this delta),
 *	which is the distance from the junction to the edge of the circular segment. Delta needs
 *	to be defined, so let's replace the term max_jerk (see note 1) with max_junction_deviation,
 *	or "delta". This indirectly sets the radius of the circle, and hence limits the velocity
 *	by the centripetal acceleration. Think of the this as widening the race track. If a race
 *	car is driving on a track only as wide as a car, it'll have to slow down a lot to turn
 *	corners. If we widen the track a bit, the car can start to use the track to go into the
 *	turn. The wider it is, the faster through the corner it can go.
 *
 * (Note 1: "max_jerk" refers to the old grbl/marlin max_jerk" approximation term, not the
 *	tinyG jerk terms)
 *
 *	If you do the geometry in terms of the known variables, you get:
 *		sin(theta/2) = R/(R+delta)  Re-arranging in terms of circle radius (R)
 *		R = delta*sin(theta/2)/(1-sin(theta/2).
 *
 *	Theta is the angle between line segments given by:
 *		cos(theta) = dot(a,b)/(norm(a)*norm(b)).
 *
 *	Most of these calculations are already done in the planner. To remove the acos()
 *	and sin() computations, use the trig half angle identity:
 *		sin(theta/2) = +/- sqrt((1-cos(theta))/2).
 *
 *	For our applications, this should always be positive. Now just plug the equations into
 *	the centripetal acceleration equation: v_c = sqrt(a_max*R). You'll see that there are
 *	only two sqrt computations and no sine/cosines."
 *
 *	How to compute the radius using brute-force trig:
 *		float theta = acos(costheta);
 *		float radius = delta * sin(theta/2)/(1-sin(theta/2));
 */
/*  This version extends Chamnit's algorithm by computing a value for delta that takes
 *	the contributions of the individual axes in the move into account. This allows the
 *	control radius to vary by axis. This is necessary to support axes that have
 *	different dynamics; such as a Z axis that doesn't move as fast as X and Y (such as a
 *	screw driven Z axis on machine with a belt driven XY - like a Shapeoko), or rotary
 *	axes ABC that have completely different dynamics than their linear counterparts.
 *
 *	The function takes the absolute values of the sum of the unit vector components as
 *	a measure of contribution to the move, then scales the delta values from the non-zero
 *	axes into a composite delta to be used for the move. Shown for an XY vector:
 *
 *	 	U[i]	Unit sum of i'th axis	fabs(unit_a[i]) + fabs(unit_b[i])
 *	 	Usum	Length of sums			Ux + Uy
 *	 	d		Delta of sums			(Dx*Ux+DY*UY)/Usum
 */

static float _get_junction_vmax(const float a_unit[], const float b_unit[])
{
	float costheta = - (a_unit[AXIS_X] * b_unit[AXIS_X])
					 - (a_unit[AXIS_Y] * b_unit[AXIS_Y])
					 - (a_unit[AXIS_Z] * b_unit[AXIS_Z])
					 - (a_unit[AXIS_A] * b_unit[AXIS_A])
					 - (a_unit[AXIS_B] * b_unit[AXIS_B])
					 - (a_unit[AXIS_C] * b_unit[AXIS_C]);

	if (costheta < -0.99) { return (10000000); } 		// straight line cases
	if (costheta > 0.99)  { return (0); } 				// reversal cases

	// Fuse the junction deviations into a vector sum
	float a_delta = square(a_unit[AXIS_X] * cm.a[AXIS_X].junction_dev);
		 a_delta += square(a_unit[AXIS_Y] * cm.a[AXIS_Y].junction_dev);
		 a_delta += square(a_unit[AXIS_Z] * cm.a[AXIS_Z].junction_dev);
		 a_delta += square(a_unit[AXIS_A] * cm.a[AXIS_A].junction_dev);
		 a_delta += square(a_unit[AXIS_B] * cm.a[AXIS_B].junction_dev);
		 a_delta += square(a_unit[AXIS_C] * cm.a[AXIS_C].junction_dev);

	float b_delta = square(b_unit[AXIS_X] * cm.a[AXIS_X].junction_dev);
		 b_delta += square(b_unit[AXIS_Y] * cm.a[AXIS_Y].junction_dev);
		 b_delta += square(b_unit[AXIS_Z] * cm.a[AXIS_Z].junction_dev);
		 b_delta += square(b_unit[AXIS_A] * cm.a[AXIS_A].junction_dev);
		 b_delta += square(b_unit[AXIS_B] * cm.a[AXIS_B].junction_dev);
		 b_delta += square(b_unit[AXIS_C] * cm.a[AXIS_C].junction_dev);

	float delta = (sqrt(a_delta) + sqrt(b_delta))/2;
	float sintheta_over2 = sqrt((1 - costheta)/2);
	float radius = delta * sintheta_over2 / (1-sintheta_over2);
	return(sqrt(radius * cm.junction_acceleration));
}

/*************************************************************************
 * feedholds - functions for performing holds
 *
 * mp_plan_hold_callback() - replan block list to execute hold
 * mp_end_hold() 		   - release the hold and restart block list
 *
 *	Feedhold is executed as cm.hold_state transitions executed inside
 *	_exec_aline() and main loop callbacks to these functions:
 *	mp_plan_hold_callback() and mp_end_hold().
 */
/*	Holds work like this:
 *
 * 	  - Hold is asserted by calling cm_feedhold() (usually invoked via a ! char)
 *		If hold_state is OFF and motion_state is RUNning it sets
 *		hold_state to SYNC and motion_state to HOLD.
 *
 *	  - Hold state == SYNC tells the aline exec routine to execute the next aline
 *		segment then set hold_state to PLAN. This gives the planner sufficient
 *		time to replan the block list for the hold before the next aline segment
 *		needs to be processed.
 *
 *	  - Hold state == PLAN tells the planner to replan the mr buffer, the current
 *		run buffer (bf), and any subsequent bf buffers as necessary to execute a
 *		hold. Hold planning replans the planner buffer queue down to zero and then
 *		back up from zero. Hold state is set to DECEL when planning is complete.
 *
 *	  - Hold state == DECEL persists until the aline execution runs to zero
 *		velocity, at which point hold state transitions to HOLD.
 *
 *	  - Hold state == HOLD persists until the cycle is restarted. A cycle start
 *		is an asynchronous event that sets the cycle_start_flag TRUE. It can
 *		occur any time after the hold is requested - either before or after
 *		motion stops.
 *
 *	  - mp_end_hold() is executed from cm_feedhold_sequencing_callback() once the
 *		hold state == HOLD and a cycle_start has been requested.This sets the hold
 *		state to OFF which enables _exec_aline() to continue processing. Move
 *		execution begins with the first buffer after the hold.
 *
 *	Terms used:
 *	 - mr is the runtime buffer. It was initially loaded from the bf buffer
 *	 - bp+0 is the "companion" bf buffer to the mr buffer.
 *	 - bp+1 is the bf buffer following bp+0. This runs through bp+N
 *	 - bp (by itself) just refers to the current buffer being adjusted / replanned
 *
 *	Details: Planning re-uses bp+0 as an "extra" buffer. Normally bp+0 is returned
 *		to the buffer pool as it is redundant once mr is loaded. Use the extra
 *		buffer to split the move in two where the hold decelerates to zero. Use
 *		one buffer to go to zero, the other to replan up from zero. All buffers past
 *		that point are unaffected other than that they need to be replanned for velocity.
 *
 *	Note: There are multiple opportunities for more efficient organization of
 *		  code in this module, but the code is so complicated I just left it
 *		  organized for clarity and hoped for the best from compiler optimization.
 */

static float _compute_next_segment_velocity()
{
	if (mr.section == SECTION_BODY) return (mr.segment_velocity);
	return (mr.segment_velocity + mr.forward_diff_5);
}

stat_t mp_replan_callback()
{
    if(mr.replan_state == REPLAN_REQUESTED) {
        mpBuf_t *runBf = mp_get_run_buffer();
        if(runBf != NULL) {
            mr.replan_interrupted = 0;
            _plan_block_list(mp_get_last_buffer());
            if(runBf->move_state == MOVE_RUN) {
                mr.replanned_entry_velocity = runBf->replanned_entry_velocity;
                mr.replanned_cruise_velocity = runBf->replanned_cruise_velocity;
                mr.replanned_exit_velocity = runBf->replanned_exit_velocity;
                mr.replanned_head_length = runBf->replanned_head_length;
                mr.replanned_body_length = runBf->replanned_body_length;
                mr.replanned_tail_length = runBf->replanned_tail_length;
            } else {
                mr.replanned_entry_velocity = mr.entry_velocity;
                mr.replanned_cruise_velocity = mr.cruise_velocity;
                mr.replanned_exit_velocity = mr.exit_velocity;
                mr.replanned_head_length = mr.head_length;
                mr.replanned_body_length = mr.body_length;
                mr.replanned_tail_length = mr.tail_length;
            }
            _request_replan_swap(runBf, NULL);
        } else {
            _finish_replan();
            return (STAT_NOOP);
        }
    }
    
    if(mr.replan_state == REPLAN_SWAP_REQUESTED && !st_exec_isbusy())
    {
        /* Since exec isn't running, just do this ourselves */
        mp_check_for_replan();
    }
    
    return (STAT_OK);
}

stat_t mp_plan_hold_callback()
{
	//if we're partway through a hold but the stepper chain has stopped, finish the hold
	if (cm.hold_state > FEEDHOLD_OFF && cm.hold_state < FEEDHOLD_HOLD && !st_exec_isbusy()) {
		mp_start_hold();
		return (STAT_OK);
	}
    
    //sets mr, sets vmax, calls _plan_block_list(last, 1), calls _request_replan_swap

	//otherwise, we we wait for FEEDHOLD_PLAN and then plan a DECEL buffer
	if (cm.hold_state != FEEDHOLD_PLAN) { return (STAT_NOOP);}	// not planning a feedhold
    if (mr.replan_state == REPLAN_SWAP_REQUESTED) { return (STAT_NOOP);} //we're waiting for exec to accept the current replan
    
    mr.replan_interrupted = 0;
    
    mpBuf_t *bp = mp_get_run_buffer();
    if(bp == NULL) { return (STAT_NOOP);} // Oops! nothing's running
    
    //update mr
    
    //length left in mr buffer
    float mr_available_length = get_axis_vector_length(mr.target, mr.position);
    //velocity left to shed to brake to zero
    float braking_velocity = _compute_next_segment_velocity();
    //distance required to brake to zero from braking_velocity
    float braking_length = mp_get_target_length(braking_velocity, 0, bp);
    
    // Hack to prevent Case 2 moves for perfect-fit decels. Happens in homing situations
	// The real fix: The braking velocity cannot simply be the mr.segment_velocity as this
	// is the velocity of the last segment, not the one that's going to be executed next.
	// The braking_velocity needs to be the velocity of the next segment that has not yet
	// been computed. In the mean time, this hack will work.
	if ((braking_length > mr_available_length) && (fp_ZERO(bp->exit_velocity))) {
		braking_length = mr_available_length;
	}

	// Case 1: deceleration fits entirely into the length remaining in mr buffer
	if (braking_length <= mr_available_length) {
		// set mr to a tail to perform the deceleration
        mr.replanned_head_length = mr.replanned_body_length = 0;
        mr.replanned_entry_velocity = mr.replanned_cruise_velocity = braking_velocity;
        mr.replanned_exit_velocity = 0;
        mr.replanned_tail_length = braking_length;

		// re-use bp+0 to be the hold point and to run the remaining block length
        bp->entry_vmax = 0;
        mr.replan_bp0_length = mr_available_length - braking_length;
        _request_replan_swap(bp, bp); // insert bp0 after bp0 (aka no-op)
        return (STAT_OK);
	}

	// Case 2: deceleration exceeds length remaining in mr buffer
	// First, replan mr to minimum (but non-zero) exit velocity

    mr.replanned_head_length = mr.replanned_body_length = 0;
    mr.replanned_entry_velocity = mr.replanned_cruise_velocity = braking_velocity;
    mr.replanned_tail_length = mr_available_length;
	mr.replanned_exit_velocity = braking_velocity - mp_get_target_velocity(0, mr_available_length, bp);

	// Find the point where deceleration reaches zero. This could span multiple buffers.
	braking_velocity = mr.replanned_exit_velocity;		// adjust braking velocity downward
    mpBuf_t *bp0 = bp;
    bp = bp->nx;
	for (uint8_t i=0; i<PLANNER_BUFFER_POOL_SIZE; i++) {// a safety to avoid wraparound
		if (bp->move_type != MOVE_TYPE_ALINE) {	// skip any non-move buffers
			bp = mp_get_next_buffer(bp);		// point to next buffer
			continue;
		}
        braking_length = mp_get_target_length(braking_velocity, 0, bp);
        
        bp->replanned_head_length = bp->replanned_body_length = 0;
        bp->entry_vmax = bp->cruise_vmax = bp->replanned_entry_velocity = bp->replanned_cruise_velocity = braking_velocity;
        if(braking_length > bp->length) {
            bp->exit_vmax = bp->replanned_exit_velocity = braking_velocity - mp_get_target_velocity(0, bp->length, bp);
            bp->replanned_tail_length = bp->length;
            braking_velocity = bp->replanned_exit_velocity;
            bp = mp_get_next_buffer(bp);
            continue;
        } else
            break;
	}
	// Deceleration now fits in the current bp buffer
	// Plan the bp as the decel, bp0 as the accel
    bp->replanned_tail_length = braking_length;
    bp->exit_vmax = bp->replanned_exit_velocity = 0;
    mr.replan_hold_buffer_length = braking_length;

    bp0->entry_vmax = 0;
    mr.replan_bp0_length = bp->length - braking_length;
    
    _request_replan_swap(bp0, bp);
    return (STAT_OK);
}

/*
 * mp_start_hold() - called from the stepper chain when the hold takes effect
 */
stat_t mp_start_hold()
{
    cm_spindle_control_immediate(SPINDLE_PAUSED | cm.gm.spindle_mode);
    cm.hold_state = FEEDHOLD_HOLD;
    sr_request_status_report(SR_REQUEST_IMMEDIATE);
    return (STAT_OK);
}
