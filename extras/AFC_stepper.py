# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import chelper, stepper
from kinematics import extruder
from . import AFC_assist

#LED
BACKGROUND_PRIORITY_CLOCK = 0x7fffffff00000000
BIT_MAX_TIME=.000004
RESET_MIN_TIME=.000050
MAX_MCU_SIZE = 500  # Sanity check on LED chain length
def calc_move_time(dist, speed, accel):
    """
    Calculate the movement time and parameters for a given distance, speed, and acceleration.
    This function computes the axis direction, acceleration time, cruise time, and cruise speed
    required to move a specified distance with given speed and acceleration.
    Parameters:
    dist (float): The distance to move.
    speed (float): The speed of the movement.
    accel (float): The acceleration of the movement.
    Returns:
    tuple: A tuple containing:
        - axis_r (float): The direction of the axis (1 for positive, -1 for negative).
        - accel_t (float): The time spent accelerating.
        - cruise_t (float): The time spent cruising at constant speed.
        - speed (float): The cruise speed.
    """
    axis_r = 1.
    if dist < 0.:
        axis_r = -1.
        dist = -dist
    if not accel or not dist:
        return axis_r, 0., dist / speed, speed
    max_cruise_v2 = dist * accel
    if max_cruise_v2 < speed**2:
        speed = math.sqrt(max_cruise_v2)
    accel_t = speed / accel
    accel_decel_d = accel_t * speed
    cruise_t = (dist - accel_decel_d) / speed
    return axis_r, accel_t, cruise_t, speed

class AFCExtruderStepper:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.name = config.get_name().split()[-1]
        self.reactor = self.printer.get_reactor()
        self.extruder_stepper = stepper.PrinterStepper(config)
        self.steppers = [self.extruder_stepper]
        self.extruder_name = config.get('extruder')
        self.gcode_cmd = config.get('cmd',None)
        self.calc_position_from_coord = self.extruder_stepper.calc_position_from_coord

        self.motion_queue = None
        self.status = None
        self.hub_load = False
        self.next_cmd_time = 0.
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.extruder_stepper.setup_itersolve('cartesian_stepper_alloc', b'x')
        self.assist_activate = False
        self.gcode = self.printer.lookup_object('gcode')

        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)

        # Units
        unit = config.get('unit', None)
        if unit != None:
            self.unit = unit.split(':')[0]
            self.index = int(unit.split(':')[1])
        else:
            self.unit = 'Unknown'
            self.index = 0
        self.hub = ''
        self.hub_dist = config.getfloat('hub_dist',20)
        self.dist_hub = config.getfloat('dist_hub', 60)
        # distance to retract filament from the hub
        self.park_dist = config.getfloat('park_dist', 10)
        self.led_index = config.get('led_index', None)
        self.endstops  = []
        self.endstop_map = {}
        self.past_position = None
        self.estimated_print_time = None

        # look at next
        # endstop = mcu.MCU_endstop(mcu, {'pin': HUB_PIN, 'pullup': True, 'invert': False})
        # endstop.add_stepper(CUR_LANE.stepper)
        # trigger_completion = endstop.home_start(print_time, sample_time, sample_count, rest_time, triggered=True)
        # endstop.home_wait(home_end_time)


        # lane triggers
        buttons = self.printer.load_object(config, "buttons")
        self.prep = config.get('prep', None)
        if self.prep is not None:
            self.prep_state = False
            buttons.register_buttons([self.prep], self.prep_callback)
        self.load = config.get('load', None)
        if self.load is not None:
            self.load_state = False
            buttons.register_buttons([self.load], self.load_callback)

        # Respoolers
        self.afc_motor_rwd = config.get('afc_motor_rwd', None)
        self.afc_motor_fwd = config.get('afc_motor_fwd', None)
        self.afc_motor_enb = config.get('afc_motor_enb', None)
        self.afc_motor_fwd_pulse = config.getfloat('afc_motor_fwd_pulse', None)
        self.afc_motor_fwd_gear_ratio = config.get('afc_motor_fwd_gear_ratio', None)
        self.afc_motor_fwd_drive_diam = config.getfloat('afc_motor_fwd_drive_diam', None)
        if self.afc_motor_rwd is not None:
            self.afc_motor_rwd = AFC_assist.AFCassistMotor(config, 'rwd')
        if self.afc_motor_fwd is not None:
            self.afc_motor_fwd = AFC_assist.AFCassistMotor(config, 'fwd')
        if self.afc_motor_enb is not None:
            self.afc_motor_enb = AFC_assist.AFCassistMotor(config, 'enb')
        self.AFC = self.printer.lookup_object('AFC')
        self.gcode = self.printer.lookup_object('gcode')

        self.filament_diameter  = config.getfloat("filament_diameter", 1.75)
        self.filament_density   = config.getfloat("filament_density", 1.24)
        self.inner_diameter     = config.getfloat("spool_inner_diameter", 100)  # Inner diameter in mm
        self.outer_diameter     = config.getfloat("spool_outer_diameter", 200)  # Outer diameter in mm
        self.empty_spool_weight = config.getfloat("empty_spool_weight", 190)  # Empty spool weight in g
        self.remaining_weight   = config.getfloat("spool_weight", 1000)  # Remaining spool weight in g
        self.max_motor_rpm      = config.getfloat("assist_max_motor_rpm", 500)  # Max motor RPM
        self.rwd_speed_multi    = config.getfloat("rwd_speed_multiplier", 0.5) # Multiplier to apply to rpm
        self.fwd_speed_multi    = config.getfloat("fwd_speed_multiplier", 0.5) # Multiplier to apply to rpm
        self.diameter_range     = self.outer_diameter - self.inner_diameter  # Range for effective diameter

        # Set hub loading speed depending on distance between extruder and hub
        self.dist_hub_move_speed = self.AFC.long_moves_speed if self.dist_hub >= 200 else self.AFC.short_moves_speed
        self.dist_hub_move_accel = self.AFC.long_moves_accel if self.dist_hub >= 200 else self.AFC.short_moves_accel

        # Defaulting to false so that extruder motors to not move until PREP has been called
        self._afc_prep_done = False

        # Get and save base rotation dist
        self.base_rotation_dist = self.extruder_stepper.get_rotation_distance()[0]

        # manual filament home
        self.gcode.register_mux_command('MANUAL_FILAMENT_HOME', "LANE", self.name, self.cmd_MANUAL_FILAMENT_HOME, desc=self.cmd_MANUAL_FILAMENT_HOME_help)

    def _handle_connect(self):
        toolhead = self.printer.lookup_object('toolhead')
        toolhead.register_step_generator(self.extruder_stepper.generate_steps)

    def sync_to_extruder(self, extruder_name):
        toolhead = self.printer.lookup_object('toolhead')
        PrinterExtruder = self.printer.lookup_object('extruder')
        toolhead.flush_step_generation()
        if not extruder_name:
            self.extruder_stepper.set_trapq(None)
            self.motion_queue = None
            return
        extruder = self.printer.lookup_object(extruder_name, None)
        # if extruder is None or not isinstance(extruder, PrinterExtruder):
        #     raise self.printer.command_error("'%s' is not a valid extruder."
        #                                      % (extruder_name,))
        self.extruder_stepper.set_position([extruder.last_position, 0., 0.])
        self.extruder_stepper.set_trapq(extruder.get_trapq())
        self.motion_queue = extruder_name

    def assist(self, value, is_resend=False):
        if self.afc_motor_rwd is None:
            return
        if value < 0:
            value *= -1
            assit_motor=self.afc_motor_rwd
        elif value > 0:
            if self.afc_motor_fwd is None:
                    return
            else:
                assit_motor=self.afc_motor_fwd
        elif value == 0:
            toolhead = self.printer.lookup_object('toolhead')
            toolhead.register_lookahead_callback(lambda print_time: self.afc_motor_rwd._set_pin(print_time, value))
            if self.afc_motor_fwd is not None:
                toolhead.register_lookahead_callback(lambda print_time: self.afc_motor_fwd._set_pin(print_time, value))
            return
        value /= assit_motor.scale
        if not assit_motor.is_pwm and value not in [0., 1.]:
            if value > 0:
                value = 1
        # Obtain print_time and apply requested settings
        toolhead = self.printer.lookup_object('toolhead')
        if self.afc_motor_enb is not None:
            if value != 0:
                enable = 1
            else:
                enable = 0
            toolhead.register_lookahead_callback(
            lambda print_time: self.afc_motor_enb._set_pin(print_time, enable))
        toolhead.register_lookahead_callback(
            lambda print_time: assit_motor._set_pin(print_time, value))

    def get_name(self):
        return self.name

    def get_extruder_pos(self, eventtime=None):
        if eventtime is None:
            eventtime = self.reactor.monotonic()
        self.estimated_print_time = (self.printer.lookup_object('mcu').estimated_print_time)
        print_time = self.estimated_print_time(eventtime)
        last_position = self.extruder_stepper.get_past_mcu_position(print_time)

        if self.past_position is None or last_position > self.past_position:
            self.past_position = last_position
            #if self.debug and last_position > 0: self.gcode.respond_info("Extruder last position: {}".format(last_position))
            return last_position

        else:
            return self.past_position

    def setup_endstop(self, pin, pin_name, stepper):
        """
        Configure and register an MCU endstop.

        This method sets up an endstop by parsing the pin parameters, initializing 
        the MCU endstop, and registering it in the endstop map and query system. 
        It also links the provided stepper motor to the endstop for movement 
        coordination.

        Args:
            pin (str): The pin identifier for the endstop.
            pin_name (str): The name to associate with the endstop in the system.
            stepper (Stepper): The stepper motor to be linked to the endstop.

        Returns:
            MCU_endstop: The configured MCU endstop object.
        """
        endstop_pin = pin
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.parse_pin(endstop_pin, True, True)
        mcu_endstop = ppins.setup_pin('endstop', endstop_pin)
        self.endstop_map[pin_name] = {'endstop': mcu_endstop,
                                      'invert': pin_params['invert'],
                                      'pullup': pin_params['pullup']}
        name = pin_name
        query_endstops = self.printer.load_object(self.config, 'query_endstops')
        query_endstops.register_endstop(mcu_endstop, name)
        mcu_endstop.add_stepper(stepper)
        return (mcu_endstop, name)


    # filament homing
    # Use function to send a homing command to move filament to "endstop"
    def do_homing_move(self, endstop, movepos, speed, accel, triggered, check_trigger):
        self.homing_accel = accel
        pos = [movepos, 0., 0., 0.]
        phoming = self.printer.lookup_object('homing')
        phoming.manual_home(self, endstop, pos, speed, triggered, check_trigger)

    cmd_MANUAL_FILAMENT_HOME_help = "send filament from a lane to an 'endstop'"
    def cmd_MANUAL_FILAMENT_HOME(self, gcmd):
        """
        Manually send filament from a specified lane to an endstop.

        Args:
            gcmd (object): The G-code command object containing parameters for the homing move.

        Parameters:
            - LANE (str): The name of the lane to send the filament from.
            - BUFFER (str): The buffer name, used for filament advance if specified.
            - SPEED (float): The speed of the move (default 50).
            - ACCEL (float): The acceleration for the move (default 150).
            - MOVE (float): The target position for the homing move (default 200).
            - STOP (str): The type of endstop to home to ("HUB" or "TOOL").

        Behavior:
            - Looks up the lane and hub objects.
            - Sends the filament to the specified endstop based on the provided parameters.

        Example:
            To send filament from lane 1 to the HUB endstop at 100mm with speed 60:

            MANUAL_FILAMENT_HOME LANE=lane1 STOP=HUB SPEED=60 MOVE=100

            To send filament from lane 2 to the TOOL endstop with acceleration 200:

            MANUAL_FILAMENT_HOME LANE=lane2 BUFFER=TN2 STOP=TOOL ACCEL=200
        """
        lane        = gcmd.get('LANE', None)
        buffer_name = gcmd.get('BUFFER', None)
        speed       = gcmd.get_float('SPEED', 50, above=0.)
        accel       = gcmd.get_float('ACCEL', 150, minval=0.)
        movepos     = gcmd.get_float('MOVE', 200)
        homing_move = 1
        endstop     = gcmd.get('STOP',None)
        endstops    = []

        if lane != None:
            CUR_LANE = self.printer.lookup_object('AFC_stepper ' + lane)
            CUR_HUB = self.printer.lookup_object('AFC_hub '+ CUR_LANE.unit)
            HUB_PIN = '{}'.format(CUR_HUB.switch_pin)
        else: return

        if buffer_name != None:
            buffer = self.printer.lookup_object('AFC_buffer ' + buffer_name)
            buffer_pin = '{}'.format(buffer.advance_pin)

        # Establish endstops based on STOP selected
        if endstop == "HUB":
            stop = self.setup_endstop(HUB_PIN, CUR_LANE.unit, CUR_LANE.extruder_stepper)
            endstops.append(stop)
        elif endstop == "TOOL" and buffer_name != None:
            stop = self.setup_endstop(buffer_pin, buffer_name, CUR_LANE.extruder_stepper)
            endstops.append(stop)
        else: return

        if endstops != None:
            self.do_enable(True)
            starting_pos = self.get_extruder_pos()
            self.do_homing_move(endstops, movepos, speed, accel, homing_move > 0, abs(homing_move) == 1)
            ending_pos = self.get_extruder_pos()
            dis_traveled = ending_pos - starting_pos
            self.gcode_respond_info('Distance traveled when switch engagged: {}'.format(dis_traveled))


    def move(self, distance, speed, accel, assist_active=False):
        """
        Move the specified lane a given distance with specified speed and acceleration.
        This function calculates the movement parameters and commands the stepper motor
        to move the lane accordingly.
        Parameters:
        distance (float): The distance to move.
        speed (float): The speed of the movement.
        accel (float): The acceleration of the movement.
        """

        if assist_active:
            self.update_remaining_weight(distance)
            if distance < 0:
                # Calculate Rewind Speed
                value = self.calculate_pwm_value(speed, True) * -1
            else:
                # Calculate Forward Assist Speed
                value = self.calculate_pwm_value(speed)

            # Clamp value to a maximum of 1
            if value > 1:
                value = 1
            self.assist(value)  # Activate assist motor with calculated value

        toolhead = self.printer.lookup_object('toolhead')
        toolhead.flush_step_generation()
        prev_sk = self.extruder_stepper.set_stepper_kinematics(self.extruder_stepper._stepper_kinematics)
        prev_trapq = self.extruder_stepper.set_trapq(self.trapq)
        self.extruder_stepper.set_position((0., 0., 0.))
        axis_r, accel_t, cruise_t, cruise_v = calc_move_time(distance, speed, accel)
        print_time = toolhead.get_last_move_time()
        self.trapq_append(self.trapq, print_time, accel_t, cruise_t, accel_t,
                          0., 0., 0., axis_r, 0., 0., 0., cruise_v, accel)
        print_time = print_time + accel_t + cruise_t + accel_t
        self.extruder_stepper.generate_steps(print_time)
        self.trapq_finalize_moves(self.trapq, print_time + 99999.9,
                                  print_time + 99999.9)
        self.extruder_stepper.set_trapq(prev_trapq)
        self.extruder_stepper.set_stepper_kinematics(prev_sk)
        toolhead.note_mcu_movequeue_activity(print_time)
        toolhead.dwell(accel_t + cruise_t + accel_t)
        toolhead.flush_step_generation()
        toolhead.wait_moves()
        if assist_active: self.assist(0)

    def set_afc_prep_done(self):
        """
        set_afc_prep_done function should only be called once AFC PREP function is done. Once this
            function is called it sets afc_prep_done to True. Once this is done the prep_callback function will
            now load once filament is inserted.
        """
        self._afc_prep_done = True
    def load_callback(self, eventtime, state):
        self.load_state = state

    def prep_callback(self, eventtime, state):
        self.prep_state = state
        # Checking to make sure printer is ready and making sure PREP has been called before trying to load anything
        if self.printer.state_message == 'Printer is ready' and True == self._afc_prep_done:
            led = self.led_index
            if self.prep_state == True:
                x = 0
                while self.load_state == False and self.prep_state == True:
                    x += 1
                    self.do_enable(True)
                    self.move(10,500,400)
                    self.reactor.pause(self.reactor.monotonic() + 0.1)
                    if x> 40:
                        msg = (' FAILED TO LOAD, CHECK FILAMENT AT TRIGGER\n||==>--||----||------||\nTRG   LOAD   HUB    TOOL')
                        self.AFC.AFC_error(msg)
                        self.AFC.afc_led(self.AFC.led_fault, led)
                        self.status=''
                        break
                self.status=''
                self.do_enable(False)
                if self.load_state == True and self.prep_state == True:
                    self.status = 'Loaded'
                    self.AFC.afc_led(self.AFC.led_ready, led)
            else:
                self.status = None
                self.AFC.afc_led(self.AFC.led_not_ready, led)

    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time

    def do_enable(self, enable):
        self.sync_print_time()
        stepper_enable = self.printer.lookup_object('stepper_enable')
        if enable:
            se = stepper_enable.lookup_enable('AFC_stepper ' + self.name)
            se.motor_enable(self.next_cmd_time)
        else:
            se = stepper_enable.lookup_enable('AFC_stepper ' + self.name)
            se.motor_disable(self.next_cmd_time)
        self.sync_print_time()

    def update_rotation_distance(self, multiplier):
        self.extruder_stepper.set_rotation_distance( self.base_rotation_dist / multiplier )

    def calculate_effective_diameter(self, weight_g, spool_width_mm=60):

        # Calculate the cross-sectional area of the filament
        density_g_mm3 = self.filament_density / 1000.0
        filament_cross_section_mm2 = 3.14159 * (self.filament_diameter / 2) ** 2
        filament_volume_mm3 = weight_g / density_g_mm3
        filament_length_mm = filament_volume_mm3 / filament_cross_section_mm2
        filament_area_mm2 = filament_length_mm * self.filament_diameter / spool_width_mm
        spool_outer_diameter_mm2 = (4 * filament_area_mm2 / 3.14159) + self.inner_diameter ** 2
        spool_outer_diameter_mm = spool_outer_diameter_mm2 ** 0.5

        return spool_outer_diameter_mm

    def calculate_rpm(self, feed_rate):
        """
        Calculate the RPM for the assist motor based on the filament feed rate.

        :param feed_rate: Filament feed rate in mm/s
        :return: Calculated RPM for the assist motor
        """
        if self.remaining_weight <= self.empty_spool_weight:
            return 0  # No filament left to assist

        # Calculate the effective diameter
        effective_diameter = self.calculate_effective_diameter(self.remaining_weight)

        # Calculate RPM
        rpm = (feed_rate * 60) / (math.pi * effective_diameter)
        return min(rpm, self.max_motor_rpm)  # Clamp to max motor RPM

    def calculate_pwm_value(self, feed_rate, rewind=False):
        """
        Calculate the PWM value for the assist motor based on the feed rate.

        :param feed_rate: Filament feed rate in mm/s
        :return: PWM value between 0 and 1
        """
        rpm = self.calculate_rpm(feed_rate)
        if not rewind:
            pwm_value = rpm / (self.max_motor_rpm / (1 + 9 * self.fwd_speed_multi))
        else:
            pwm_value = rpm / (self.max_motor_rpm / (15 + 15 * self.rwd_speed_multi))
        return max(0.0, min(pwm_value, 1.0))  # Clamp the value between 0 and 1

    def update_remaining_weight(self, distance_moved):
        """
        Update the remaining filament weight based on the filament distance moved.

        :param distance_moved: Distance of filament moved in mm.
        """
        filament_volume_mm3 = math.pi * (self.filament_diameter / 2) ** 2 * distance_moved
        filament_weight_change = filament_volume_mm3 * self.filament_density / 1000  # Convert mm cubed to g
        self.remaining_weight -= filament_weight_change

        if self.remaining_weight < self.empty_spool_weight:
            self.remaining_weight = self.empty_spool_weight  # Ensure weight doesn't drop below empty spool weight

    # wrappers to support homing
    def flush_step_generation(self):
        self.sync_print_time()
    def get_position(self):
        return [self.extruder_stepper.get_commanded_position(), 0., 0., 0.]
    def set_position(self, newpos, homing_axes=()):
        self.do_set_position(newpos[0])
    def get_last_move_time(self):
        self.sync_print_time()
        return self.next_cmd_time
    def dwell(self, delay):
        self.next_cmd_time += max(0., delay)
    def drip_move(self, newpos, speed, drip_completion):
        self.do_move(newpos[0], speed, self.homing_accel)
    def get_kinematics(self):
        return self
    def get_steppers(self):
        return list(self.steppers)
    def calc_position(self, stepper_positions):
        return [stepper_positions[self.get_name()], 0., 0.]

def load_config_prefix(config):
    return AFCExtruderStepper(config)
