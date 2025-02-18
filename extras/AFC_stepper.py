# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import math
import chelper
from kinematics import extruder
from . import AFC_assist
from configfile import error
from extras.AFC import add_filament_switch

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
        self.printer = config.get_printer()
        self.AFC = self.printer.lookup_object('AFC')
        self.gcode = self.printer.lookup_object('gcode')
        self.reactor = self.printer.get_reactor()
        self.extruder_stepper = extruder.ExtruderStepper(config)

        #stored status variables
        self.name = config.get_name().split()[-1]
        self.extruder_name = config.get('extruder', None)                                           # AFC_extruder name in config file
        self.extruder_obj = None

        self.map = config.get('cmd','NONE')                                                         # Need description
        self.tool_loaded = False
        self.loaded_to_hub = False
        self.spool_id = None
        self.material = None
        self.color = None
        self.weight = None
        self.runout_lane = 'NONE'
        self.status = 'Not Loaded'
        self.extruder_temp = None                                                                   # Extruder temp based off material
        self.unit_obj = None                                                                        # Set on unit_name:connect callback

        unit = config.get('unit', None)                                                             # Unit name(AFC_hub) that this lane belongs to.
        if unit != None:
            self.unit = unit.split(':')[0]
            self.index = int(unit.split(':')[1])
        else:
            self.unit = 'Unknown'
            self.index = 0
        self.hub= config.get("hub", None)

        self.printer.register_event_handler("{}:connect".format(self.unit),self.handle_unit_connect)
        self.printer.register_event_handler("klippy:ready", self.handle_ready)

        self.motion_queue = None
        self.next_cmd_time = 0.
        ffi_main, ffi_lib = chelper.get_ffi()
        self.trapq = ffi_main.gc(ffi_lib.trapq_alloc(), ffi_lib.trapq_free)
        self.trapq_append = ffi_lib.trapq_append
        self.trapq_finalize_moves = ffi_lib.trapq_finalize_moves
        self.stepper_kinematics = ffi_main.gc(
            ffi_lib.cartesian_stepper_alloc(b'x'), ffi_lib.free)
        self.assist_activate=False

        self.afc_bowden_length  = config.getfloat('afc_bowden_length', None)                        # Bowden distance between hub and primary extruder
        self.dist_hub           = config.getfloat('dist_hub', 60)                                   # Bowden distance between Boxturtle extruder and hub
        self.hub_clear_move_dis = config.getfloat("hub_clear_move_dis", None)                       # Distance to insure filament is clear of hub
        self.move_dis           = config.getfloat('move_dis', None)                                 # Distance to move through hub
        self.park_dist          = config.getfloat('park_dist', 10)                                  # Currently unused
        self.led_index          = config.get('led_index', None)                                     # LED index of lane in chain of lane LEDs

        # lane triggers
        buttons = self.printer.load_object(config, "buttons")
        self.prep = config.get('prep', None)                                                        # MCU pin for prep trigger
        if self.prep is not None:
            self.prep_state = False
            buttons.register_buttons([self.prep], self.prep_callback)
        self.load = config.get('load', None)                                                        # MCU pin load trigger
        if self.load is not None:
            self.load_state = False
            buttons.register_buttons([self.load], self.load_callback)

        # Respoolers
        self.afc_motor_rwd = config.get('afc_motor_rwd', None)                                      # Reverse pin on MCU for spoolers
        self.afc_motor_fwd = config.get('afc_motor_fwd', None)                                      # Forwards pin on MCU for spoolers
        self.afc_motor_enb = config.get('afc_motor_enb', None)                                      # Enable pin on MCU for spoolers
        self.afc_motor_fwd_pulse = config.getfloat('afc_motor_fwd_pulse', None)                     # Need description
        self.afc_motor_fwd_gear_ratio = config.get('afc_motor_fwd_gear_ratio', None)                # Need description
        self.afc_motor_fwd_drive_diam = config.getfloat('afc_motor_fwd_drive_diam', None)           # Need description
        if self.afc_motor_rwd is not None:
            self.afc_motor_rwd = AFC_assist.AFCassistMotor(config, 'rwd')
        if self.afc_motor_fwd is not None:
            self.afc_motor_fwd = AFC_assist.AFCassistMotor(config, 'fwd')
        if self.afc_motor_enb is not None:
            self.afc_motor_enb = AFC_assist.AFCassistMotor(config, 'enb')

        self.tmc_print_current = config.getfloat("print_current", self.AFC.global_print_current)    # Current to use while printing, set to a lower current to reduce stepper heat when printing. Defaults to global_print_current, if not specified current is not changed.
        self._get_tmc_values( config )

        self.filament_diameter = config.getfloat("filament_diameter", 1.75)                         # Diameter of filament being used
        self.filament_density = config.getfloat("filament_density", 1.24)                           # Density of filament being used
        self.inner_diameter = config.getfloat("spool_inner_diameter", 100)                          # Inner diameter in mm
        self.outer_diameter = config.getfloat("spool_outer_diameter", 200)                          # Outer diameter in mm
        self.empty_spool_weight = config.getfloat("empty_spool_weight", 190)                        # Empty spool weight in g
        self.remaining_weight = config.getfloat("spool_weight", 1000)                               # Remaining spool weight in g
        self.max_motor_rpm = config.getfloat("assist_max_motor_rpm", 500)                           # Max motor RPM
        self.rwd_speed_multi = config.getfloat("rwd_speed_multiplier", 0.5)                         # Multiplier to apply to rpm
        self.fwd_speed_multi = config.getfloat("fwd_speed_multiplier", 0.5)                         # Multiplier to apply to rpm
        self.diameter_range = self.outer_diameter - self.inner_diameter  # Range for effective diameter

        # Overrides buffers set at the unit and extruder level
        self.buffer_name = config.get("buffer_name", None)

        # Set hub loading speed depending on distance between extruder and hub
        self.dist_hub_move_speed = self.AFC.long_moves_speed if self.dist_hub >= 200 else self.AFC.short_moves_speed
        self.dist_hub_move_accel = self.AFC.long_moves_accel if self.dist_hub >= 200 else self.AFC.short_moves_accel

        # Defaulting to false so that extruder motors to not move until PREP has been called
        self._afc_prep_done = False

        # Get and save base rotation dist
        self.base_rotation_dist = self.extruder_stepper.stepper.get_rotation_distance()[0]
        self.enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", self.AFC.enable_sensors_in_gui)
        self.sensor_to_show = config.get("sensor_to_show", None)

        if self.enable_sensors_in_gui:
            if self.sensor_to_show is None or self.sensor_to_show == 'prep':
                self.prep_filament_switch_name = "filament_switch_sensor {}_prep".format(self.name)
                self.fila_prep = add_filament_switch(self.prep_filament_switch_name, self.prep, self.printer )

            if self.sensor_to_show is None or self.sensor_to_show == 'load':
                self.load_filament_switch_name = "filament_switch_sensor {}_load".format(self.name)
                self.fila_load = add_filament_switch(self.load_filament_switch_name, self.load, self.printer )

    def handle_ready(self):
        if self.unit_obj is None:
            raise error("Unit {unit} is not defined in your configuration file. Please defined unit ex. [AFC_BoxTurtle {unit}]".format(unit=self.unit))

    def handle_unit_connect(self, unit_obj):
        """
        Callback to register units object
        """
        # Saving reference to unit
        self.unit_obj = unit_obj
        self.buffer_obj = self.unit_obj.buffer_obj

        # Registering lane name in unit
        self.unit_obj.lanes[self.name] = self
        self.AFC.stepper[self.name] = self

        # TODO: Need to add error checking
        if self.hub is None:
            self.hub_obj = self.unit_obj.hub_array[list(self.unit_obj.hub_array)[0]]
        else:
            # TODO: manually lookup hub object
            self.hub_obj = self.unit_obj.hub_array[self.hub]

        if self.afc_bowden_length is None:
            self.afc_bowden_length = self.hub_obj.afc_bowden_length
        # else:
        #     self.afc_bowden_length = 900           # default value if no hub

        if self.move_dis is None:
            self.move_dis = self.hub_obj.move_dis
        # else:
        #     self.move_dis = 50                     # default value if no hub

        if self.hub_clear_move_dis is None:
            self.hub_clear_move_dis = self.hub_obj.hub_clear_move_dis
        # else:
        #     self.hub_clear_move_dis = 15           # default value if no hub

        try:
            if self.extruder_name is not None:
                self.extruder_obj = self.printer.lookup_object('AFC_extruder {}'.format(self.extruder_name))
            else:
                self.extruder_obj = self.unit_obj.extruder_obj
            self.extruder_name = self.extruder_obj.name
        except:
            error_string = 'Error: No config found for extruder: {extruder} in [AFC_stepper {stepper}]. Please make sure [AFC_extruder {extruder}] section exists in your config'.format(
                extruder=self.extruder_name, stepper=self.name )
            raise error(error_string)

        try:
            saved_variables = self.printer.lookup_object("save_variables", None)
            variable_name = "afc_cal_{}_{}".format(self.name, "dist_hub")
            if saved_variables and variable_name in saved_variables.allVariables:
                self.dist_hub = saved_variables.allVariables[variable_name]
                self.AFC.gcode.respond_info("Using dist_hub saved value({}) for AFC_stepper {}".format(self.dist_hub, self.name))
        except:
            self.AFC.ERROR.AFC_error("Problem looking up extruder saved variables", False)


        # Use buffer defined in stepper and override buffers that maybe set at the UNIT or extruder levels
        if self.buffer_name is not None:
            self.buffer_obj = self.printer.lookup_object("AFC_buffer {}".format(self.buffer_name))
        # Checking if buffer was defined in extruder if not defined in unit/hub
        elif self.buffer_obj is None and self.extruder_obj.tool_start == "buffer":
            if self.extruder_obj.buffer_name is not None:
                self.AFC.gcode.respond_info("test")
                self.buffer_obj = self.printer.lookup_object("AFC_buffer {}".format(self.extruder_obj.buffer_name))

        self.AFC.gcode.respond_info("{} Buffer Obj: {}".format(self.name, self.buffer_obj))

        self.restore_prev_state()
        
        # Send out event so that macros and be registered properly with valid lane names
        self.printer.send_event("afc_stepper:register_macros", self)
    
    def restore_prev_state(self):
        if self.unit_obj.name not in self.AFC.units_f: 
            return
        if self.name not in self.AFC.units_f[self.unit_obj.name]:
            return
        
        self.AFC.gcode.respond_info("{}".format(self.AFC.units_f[self.unit_obj.name][self.name]))
        l = self.AFC.units_f[self.unit_obj.name][self.name]

        if self.AFC.spoolman_ip !=None and l['spool_id'] != None:
            self.AFC.SPOOL.set_spoolID(self, l['spool_id'], save_vars=False)
        else:
            self.material = l['material']
            self.color = l['color']
            self.weight=l['weight']

        self.runout_lane = l['runout_lane']
        if self.runout_lane == '': self.runout_lane='NONE'
        self.map = l['map']
        if self.map != 'NONE':
            self.AFC.tool_cmds[self.map] = self.name
        # Check first for hub_loaded as this was the old name in software with version <= 1030
        if 'hub_loaded' in l: self.loaded_to_hub = l['hub_loaded']
        # Check for loaded_to_hub as this is how its being saved version > 1030
        if 'loaded_to_hub' in l: self.loaded_to_hub = l['loaded_to_hub']
        self.tool_loaded = l['tool_loaded']
        self.status = l['status']

    def get_toolhead_sensor_state(self):
        if self.extruder_obj.tool_start == "buffer":
            return self.buffer_obj.advance_state
        else:
            return self.extruder_obj.tool_start_state

    def _get_tmc_values(self, config):
        """
        Searches for TMC driver that corresponds to stepper to get run current that is specified in config
        """
        try:
            self.tmc_driver = next(config.getsection(s) for s in config.fileconfig.sections() if 'tmc' in s and config.get_name() in s)
        except:
            raise self.gcode.error("Count not find TMC for stepper {}".format(self.name))

        self.tmc_load_current = self.tmc_driver.getfloat('run_current')

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
        prev_sk = self.extruder_stepper.stepper.set_stepper_kinematics(self.stepper_kinematics)
        prev_trapq = self.extruder_stepper.stepper.set_trapq(self.trapq)
        self.extruder_stepper.stepper.set_position((0., 0., 0.))
        axis_r, accel_t, cruise_t, cruise_v = calc_move_time(distance, speed, accel)
        print_time = toolhead.get_last_move_time()
        self.trapq_append(self.trapq, print_time, accel_t, cruise_t, accel_t,
                          0., 0., 0., axis_r, 0., 0., 0., cruise_v, accel)
        print_time = print_time + accel_t + cruise_t + accel_t
        self.extruder_stepper.stepper.generate_steps(print_time)
        self.trapq_finalize_moves(self.trapq, print_time + 99999.9,
                                  print_time + 99999.9)
        self.extruder_stepper.stepper.set_trapq(prev_trapq)
        self.extruder_stepper.stepper.set_stepper_kinematics(prev_sk)
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
            # Only try to load when load state trigger is false
            if self.prep_state == True and self.load_state == False:
                x = 0
                # Check to see if the printer is printing or moving as trying to load while printer is doing something will crash klipper
                if self.AFC.is_printing():
                    self.AFC.ERROR.AFC_error("Cannot load spools while printer is actively moving or homing", False)
                    return

                while self.load_state == False and self.prep_state == True:
                    x += 1
                    self.do_enable(True)
                    self.move(10,500,400)
                    self.reactor.pause(self.reactor.monotonic() + 0.1)
                    if x> 40:
                        msg = (' FAILED TO LOAD, CHECK FILAMENT AT TRIGGER\n||==>--||----||------||\nTRG   LOAD   HUB    TOOL')
                        self.AFC.ERROR.AFC_error(msg, False)
                        self.AFC.afc_led(self.AFC.led_fault, led)
                        self.status=''
                        break
                self.status=''

                # Checking if loaded to hub(it should not be since filament was just inserted), if false load to hub. Does a fast load if hub distance is over 200mm
                if not self.loaded_to_hub and self.load_state and self.prep_state:
                    self.move(self.dist_hub, self.dist_hub_move_speed, self.dist_hub_move_accel, self.dist_hub > 200)
                    self.loaded_to_hub = True
                    if self.get_advancing():
                        self.move(self.AFC.short_move_dis * -1, self.AFC.short_moves_speed, self.AFC.short_moves_accel, True)

                self.do_enable(False)
                if self.load_state == True and self.prep_state == True:
                    self.status = 'Loaded'
                    self.AFC.afc_led(self.AFC.led_ready, led)

            elif self.name == self.AFC.current and self.AFC.IDLE.state == 'Printing' and self.load_state and self.status != 'ejecting':
                # Checking to make sure runout_lane is set and does not equal 'NONE'
                if  self.runout_lane != 'NONE':
                    self.status = None
                    self.AFC.gcode.respond_info("Infinite Spool triggered for {}, PAUSING and changing to runout lane {}".format(self.name.upper(), self.runout_lane.upper()))
                    empty_LANE = self.AFC.stepper[self.AFC.current]
                    change_LANE = self.AFC.stepper[self.runout_lane]

                    # Pause printer
                    self.gcode.run_script_from_command("PAUSE")
                    # Command T(n)
                    self.gcode.run_script_from_command(change_LANE.map)
                    # Change Mapping
                    self.gcode.run_script_from_command('SET_MAP LANE={} MAP={}'.format(change_LANE.name, empty_LANE.map))
                    # Eject lane from BT
                    self.gcode.run_script_from_command('LANE_UNLOAD LANE={}'.format(empty_LANE.name))
                    # Resume
                    self.gcode.run_script_from_command("RESUME")
                    # Set LED to not ready
                    self.AFC.afc_led(self.AFC.led_not_ready, led)
                else:
                    # Pause print
                    self.status = None
                    self.AFC.afc_led(self.AFC.led_not_ready, led)
                    self.AFC.gcode.respond_info("Runout triggered for lane {} and runout lane is not setup to switch to another lane".format(self.name))
                    self.AFC.ERROR.pause_print()
            else:
                self.status = None
                self.AFC.afc_led(self.AFC.led_not_ready, led)
        self.AFC.save_vars()

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

    def sync_print_time(self):
        toolhead = self.printer.lookup_object('toolhead')
        print_time = toolhead.get_last_move_time()
        if self.next_cmd_time > print_time:
            toolhead.dwell(self.next_cmd_time - print_time)
        else:
            self.next_cmd_time = print_time

    def sync_to_extruder(self, update_current=True):
        """
        Helper function to sync lane to extruder and set print current if specified.

        :param update_current: Sets current to specified print current when True
        """
        self.extruder_stepper.sync_to_extruder(self.extruder_name)
        if update_current: self.set_print_current()

    def unsync_to_extruder(self, update_current=True):
        """
        Helper function to un-sync lane to extruder and set load current if specified.

        :param update_current: Sets current to specified load current when True
        """
        self.extruder_stepper.sync_to_extruder(None)
        if update_current: self.set_load_current()

    def _set_current(self, current):
        """
        Helper function to update TMC current.

        :param current: Sets TMC current to specified value
        """
        if self.tmc_print_current is not None:
            self.gcode.run_script_from_command("SET_TMC_CURRENT STEPPER='{}' CURRENT={}".format(self.name, current))

    def set_load_current(self):
        """
        Helper function to update TMC current to use run current value
        """
        self._set_current( self.tmc_load_current )

    def set_print_current(self):
        """
        Helper function to update TMC current to use print current value
        """
        self._set_current( self.tmc_print_current )

    def update_rotation_distance(self, multiplier):
        self.extruder_stepper.stepper.set_rotation_distance( self.base_rotation_dist / multiplier )

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

    def check_hub_clear(self):
        if self.hub_obj is not None:
            self.AFC.gcode.respond_info('hub {} found'.format(self.hub_obj))
            return self.hub_obj.state
        else:
            self.AFC.gcode.respond_info('No hub move on')
            return False

    def check_hub_present(self):
        if self.hub_obj is not None:
            self.AFC.gcode.respond_info('hub {} found'.format(self.hub_obj))
            return self.hub_obj.state
        else:
            self.AFC.gcode.respond_info('No hub move on hub: {}'.format(self.hub_obj))
            return True

    def set_loaded(self):
        self.tool_loaded = True
        self.AFC.current = self.extruder_obj.lane_loaded = self.name
        self.status = 'Tooled'
        self.AFC.SPOOL.set_active_spool(self.spool_id)

    def set_unloaded(self):
        self.tool_loaded = False
        self.extruder_obj.lane_loaded = ""
        self.status = None
        self.AFC.current = None

    def enable_buffer(self):
      """
      Enable the buffer if `buffer_name` is set.
      Retrieves the buffer object and calls its `enable_buffer()` method to activate it.
      """
      if self.buffer_obj is not None:
         self.buffer_obj.enable_buffer()

    def disable_buffer(self):
       """
       Disable the buffer if `buffer_name` is set.
       Calls the buffer's `disable_buffer()` method to deactivate it.
       """
       if self.buffer_obj is not None:
          self.buffer_obj.disable_buffer()

    def buffer_status(self):
       """
       Retrieve the current status of the buffer.
       If `buffer_name` is set, returns the buffer's status using `buffer_status()`.
       Otherwise, returns None.
       """
       if self.buffer_obj is not None:
          return self.buffer_obj.buffer_status()

       else: return None

    def get_trailing(self):
        if self.buffer_obj is not None:
            return self.buffer_obj.trailing_state
        else: return None

    def get_advancing(self):
        if self.buffer_obj is not None:
            return self.buffer_obj.advance_state
        else: return None

def load_config_prefix(config):
    return AFCExtruderStepper(config)
