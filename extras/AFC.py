# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.


import json
import os
from configparser import Error as error

AFC_VERSION="1.0.0"

class afc:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.printer.register_event_handler("klippy:connect",self.handle_connect)

        # Registering stepper callback so that mux macro can be set properly with valid lane names
        self.printer.register_event_handler("afc_stepper:register_macros",self.register_lane_macros)

        self.SPOOL = self.printer.load_object(config,'AFC_spool')
        self.ERROR = self.printer.load_object(config,'AFC_error')
        self.IDLE = self.printer.load_object(config,'idle_timeout')
        self.gcode = self.printer.lookup_object('gcode')

        self.gcode_move = self.printer.load_object(config, 'gcode_move')
        self.VarFile = config.get('VarFile')
        self.current = None
        self.error_state = False
        self.units = {}
        self.units_f = {}
        self.extruders = {}
        self.extruders_f = {}
        self.stepper = {}
        self.tool_cmds={}
        self.afc_monitoring = False
        self.number_of_toolchanges  = 0
        self.current_toolchange     = 0

        self.desired_order_list = config.get('Vdesired_order_list','')

        # tool position when tool change was requested
        self.change_tool_pos = None
        self.in_toolchange = False
        self.tool_start = None

        # Save/resume pos variables
        self.base_position = [0.0, 0.0, 0.0, 0.0]
        self.last_gcode_position = [0.0, 0.0, 0.0, 0.0]
        self.last_toolhead_position = [0.0, 0.0, 0.0, 0.0]
        self.homing_position = [0.0, 0.0, 0.0, 0.0]
        self.speed = 25.
        self.absolute_coord = True

        # SPOOLMAN
        self.spoolman_ip = config.get('spoolman_ip', None)                          # To utilize spoolman enter spoolmans IP address
        self.spoolman_port = config.get('spoolman_port', None)                      # To utilize spoolman enter spoolmans port

        #LED SETTINGS
        self.ind_lights = None
        self.led_name = config.get('led_name')                                      # Not used removed?
        self.led_fault =config.get('led_fault','1,0,0,0')                           # LED color to set when faults occur in lane        (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_ready = config.get('led_ready','1,1,1,1')                          # LED color to set when lane is ready               (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_not_ready = config.get('led_not_ready','1,1,0,0')                  # LED color to set when lane not ready              (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_loading = config.get('led_loading','1,0,0,0')                      # LED color to set when lane is loading             (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_prep_loaded = config.get('led_loading','1,1,0,0')                  # LED color to set when lane is loaded              (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_unloading = config.get('led_unloading','1,1,.5,0')                 # LED color to set when lane is unloading           (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_tool_loaded = config.get('led_tool_loaded','1,1,0,0')              # LED color to set when lane is loaded into tool    (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_advancing = config.get('led_buffer_advancing','0,0,1,0')           # LED color to set when buffer is advancing         (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_trailing = config.get('led_buffer_trailing','0,1,0,0')             # LED color to set when buffer is trailing          (R,G,B,W) 0 = off, 1 = full brightness.
        self.led_buffer_disabled = config.get('led_buffer_disable', '0,0,0,0.25')   # LED color to set when buffer is disabled          (R,G,B,W) 0 = off, 1 = full brightness.

        # TOOL Cutting Settings
        self.tool = ''
        self.tool_cut = config.getboolean("tool_cut", False)                        # Set to True to enable toolhead cutting
        self.tool_cut_cmd = config.get('tool_cut_cmd', None)                        # Macro to use when doing toolhead cutting. Change macro name if you would like to use your own cutting macro

        # CHOICES
        self.park = config.getboolean("park", False)                                # Set to True to enable parking during unload
        self.park_cmd = config.get('park_cmd', None)                                # Macro to use when parking. Change macro name if you would like to use your own park macro
        self.kick = config.getboolean("kick", False)                                # Set to True to enable poop kicking after lane loads
        self.kick_cmd = config.get('kick_cmd', None)                                # Macro to use when kicking. Change macro name if you would like to use your own kick macro
        self.wipe = config.getboolean("wipe", False)                                # Set to True to enable nozzle wipeing after lane loads
        self.wipe_cmd = config.get('wipe_cmd', None)                                # Macro to use when nozzle wipeing. Change macro name if you would like to use your own wipe macro
        self.poop = config.getboolean("poop", False)                                # Set to True to enable pooping(purging color) after lane loads
        self.poop_cmd = config.get('poop_cmd', None)                                # Macro to use when pooping. Change macro name if you would like to use your own poop/purge macro

        self.form_tip = config.getboolean("form_tip", False)                        # Set to True to tip forming when unloading lanes
        self.form_tip_cmd = config.get('form_tip_cmd', None)                        # Macro to use when tip forming. Change macro name if you would like to use your own tip forming macro

        # MOVE SETTINGS
        self.tool_sensor_after_extruder = config.getfloat("tool_sensor_after_extruder", 0) # Currently unused
        self.long_moves_speed = config.getfloat("long_moves_speed", 100)            # Speed in mm/s to move filament when doing long moves
        self.long_moves_accel = config.getfloat("long_moves_accel", 400)            # Acceleration in mm/s squared when doing long moves
        self.short_moves_speed = config.getfloat("short_moves_speed", 25)           # Speed in mm/s to move filament when doing short moves
        self.short_moves_accel = config.getfloat("short_moves_accel", 400)          # Acceleration in mm/s squared when doing short moves
        self.short_move_dis = config.getfloat("short_move_dis", 10)                 # Move distance in mm for failsafe moves.
        self.tool_max_unload_attempts = config.getint('tool_max_unload_attempts', 2)# Max number of attempts to unload filament from toolhead when using buffer as ramming sensor
        self.tool_max_load_checks = config.getint('tool_max_load_checks', 4)        # Max number of attempts to check to make sure filament is loaded into toolhead extruder when using buffer as ramming sensor

        self.z_hop =config.getfloat("z_hop", 0)                                     # Height to move up before and after a tool change completes
        self.xy_resume =config.getboolean("xy_resume", False)                       # Need description or remove as this is currently an unused variable
        self.resume_speed =config.getfloat("resume_speed", 0)                       # Speed mm/s of resume move. Set to 0 to use gcode speed
        self.resume_z_speed = config.getfloat("resume_z_speed", 0)                  # Speed mm/s of resume move in Z. Set to 0 to use gcode speed

        self.global_print_current = config.getfloat("global_print_current", None)   # Global variable to set steppers current to a specified current when printing. Going lower than 0.6 may result in TurtleNeck buffer's not working correctly

        self.enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", False)
        self._update_trsync(config)

        self.VarFile = config.get('VarFile')                                        # Path to the variables file for AFC configuration.

        # Get debug and cast to boolean
        #self.debug = True == config.get('debug', 0)
        self.debug = False

        self._load_last_state()

        # Printing here will not display in console but it will go to klippy.log
        self.print_version()

    def _update_trsync(self, config):
        # Logic to update trsync values
        update_trsync = config.getboolean("trsync_update", False)                   # Set to true to enable updating trsync value in klipper mcu. Enabling this and updating the timeouts can help with Timer Too Close(TTC) errors
        if update_trsync:
            try:
                import mcu
                trsync_value = config.getfloat("trsync_timeout", 0.05)              # Timeout value to update in klipper mcu. Klippers default value is 0.025
                trsync_single_value = config.getfloat("trsync_single_timeout", 0.5) # Single timeout value to update in klipper mcu. Klippers default value is 0.250
                self.gcode.respond_info("Applying TRSYNC update")

                # Making sure value exists as kalico(danger klipper) does not have TRSYNC_TIMEOUT value
                if( hasattr(mcu, "TRSYNC_TIMEOUT")): mcu.TRSYNC_TIMEOUT = max(mcu.TRSYNC_TIMEOUT, trsync_value)
                else : self.gcode.respond_info("TRSYNC_TIMEOUT does not exist in mcu file, not updating")

                if( hasattr(mcu, "TRSYNC_SINGLE_MCU_TIMEOUT")): mcu.TRSYNC_SINGLE_MCU_TIMEOUT = max(mcu.TRSYNC_SINGLE_MCU_TIMEOUT, trsync_single_value)
                else : self.gcode.respond_info("TRSYNC_SINGLE_MCU_TIMEOUT does not exist in mcu file, not updating")
            except Exception as e:
                self.gcode.respond_info("Unable to update TRSYNC_TIMEOUT: {}".format(e))

    def _load_last_state(self):
        ## load Unit variables
        if os.path.exists(self.VarFile + '.unit') and os.stat(self.VarFile + '.unit').st_size > 0:
            self.units_f=json.load(open(self.VarFile + '.unit'))
        ## load Toolhead variables
        if os.path.exists(self.VarFile + '.tool') and os.stat(self.VarFile + '.tool').st_size > 0:
            self.extruders_f=json.load(open(self.VarFile + '.tool'))

    def register_lane_macros(self, lane_obj):
        """
        Callback function to register macros with proper lane names so that klipper errors out correctly when users supply lanes that 
        are not valid

        :param lane_obj: object for lane to register
        """
        self.gcode.register_mux_command('LANE_MOVE',    "LANE", lane_obj.name, self.cmd_LANE_MOVE,      desc=self.cmd_LANE_MOVE_help)
        self.gcode.register_mux_command('TEST',         "LANE", lane_obj.name, self.cmd_TEST,           desc=self.cmd_TEST_help)
        self.gcode.register_mux_command('HUB_LOAD',     "LANE", lane_obj.name, self.cmd_HUB_LOAD,       desc=self.cmd_HUB_LOAD_help)
        self.gcode.register_mux_command('LANE_UNLOAD',  "LANE", lane_obj.name, self.cmd_LANE_UNLOAD,    desc=self.cmd_LANE_UNLOAD_help)
        self.gcode.register_mux_command('TOOL_LOAD',    "LANE", lane_obj.name, self.cmd_TOOL_LOAD,      desc=self.cmd_TOOL_LOAD_help)
        self.gcode.register_mux_command('TOOL_UNLOAD',  "LANE", lane_obj.name, self.cmd_TOOL_UNLOAD,    desc=self.cmd_TOOL_UNLOAD_help)
        

    def handle_connect(self):
        """
        Handle the connection event.
        This function is called when the printer connects. It looks up the toolhead object
        and assigns it to the instance variable `self.toolhead`.
        """

        self.toolhead = self.printer.lookup_object('toolhead')
        
        self.gcode.register_command('CHANGE_TOOL', self.cmd_CHANGE_TOOL, desc=self.cmd_CHANGE_TOOL_help)
        self.gcode.register_command('HUB_CUT_TEST', self.cmd_HUB_CUT_TEST, desc=self.cmd_HUB_CUT_TEST_help)
        self.gcode.register_mux_command('SET_BOWDEN_LENGTH', 'AFC', None, self.cmd_SET_BOWDEN_LENGTH, desc=self.cmd_SET_BOWDEN_LENGTH_help)
        self.gcode.register_command('AFC_STATUS', self.cmd_AFC_STATUS, desc=self.cmd_AFC_STATUS_help)
        self.gcode.register_command('SET_AFC_TOOLCHANGES', self.cmd_SET_AFC_TOOLCHANGES)

    def print_version(self):
        import subprocess
        import os
        afc_dir  = os.path.dirname(os.path.realpath(__file__))
        git_hash = subprocess.check_output(['git', '-C', '{}'.format(afc_dir), 'rev-parse', '--short', 'HEAD']).decode('ascii').strip()
        git_commit_num = subprocess.check_output(['git', '-C', '{}'.format(afc_dir), 'rev-list', 'HEAD', '--count']).decode('ascii').strip()
        self.gcode.respond_info("AFC Version: v{}-{}-{}".format(AFC_VERSION, git_commit_num, git_hash))

    def cmd_SET_AFC_TOOLCHANGES(self, gcmd):
        self.number_of_toolchanges  = gcmd.get_int("TOOLCHANGES")
        self.current_toolchange     = 0 # Reset back to one
        if self.number_of_toolchanges > 0: 
            self.gcode.respond_info("Total number of toolchanges set to {}".format(self.number_of_toolchanges))

    cmd_AFC_STATUS_help = "Return current status of AFC"
    def cmd_AFC_STATUS(self, gcmd):
        """
        This function generates a status message for each unit and lane, indicating the preparation,
        loading, hub, and tool states. The status message is formatted with HTML tags for display.

        Usage: `AFC_STATUS`
        Example: `AFC_STATUS`

        Args:
            gcmd: The G-code command object containing the parameters for the command.

        Returns:
            None
        """
        status_msg = ''

        for UNIT in self.units.values():
            # Find the maximum length of lane names to determine the column width
            self.gcode.respond_info("Status {}".format(UNIT))
            max_lane_length = max(len(lane) for lane in UNIT.lanes.keys())

            status_msg += '<span class=info--text>{} Status</span>\n'.format(UNIT.name)

            # Create a dynamic format string that adjusts based on lane name length
            header_format = '{:<{}} | Prep | Load |\n'
            status_msg += header_format.format("LANE", max_lane_length)

            for CUR_LANE in UNIT.lanes.values():
                lane_msg = ''
                if self.current != None:
                    if self.current == CUR_LANE.name:
                        if not CUR_LANE.extruder_obj.tool_start_state or not CUR_LANE.hub_obj.state:
                            lane_msg += '<span class=warning--text>{:<{}} </span>'.format(CUR_LANE.name.upper(), max_lane_length)
                        else:
                            lane_msg += '<span class=success--text>{:<{}} </span>'.format(CUR_LANE.name.upper(), max_lane_length)
                    else:
                        lane_msg += '{:<{}} '.format(CUR_LANE.name.upper(),max_lane_length)
                else:
                    lane_msg += '{:<{}} '.format(CUR_LANE.name.upper(),max_lane_length)

                if CUR_LANE.prep_state == True:
                    lane_msg += '| <span class=success--text><--></span> |'
                else:
                    lane_msg += '|  <span class=error--text>xx</span>  |'
                if CUR_LANE.load_state == True:
                    lane_msg += ' <span class=success--text><--></span> |\n'
                else:
                    lane_msg += '  <span class=error--text>xx</span>  |\n'
                status_msg += lane_msg
            if CUR_LANE.hub_obj.state == True:
                status_msg += 'HUB: <span class=success--text><-></span>'
            else:
                status_msg += 'HUB: <span class=error--text>x</span>'
            if CUR_LANE.extruder_obj.tool_start_state == True:
                status_msg += '  Tool: <span class=success--text><-></span>'
            else:
                status_msg += '  Tool: <span class=error--text>x</span>'
            if CUR_LANE.extruder_obj.tool_start == 'buffer':
                status_msg += '\n<span class=info--text>Ram sensor enabled</span>'
        self.gcode.respond_raw(status_msg)

    cmd_SET_BOWDEN_LENGTH_help = "Helper to dynamically set length of bowden between hub and toolhead. Pass in HUB if using multiple box turtles"
    def cmd_SET_BOWDEN_LENGTH(self, gcmd):
        """
        This function adjusts the length of the Bowden tube between the hub and the toolhead.
        It retrieves the hub specified by the 'HUB' parameter and the length adjustment specified
        by the 'LENGTH' parameter. If the hub is not specified and a lane is currently loaded,
        it uses the hub of the current lane.

        Usage: `SET_BOWDEN_LENGTH HUB=<hub> LENGTH=<length>`
        Example: `SET_BOWDEN_LENGTH HUB=Turtle_1 LENGTH=100`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameters:
                  - HUB: The name of the hub to be adjusted (optional).
                  - LENGTH: The length adjustment value (optional).

        Returns:
            None
        """
        hub           = gcmd.get("HUB", None )
        length_param  = gcmd.get('LENGTH', None)

        # If hub is not passed in try and get hub if a lane is currently loaded
        if hub is None and self.current is not None:
            CUR_LANE = self.stepper[self.current]
            hub     = CUR_LANE.hub_obj
        elif hub is None and self.current is None:
            self.gcode.respond_info("A lane is not loaded please specify hub to adjust bowden length")
            return

        CUR_HUB       = self.printer.lookup_object('AFC_hub '+ hub )
        config_bowden = CUR_HUB.afc_bowden_length

        if length_param is None or length_param.strip() == '':
            bowden_length = CUR_HUB.config_bowden_length
        else:
            if length_param[0] in ('+', '-'):
                bowden_value = float(length_param)
                bowden_length = config_bowden + bowden_value
            else:
                bowden_length = float(length_param)

        CUR_HUB.afc_bowden_length = bowden_length
        msg =  '// Hub : {}\n'.format( hub )
        msg += '//   Config Bowden Length:   {}\n'.format(CUR_HUB.config_bowden_length)
        msg += '//   Previous Bowden Length: {}\n'.format(config_bowden)
        msg += '//   New Bowden Length:      {}\n'.format(bowden_length)
        msg += '\n// TO SAVE BOWDEN LENGTH afc_bowden_length MUST BE UPDATED IN AFC_Hardware.cfg for each hub if there are multiple'
        self.gcode.respond_raw(msg)

    cmd_LANE_MOVE_help = "Lane Manual Movements"
    def cmd_LANE_MOVE(self, gcmd):
        """
        This function handles the manual movement of a specified lane. It retrieves the lane
        specified by the 'LANE' parameter and moves it by the distance specified by the 'DISTANCE' parameter.

        Usage: `LANE_MOVE LANE=<lane> DISTANCE=<distance>`
        Example: `LANE_MOVE LANE=leg1 DISTANCE=100`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameters:
                  - LANE: The name of the lane to be moved.
                  - DISTANCE: The distance to move the lane.

        NO_DOC: True

        Returns:
            None
        """
        lane = gcmd.get('LANE', None)
        distance = gcmd.get_float('DISTANCE', 0)
        if lane not in self.stepper:
            self.gcode.respond_info('{} Unknown'.format(lane.upper()))
            return
        CUR_LANE = self.stepper[lane]
        CUR_LANE.set_load_current() # Making current is set correctly when doing lane moves
        CUR_LANE.do_enable(True)
        CUR_LANE.move(distance, self.short_moves_speed, self.short_moves_accel, True)
        CUR_LANE.do_enable(False)

    def save_pos(self):
        # Only save previous location on the first toolchange call to keep an error state from overwriting the location
        if self.in_toolchange == False:
            if self.error_state == False:
                self.last_toolhead_position = self.toolhead.get_position()
                self.base_position = self.gcode_move.base_position
                self.last_gcode_position = self.gcode_move.last_position
                self.homing_position = self.gcode_move.homing_position
                self.speed = self.gcode_move.speed
                self.absolute_coord = self.gcode_move.absolute_coord

    def restore_pos(self):
        """
        restore_pos function restores the previous saved position, speed and coord type. The resume uses
        the z_hop value to lift, move to previous x,y coords, then lower to saved z position.
        """
        newpos = self.toolhead.get_position()
        newpos[2] = self.last_gcode_position[2] + self.z_hop

        # Restore absolute coords
        self.gcode_move.absolute_coord = self.absolute_coord

        speed = self.resume_speed if self.resume_speed > 0 else self.speed
        speedz = self.resume_z_speed if self.resume_z_speed > 0 else self.speed
        # Update GCODE STATE variables
        self.gcode_move.base_position = self.base_position
        self.gcode_move.last_position[:3] = self.last_gcode_position[:3]
        self.gcode_move.homing_position = self.homing_position

        # Restore the relative E position
        e_diff = newpos[3] - self.last_gcode_position[3]
        self.gcode_move.base_position[3] += e_diff

        # Move toolhead to previous z location with zhop added
        self.gcode_move.move_with_transform(newpos, speedz)

        # Move to previous x,y location
        newpos[:2] = self.last_gcode_position[:2]
        self.gcode_move.move_with_transform(newpos, speed)

        # Drop to previous z
        newpos[2] = self.last_gcode_position[2]
        self.gcode_move.move_with_transform(newpos, speedz)

    # Helper function to write variables to file. Prints with indents to make it more readable for users
    def save_vars(self):
        """
        save_vars function saves lane variables to var file and prints with indents to
                  make it more readable for users
        """
        with open(self.VarFile+ '.unit', 'w') as f:
            status = self.get_status(0)
            f.write(json.dumps(status, indent=4))
        with open(self.VarFile+ '.tool', 'w') as f:
            f.write(json.dumps(status['system']['extruders'], indent=4))

    cmd_HUB_CUT_TEST_help = "Test the cutting sequence of the hub cutter, expects LANE=legN"
    def cmd_HUB_CUT_TEST(self, gcmd):
        """
        This function tests the cutting sequence of the hub cutter for a specified lane.
        It retrieves the lane specified by the 'LANE' parameter, performs the hub cut,
        and responds with the status of the operation.

        Usage: `HUB_CUT_TEST LANE=<lane>`
        Example: `HUB_CUT_TEST LANE=leg1`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameter:
                  - LANE: The name of the lane to be tested.

        Returns:
            None
        """
        lane = gcmd.get('LANE', None)
        self.gcode.respond_info('Testing Hub Cut on Lane: ' + lane)
        if lane not in self.stepper:
            self.gcode.respond_info('{} Unknown'.format(lane.upper()))
            return
        CUR_LANE = self.stepper[lane]
        CUR_LANE.hub_obj.hub_cut(CUR_LANE)
        self.gcode.respond_info('Hub cut Done!')

    cmd_TEST_help = "Test Assist Motors"
    def cmd_TEST(self, gcmd):
        """
        This function tests the assist motors of a specified lane at various speeds.
        It performs the following steps:
        1. Retrieves the lane specified by the 'LANE' parameter.
        2. Tests the assist motor at full speed, 50%, 30%, and 10% speeds.
        3. Reports the status of each test step.

        Usage: `TEST LANE=<lane>`
        Example: `TEST LANE=leg1`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameter:
                  - LANE: The name of the lane to be tested.

        Returns:
            None
        """
        lane = gcmd.get('LANE', None)
        if lane == None:
            self.ERROR.AFC_error('Must select LANE', False)
            return
        self.gcode.respond_info('TEST ROUTINE')
        if lane not in self.stepper:
            self.gcode.respond_info('{} Unknown'.format(lane.upper()))
            return
        CUR_LANE = self.stepper[lane]
        self.gcode.respond_info('Testing at full speed')
        CUR_LANE.assist(-1)
        self.reactor.pause(self.reactor.monotonic() + 1)
        if CUR_LANE.afc_motor_rwd.is_pwm:
            self.gcode.respond_info('Testing at 50 percent speed')
            CUR_LANE.assist(-.5)
            self.reactor.pause(self.reactor.monotonic() + 1)
            self.gcode.respond_info('Testing at 30 percent speed')
            CUR_LANE.assist(-.3)
            self.reactor.pause(self.reactor.monotonic() + 1)
            self.gcode.respond_info('Testing at 10 percent speed')
            CUR_LANE.assist(-.1)
            self.reactor.pause(self.reactor.monotonic() + 1)
        self.gcode.respond_info('Test routine complete')
        CUR_LANE.assist(0)

    cmd_SPOOL_ID_help = "LINK SPOOL into hub"
    def cmd_SPOOL_ID(self, gcmd):
        return

    # HUB COMMANDS
    cmd_HUB_LOAD_help = "Load lane into hub"
    def cmd_HUB_LOAD(self, gcmd):
        """
        This function handles the loading of a specified lane into the hub. It performs
        several checks and movements to ensure the lane is properly loaded.

        Usage: `HUB_LOAD LANE=<lane>`
        Example: `HUB_LOAD LANE=leg1`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameter:
                  - LANE: The name of the lane to be loaded.

        Returns:
            None
        """
        lane = gcmd.get('LANE', None)
        if lane not in self.stepper:
            self.gcode.respond_info('{} Unknown'.format(lane.upper()))
            return
        CUR_LANE = self.stepper[lane]
        if CUR_LANE.prep_state == False: return

        if CUR_LANE.load_state == False:
            CUR_LANE.do_enable(True)
            while CUR_LANE.load_state == False:
                CUR_LANE.move( CUR_LANE.hub_obj.move_dis, self.short_moves_speed, self.short_moves_accel)
        if CUR_LANE.loaded_to_hub == False:
            CUR_LANE.move(CUR_LANE.dist_hub, CUR_LANE.dist_hub_move_speed, CUR_LANE.dist_hub_move_accel, True if CUR_LANE.dist_hub > 200 else False)
        while CUR_LANE.hub_obj.state == False:
            CUR_LANE.move(CUR_LANE.hub_obj.move_dis, self.short_moves_speed, self.short_moves_accel)
        while CUR_LANE.hub_obj.state == True:
            CUR_LANE.move(CUR_LANE.hub_obj.move_dis * -1, self.short_moves_speed, self.short_moves_accel)
        CUR_LANE.status = ''
        self.save_vars()
        CUR_LANE.do_enable(False)
        CUR_LANE.loaded_to_hub = True
        self.save_vars()

    cmd_LANE_UNLOAD_help = "Unload lane from extruder"
    def cmd_LANE_UNLOAD(self, gcmd):
        """
        This function handles the unloading of a specified lane from the extruder. It performs
        several checks and movements to ensure the lane is properly unloaded.

        Usage: `LANE_UNLOAD LANE=<lane>`
        Example: `LANE_UNLOAD LANE=leg1`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameter:
                  - LANE: The name of the lane to be unloaded.

        Returns:
            None
        """
        lane = gcmd.get('LANE', None)
        if lane not in self.stepper:
            self.gcode.respond_info('{} Unknown'.format(lane.upper()))
            return
        CUR_LANE = self.stepper[lane]
        if CUR_LANE.name != self.current:
            # Setting status as ejecting so if filament is removed and de-activates the prep sensor while
            # extruder motors are still running it does not trigger infinite spool or pause logic
            # once user removes filament lanes status will go to None
            CUR_LANE.status = 'ejecting'
            self.save_vars()
            CUR_LANE.do_enable(True)
            if CUR_LANE.loaded_to_hub:
                CUR_LANE.move(CUR_LANE.dist_hub * -1, CUR_LANE.dist_hub_move_speed, CUR_LANE.dist_hub_move_accel, True if CUR_LANE.dist_hub > 200 else False)
            CUR_LANE.loaded_to_hub = False
            while CUR_LANE.load_state == True:
               CUR_LANE.move( CUR_LANE.hub_obj.move_dis * -1, self.short_moves_speed, self.short_moves_accel, True)
            CUR_LANE.move( CUR_LANE.hub_obj.move_dis * -5, self.short_moves_speed, self.short_moves_accel)
            CUR_LANE.do_enable(False)
            CUR_LANE.status = ''
            self.save_vars()

            # Removing spool from vars since it was ejected
            self.SPOOL.set_spoolID( CUR_LANE, "")
            self.gcode.respond_info("LANE {} eject done".format(CUR_LANE.name))

        else:
            self.gcode.respond_info("LANE {} is loaded in toolhead, can't unload.".format(CUR_LANE.name))

    cmd_TOOL_LOAD_help = "Load lane into tool"
    def cmd_TOOL_LOAD(self, gcmd):
        """
        This function handles the loading of a specified lane into the tool. It retrieves
        the lane specified by the 'LANE' parameter and calls the TOOL_LOAD method to perform
        the loading process.

        Usage: `TOOL_LOAD LANE=<lane>`
        Example: `TOOL_LOAD LANE=leg1`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameter:
                  - LANE: The name of the lane to be loaded.

        Returns:
            None
        """
        lane = gcmd.get('LANE', None)
        if lane not in self.stepper:
            self.gcode.respond_info('{} Unknown'.format(lane.upper()))
            return

        if self.current is not None:
            self.ERROR.AFC_error("Cannot load {}, {} currently loaded".format(lane.upper(), self.current.upper()), pause=False)
            return
        CUR_LANE = self.stepper[lane]
        self.TOOL_LOAD(CUR_LANE)

    def TOOL_LOAD(self, CUR_LANE):
        """
        This function handles the loading of a specified lane into the tool. It performs
        several checks and movements to ensure the lane is properly loaded.

        Usage: `TOOL_LOAD LANE=<lane>`
        Example: `TOOL_LOAD LANE=leg1`

        Args:
            CUR_LANE: The lane object to be loaded into the tool.

        Returns:
            bool: True if load was successful, False if an error occurred.
        """
        if not self.is_homed():
            self.ERROR.AFC_error("Please home printer before doing a tool load", False)
            return False

        if CUR_LANE is None:
            # Exit early if no lane is provided.
            return False

        # Check if the bypass filament sensor is triggered; abort loading if filament is already present.
        try:
            bypass = self.printer.lookup_object('filament_switch_sensor bypass').runout_helper
            if bypass.filament_present:
                self.gcode.respond_info("Filament loaded in bypass, not doing tool load")
                return False
        except:
            bypass = None

        self.gcode.respond_info("Loading {}".format(CUR_LANE.name))

        # Prepare extruder and heater.
        extruder = self.toolhead.get_extruder()
        self.heater = extruder.get_heater()

        # Set the lane status to 'loading' and activate the loading LED.
        CUR_LANE.status = 'loading'
        self.save_vars()
        self.afc_led(self.led_loading, CUR_LANE.led_index)

        # Check if the lane is in a state ready to load and hub is clear.
        if CUR_LANE.load_state and not CUR_LANE.hub_obj.state:
            # Heat the extruder if it is below the minimum extrusion temperature.
            if not self.heater.can_extrude:
                pheaters = self.printer.lookup_object('heaters')
                if self.heater.target_temp <= self.heater.min_extrude_temp:
                    self.gcode.respond_info('Extruder below min_extrude_temp, heating to 5 degrees above min.')
                    pheaters.set_temperature(extruder.get_heater(), self.heater.min_extrude_temp + 5, wait=True)

            # Enable the lane for filament movement.
            CUR_LANE.do_enable(True)

            # Move filament to the hub if it's not already loaded there.
            if not CUR_LANE.loaded_to_hub:
                CUR_LANE.move(CUR_LANE.dist_hub, CUR_LANE.dist_hub_move_speed, CUR_LANE.dist_hub_move_accel, CUR_LANE.dist_hub > 200)

            CUR_LANE.loaded_to_hub = True
            hub_attempts = 0

            # Ensure filament moves past the hub.
            while not CUR_LANE.hub_obj.state:
                if hub_attempts == 0:
                    CUR_LANE.move(CUR_LANE.hub_obj.move_dis, self.short_moves_speed, self.short_moves_accel)
                else:
                    CUR_LANE.move(self.short_move_dis, self.short_moves_speed, self.short_moves_accel)
                hub_attempts += 1
                if hub_attempts > 20:
                    message = ('PAST HUB, CHECK FILAMENT PATH\n||=====||==>--||-----||\nTRG   LOAD   HUB   TOOL')
                    self.ERROR.handle_lane_failure(CUR_LANE, message)
                    return False

            # Move filament towards the toolhead.
            CUR_LANE.move(CUR_LANE.hub_obj.afc_bowden_length, self.long_moves_speed, self.long_moves_accel, True)

            # Ensure filament reaches the toolhead.
            tool_attempts = 0
            if CUR_LANE.extruder_obj.tool_start:
                while not CUR_LANE.extruder_obj.tool_start_state:
                    tool_attempts += 1
                    CUR_LANE.move(self.short_move_dis, CUR_LANE.extruder_obj.tool_load_speed, self.long_moves_accel)
                    if tool_attempts > 20:
                        message = ('FAILED TO LOAD TO TOOL, CHECK FILAMENT PATH\n||=====||====||==>--||\nTRG   LOAD   HUB   TOOL')
                        self.ERROR.handle_lane_failure(CUR_LANE, message)
                        return False

            # Synchronize lane's extruder stepper and finalize tool loading.
            CUR_LANE.status = 'Tooled'
            self.save_vars()
            CUR_LANE.sync_to_extruder()

            # Adjust tool position for loading.
            pos = self.toolhead.get_position()
            pos[3] += CUR_LANE.extruder_obj.tool_stn
            self.toolhead.manual_move(pos, CUR_LANE.extruder_obj.tool_load_speed)
            self.toolhead.wait_moves()

            # Check if ramming is enabled, if it is go through ram load sequence.
            # Lane will load until Advance sensor is True
            # After the tool_stn distance the lane will retract off the sensor to confirm load and reset buffer
            if CUR_LANE.extruder_obj.tool_start == "buffer":
                CUR_LANE.unsync_to_extruder()
                load_checks = 0
                while CUR_LANE.extruder_obj.tool_start_state == True:
                    CUR_LANE.move( self.short_move_dis * -1, self.short_moves_speed, self.short_moves_accel )
                    load_checks += 1
                    self.reactor.pause(self.reactor.monotonic() + 0.1)
                    if load_checks > self.tool_max_load_checks:
                        msg = ''
                        msg += "Buffer did not become compressed after {} short moves.\n".format(self.tool_max_load_checks)
                        msg += "Tool may not be loaded"
                        self.gcode.respond_info("<span class=warning--text>{}</span>".format(msg))
                        break
                CUR_LANE.sync_to_extruder()
            # Update tool and lane status.
            CUR_LANE.status = 'Tooled'
            CUR_LANE.tool_loaded = True
            self.current = CUR_LANE.name
            CUR_LANE.extruder_obj.enable_buffer()

            # Activate the tool-loaded LED and handle filament operations if enabled.
            self.afc_led(self.led_tool_loaded, CUR_LANE.led_index)
            if self.poop:
                self.gcode.run_script_from_command(self.poop_cmd)
                if self.wipe:
                    self.gcode.run_script_from_command(self.wipe_cmd)
            if self.kick:
                self.gcode.run_script_from_command(self.kick_cmd)
            if self.wipe:
                self.gcode.run_script_from_command(self.wipe_cmd)

            # Update lane and extruder state for tracking.
            CUR_LANE.extruder_obj.lane_loaded = CUR_LANE.name
            self.SPOOL.set_active_spool(CUR_LANE.spool_id)
            self.afc_led(self.led_tool_loaded, CUR_LANE.led_index)
            self.save_vars()
        else:
            # Handle errors if the hub is not clear or the lane is not ready for loading.
            if CUR_LANE.hub_obj.state:
                message = ('HUB NOT CLEAR WHEN TRYING TO LOAD\n||-----||----|x|-----||\nTRG   LOAD   HUB   TOOL')
                self.ERROR.handle_lane_failure(CUR_LANE, message)
                return False
            if not CUR_LANE.load_state:
                message = ('NOT READY, LOAD TRIGGER NOT TRIGGERED\n||==>--||----||-----||\nTRG   LOAD   HUB   TOOL')
                self.ERROR.handle_lane_failure(CUR_LANE, message)
                return False

        return True

    cmd_TOOL_UNLOAD_help = "Unload from tool head"
    def cmd_TOOL_UNLOAD(self, gcmd):
        """
        This function handles the unloading of a specified lane from the tool head. It retrieves
        the lane specified by the 'LANE' parameter or uses the currently loaded lane if no parameter
        is provided, and calls the TOOL_UNLOAD method to perform the unloading process.

        Usage: `TOOL_UNLOAD [LANE=<lane>]`
        Example: `TOOL_UNLOAD LANE=leg1`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameter:
                  - LANE: The name of the lane to be unloaded (optional, defaults to the current lane).

        Returns:
            None
        """
        lane = gcmd.get('LANE', self.current)
        if lane == None:
            return
        if lane not in self.stepper:
            self.gcode.respond_info('{} Unknown'.format(lane.upper()))
            return
        CUR_LANE = self.stepper[lane]
        self.TOOL_UNLOAD(CUR_LANE)

        # User manually unloaded spool from toolhead, remove spool from active status
        self.SPOOL.set_active_spool( None )

    def TOOL_UNLOAD(self, CUR_LANE):
        """
        This function handles the unloading of a specified lane from the tool. It performs
        several checks and movements to ensure the lane is properly unloaded.

        Usage: `TOOL_UNLOAD LANE=<lane>`
        Example: `TOOL_UNLOAD LANE=leg1`
        Args:
            CUR_LANE: The lane object to be unloaded from the tool.

        Returns:
            bool: True if unloading was successful, False if an error occurred.
        """
        if not self.is_homed():
            self.ERROR.AFC_error("Please home printer before doing a tool unload", False)
            return False

        if CUR_LANE is None:
            # If no lane is provided, exit the function early with a failure.
            return False

        self.gcode.respond_info("Unloading {}".format(CUR_LANE.name))

        # TODO: Need to move setting temperature here

        # Quick pull to prevent oozing.
        pos = self.toolhead.get_position()
        pos[3] -= 2
        self.toolhead.manual_move(pos, CUR_LANE.extruder_obj.tool_unload_speed)
        self.toolhead.wait_moves()

        # Perform Z-hop to avoid collisions during unloading.
        pos[2] += self.z_hop
        self.toolhead.manual_move(pos, CUR_LANE.extruder_obj.tool_unload_speed)
        self.toolhead.wait_moves()

        # Prepare the extruder and heater for unloading.
        extruder = self.toolhead.get_extruder()
        self.heater = extruder.get_heater()
        CUR_LANE.status = 'unloading'
        self.save_vars()
        # Disable the buffer if it's active.
        CUR_LANE.extruder_obj.disable_buffer()

        # Activate LED indicator for unloading.
        self.afc_led(self.led_unloading, CUR_LANE.led_index)

        if CUR_LANE.extruder_stepper.motion_queue != CUR_LANE.extruder_name:
            # Synchronize the extruder stepper with the lane.
            CUR_LANE.sync_to_extruder()

        # Check and set the extruder temperature if below the minimum.
        wait = True
        pheaters = self.printer.lookup_object('heaters')
        if self.heater.target_temp <= self.heater.min_extrude_temp:
            self.gcode.respond_info('Extruder below min_extrude_temp, heating to 5 degrees above min.')
            pheaters.set_temperature(extruder.get_heater(), self.heater.min_extrude_temp + 5, wait)

        # Enable the lane for unloading operations.
        CUR_LANE.do_enable(True)

        # Perform filament cutting and parking if specified.
        if self.tool_cut:
            self.gcode.run_script_from_command(self.tool_cut_cmd)
            if self.park:
                self.gcode.run_script_from_command(self.park_cmd)

        # Form filament tip if necessary.
        if self.form_tip:
            if self.park:
                self.gcode.run_script_from_command(self.park_cmd)
            if self.form_tip_cmd == "AFC":
                self.AFC_tip = self.printer.lookup_object('AFC_form_tip')
                self.AFC_tip.tip_form()
            else:
                self.gcode.run_script_from_command(self.form_tip_cmd)

        # Attempt to unload the filament from the extruder, retrying if needed.
        num_tries = 0
        if CUR_LANE.extruder_obj.tool_start == "buffer":
            # if ramming is enabled, AFC will retract to collapse buffer before unloading
            CUR_LANE.unsync_to_extruder()
            while CUR_LANE.extruder_obj.buffer_trailing == False:
                # attempt to return buffer to trailng pin
                CUR_LANE.move( self.short_move_dis * -1, self.short_moves_speed, self.short_moves_accel )
                num_tries += 1
                self.reactor.pause(self.reactor.monotonic() + 0.1)
                if num_tries > self.tool_max_unload_attempts:
                    msg = ''
                    msg += "Buffer did not become compressed after {} short moves.\n".format(self.tool_max_unload_attempts)
                    msg += "Increasing 'tool_max_unload_attempts' may improve loading reliablity"
                    self.gcode.respond_info("<span class=warning--text>{}</span>".format(msg))
                    break
            CUR_LANE.sync_to_extruder(False)
            pos = self.toolhead.get_position()
            pos[3] -= CUR_LANE.extruder_obj.tool_stn_unload
            self.toolhead.manual_move(pos, CUR_LANE.extruder_obj.tool_unload_speed)
            self.toolhead.wait_moves()
        else:
            while CUR_LANE.extruder_obj.tool_start_state:
                num_tries += 1
                if num_tries > self.tool_max_unload_attempts:
                    # Handle failure if the filament cannot be unloaded.
                    message = ('FAILED TO UNLOAD. FILAMENT STUCK IN TOOLHEAD.')
                    self.ERROR.handle_lane_failure(CUR_LANE, message)
                    return False
                CUR_LANE.sync_to_extruder()
                pos = self.toolhead.get_position()
                pos[3] -= CUR_LANE.extruder_obj.tool_stn_unload
                self.toolhead.manual_move(pos, CUR_LANE.extruder_obj.tool_unload_speed)
                self.toolhead.wait_moves()

        # Move filament past the sensor after the extruder, if applicable.
        if CUR_LANE.extruder_obj.tool_sensor_after_extruder > 0:
            pos = self.toolhead.get_position()
            pos[3] -= CUR_LANE.extruder_obj.tool_sensor_after_extruder
            self.toolhead.manual_move(pos, CUR_LANE.extruder_obj.tool_unload_speed)
            self.toolhead.wait_moves()

        # Synchronize and move filament out of the hub.
        CUR_LANE.unsync_to_extruder()
        CUR_LANE.move(CUR_LANE.hub_obj.afc_bowden_length * -1, self.long_moves_speed, self.long_moves_accel, True)

        # Clear toolhead's loaded state for easier error handling later.
        CUR_LANE.tool_loaded = False
        CUR_LANE.extruder_obj.lane_loaded = ''
        CUR_LANE.status = None
        self.current = None

        self.save_vars()

        # Ensure filament is fully cleared from the hub.
        num_tries = 0
        while CUR_LANE.hub_obj.state:
            CUR_LANE.move(self.short_move_dis * -1, self.short_moves_speed, self.short_moves_accel, True)
            num_tries += 1
            if num_tries > (CUR_LANE.hub_obj.afc_bowden_length / self.short_move_dis):
                # Handle failure if the filament doesn't clear the hub.
                message = 'HUB NOT CLEARING\n'
                self.ERROR.handle_lane_failure(CUR_LANE, message)
                return False

        #Move to make sure hub path is clear based on the move_clear_dis var
        CUR_LANE.move( CUR_LANE.hub_obj.hub_clear_move_dis * -1, self.short_moves_speed, self.short_moves_accel, True)

        # Cut filament at the hub, if configured.
        if CUR_LANE.hub_obj.cut:
            if CUR_LANE.hub_obj.cut_cmd == 'AFC':
                CUR_LANE.hub_obj.hub_cut(CUR_LANE)
            else:
                self.gcode.run_script_from_command(CUR_LANE.hub_obj.cut_cmd)

            # Confirm the hub is clear after the cut.
            while CUR_LANE.hub_obj.state:
                CUR_LANE.move(self.short_move_dis * -1, self.short_moves_speed, self.short_moves_accel, True)
                num_tries += 1
                # TODO: Figure out max number of tries
                if num_tries > (CUR_LANE.hub_obj.afc_bowden_length / self.short_move_dis):
                    message = 'HUB NOT CLEARING after hub cut\n'
                    self.ERROR.handle_lane_failure(CUR_LANE, message)
                    return False

        # Finalize unloading and reset lane state.
        CUR_LANE.loaded_to_hub = True
        self.afc_led(self.led_ready, CUR_LANE.led_index)
        CUR_LANE.status = None
        self.save_vars()
        self.current = None
        CUR_LANE.do_enable(False)
        self.gcode.respond_info("LANE {} unload done".format(CUR_LANE.name))

        return True

    cmd_CHANGE_TOOL_help = "change filaments in tool head"
    def cmd_CHANGE_TOOL(self, gcmd):
        """
        This function handles the tool change process. It retrieves the lane specified by the 'LANE' parameter,
        checks the filament sensor, saves the current position, and performs the tool change by unloading the
        current lane and loading the new lane.

        Usage: `CHANGE_TOOL LANE=<lane>`
        Example: `CHANGE_TOOL LANE=leg1`

        Args:
            gcmd: The G-code command object containing the parameters for the command.
                  Expected parameter:
                  - LANE: The name of the lane to be loaded.

        Returns:
            None
        """
        if not self.is_homed():
            self.ERROR.AFC_error("Please home printer before doing a tool change", False)
            return

        tmp = gcmd.get_commandline().split()[0]
        cmd = tmp.upper()
        Tcmd = ''
        if 'LANE' in cmd:
            lane = gcmd.get('LANE', None)
            for key in self.tool_cmds.keys():
                if self.tool_cmds[key].upper() == lane.upper():
                    Tcmd = key
                    break
        else:
            Tcmd = cmd

        if Tcmd == '':
            self.gcode.respond_info("I did not understand the change -- " +cmd)
            return

        lane=self.tool_cmds[Tcmd]
        # Check if the bypass filament sensor detects filament; if so, abort the tool change.
        try:
            bypass = self.printer.lookup_object('filament_switch_sensor bypass').runout_helper
            if bypass.filament_present:
                self.gcode.respond_info("Filament loaded in bypass, not doing toolchange")
                return
        except:
            bypass = None

        # If the requested lane is not the current lane, proceed with the tool change.
        if lane != self.current:
            # Save the current toolhead position to allow restoration after the tool change.
            self.save_pos()

            # Set the in_toolchange flag to prevent overwriting the saved position during potential failures.
            self.in_toolchange = True

            # Lookup the lane object for the requested lane.
            if lane not in self.stepper:
                self.gcode.respond_info('{} Unknown'.format(lane.upper()))
                return
            CUR_LANE = self.stepper[lane]
            # Check if the lane has completed the preparation process required for tool changes.
            if CUR_LANE._afc_prep_done:
                
                # Log the tool change operation for debugging or informational purposes.
                self.gcode.respond_info("Tool Change - {} -> {}".format(self.current, lane))
                if not self.error_state and self.number_of_toolchanges != 0 and self.current_toolchange != self.number_of_toolchanges:
                    self.current_toolchange += 1
                    self.gcode.respond_raw("//      Change {} out of {}".format(self.current_toolchange, self.number_of_toolchanges))

                # If a current lane is loaded, unload it first.
                if self.current is not None:
                    if self.current not in self.stepper:
                        self.gcode.respond_info('{} Unknown'.format(self.current.upper()))
                        return
                    CUR_LANE = self.stepper[self.current]
                    if not self.TOOL_UNLOAD(CUR_LANE):
                        # Abort if the unloading process fails.
                        msg = (' UNLOAD ERROR NOT CLEARED')
                        self.ERROR.fix(msg, CUR_LANE)  #send to error handling
                        return

                # Switch to the new lane for loading.
                if lane not in self.stepper:
                    self.gcode.respond_info('{} Unknown'.format(lane.upper()))
                    return
                CUR_LANE = self.stepper[lane]
            # Load the new lane and restore the toolhead position if successful.
            if self.TOOL_LOAD(CUR_LANE) and not self.error_state:
                self.gcode.respond_info("{} is now loaded in toolhead".format(lane))
                self.restore_pos()
                self.in_toolchange = False
        else:
            self.gcode.respond_info("{} already loaded".format(lane))
            if not self.error_state and self.number_of_toolchanges != 0 and self.current_toolchange != self.number_of_toolchanges: 
                self.current_toolchange += 1

    def get_filament_status(self, LANE):
        if LANE.prep_state:
            if LANE.load_state:
                if LANE.extruder_obj is not None and LANE.extruder_obj.lane_loaded == LANE.name:
                    return 'In Tool:' + HexConvert(self.led_tool_loaded)
                return "Ready:" + HexConvert(self.led_ready)
            return 'Prep:' + HexConvert(self.led_prep_loaded)
        return 'Not Ready:' + HexConvert(self.led_not_ready)

    def get_status(self, eventtime):
        str = {}
        numoflanes = 0
        for UNIT in self.units.keys():
            try:
                screen_mac = self.printer.lookup_object('AFC_screen ' + UNIT).mac
            except error:
                screen_mac = 'None'
            str[UNIT]={}
            for NAME, CUR_LANE in self.units[UNIT].lanes.items():
                str[UNIT][NAME]={}
                str[UNIT][NAME]['LANE'] = CUR_LANE.index
                str[UNIT][NAME]['map'] = CUR_LANE.map
                str[UNIT][NAME]['load'] = bool(CUR_LANE.load_state)
                str[UNIT][NAME]["prep"] =bool(CUR_LANE.prep_state)
                str[UNIT][NAME]["tool_loaded"] = CUR_LANE.tool_loaded
                str[UNIT][NAME]["loaded_to_hub"] = CUR_LANE.loaded_to_hub
                str[UNIT][NAME]["material"]=CUR_LANE.material
                str[UNIT][NAME]["spool_id"]=CUR_LANE.spool_id
                str[UNIT][NAME]["color"]=CUR_LANE.color
                str[UNIT][NAME]["weight"]=CUR_LANE.weight
                str[UNIT][NAME]["runout_lane"]=CUR_LANE.runout_lane
                filiment_stat=self.get_filament_status(CUR_LANE).split(':')
                str[UNIT][NAME]['filament_status']=filiment_stat[0]
                str[UNIT][NAME]['filament_status_led']=filiment_stat[1]
                str[UNIT][NAME]['status'] = CUR_LANE.status if CUR_LANE.status is not None else ''
                numoflanes +=1
            str[UNIT]['system']={}
            str[UNIT]['system']['type'] = self.printer.lookup_object('AFC_hub '+ UNIT).unit.name
            str[UNIT]['system']['hub_loaded']  = True == self.printer.lookup_object('AFC_hub '+ UNIT).state
            str[UNIT]['system']['can_cut']  = True == self.printer.lookup_object('AFC_hub '+ UNIT).cut
            str[UNIT]['system']['screen'] = screen_mac

        str["system"]={}
        str["system"]['current_load']= self.current
        str["system"]['num_units'] = len(self.units)
        str["system"]['num_lanes'] = numoflanes
        str["system"]['num_extruders'] = len(self.extruders)
        str["system"]["extruders"]={}

        for key, CUR_EXTRUDER in self.extruders.items():
            str["system"]["extruders"][key]={}
            # CUR_EXTRUDER = self.printer.lookup_object('AFC_extruder ' + EXTRUDE)
            str["system"]["extruders"][key]['lane_loaded'] = CUR_EXTRUDER.lane_loaded
            if CUR_EXTRUDER.tool_start == "buffer":
                if CUR_EXTRUDER.lane_loaded == '':
                    str ["system"]["extruders"][key]['tool_start_sensor'] = False
                else:
                    str["system"]["extruders"][key]['tool_start_sensor'] = True
            else:
                str["system"]["extruders"][key]['tool_start_sensor'] = True == CUR_EXTRUDER.tool_start_state if CUR_EXTRUDER.tool_start is not None else False
            if CUR_EXTRUDER.tool_end is not None:
                str["system"]["extruders"][key]['tool_end_sensor']   = True == CUR_EXTRUDER.tool_end_state
            else:
                str["system"]["extruders"][key]['tool_end_sensor']   = None
            str["system"]["extruders"][key]['buffer']   = CUR_EXTRUDER.buffer_name
            str["system"]["extruders"][key]['buffer_status']   = CUR_EXTRUDER.buffer_status()
        return str

    def is_homed(self):
        curtime = self.reactor.monotonic()
        kin_status = self.toolhead.get_kinematics().get_status(curtime)
        if ('x' not in kin_status['homed_axes'] or 'y' not in kin_status['homed_axes'] or 'z' not in kin_status['homed_axes']):
            return False
        else:
            return True

    def is_printing(self):
        eventtime = self.reactor.monotonic()
        idle_timeout = self.printer.lookup_object("idle_timeout")
        if idle_timeout.get_status(eventtime)["state"] == "Printing":
            return True
        else:
            False

    def is_paused(self):
        eventtime = self.reactor.monotonic()
        pause_resume = self.printer.lookup_object("pause_resume")
        return bool(pause_resume.get_status(eventtime)["is_paused"])

    def afc_led (self, status, idx=None):
        if idx == None:
            return
        # Try to find led object, if not found print error to console for user to see
        afc_object = 'AFC_led '+ idx.split(':')[0]
        try: led = self.printer.lookup_object(afc_object)
        except:
            error_string = "Error: Cannot find [{}] in config, make sure led_index in config is correct for AFC_stepper {}".format(afc_object, idx.split(':')[-1])
            self.gcode.respond_info( error_string)
        led.led_change(int(idx.split(':')[1]), status)

    def TcmdAssign(self, CUR_LANE):
        if CUR_LANE.map == 'NONE' :
            for x in range(99):
                cmd = 'T'+str(x)
                if cmd not in self.tool_cmds:
                    CUR_LANE.map = cmd
                    break
        self.tool_cmds[CUR_LANE.map]=CUR_LANE.name
        try:
            self.gcode.register_command(CUR_LANE.map, self.cmd_CHANGE_TOOL, desc=self.cmd_CHANGE_TOOL_help)
        except:
            self.gcode.respond_info("Error trying to map lane {lane} to {tool_macro}, please make sure there are no macros already setup for {tool_macro}".format(lane=[CUR_LANE.name], tool_macro=CUR_LANE.map), )
        self.save_vars()

def HexConvert(tmp):
    def convert_led(value):
        if float(value)>0:
            return int(255*float(value))
        else: return 0
    
    led = []
    for l in tmp.split(','):
        led.append(convert_led(l))

    return '#{:02x}{:02x}{:02x}'.format(*led)

def add_filament_switch( switch_name, switch_pin, printer ):
    """
    Helper function to register pins as filament switch sensor so it will show up in web guis

    :param switch_name: Name of switch to register, should be in the following format: `filament_switch_sensor <name>`
    :param switch_pin: Pin to add to config for switch
    :param printer: printer object

    :return returns filament_switch_sensor object
    """
    import configparser
    import configfile
    ppins = printer.lookup_object('pins')
    ppins.allow_multi_use_pin(switch_pin.strip("!^"))
    filament_switch_config = configparser.RawConfigParser()
    filament_switch_config.add_section( switch_name )
    filament_switch_config.set( switch_name, 'switch_pin', switch_pin)

    cfg_wrap = configfile.ConfigWrapper( printer, filament_switch_config, {}, switch_name)

    fila = printer.load_object(cfg_wrap, switch_name)
    fila.runout_helper.sensor_enabled = False
    fila.runout_helper.runout_pause = False

    return fila

def load_config(config):
    return afc(config)
