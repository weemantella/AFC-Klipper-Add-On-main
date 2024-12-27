# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

import os
import json

class afcPrep:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.handle_connect)
        self.delay = config.getfloat('delay_time', 0.1, minval=0.0)                 # Time to delay when moving extruders and spoolers during PREP routine
        self.enable = config.getboolean("enable", False)                            # Set True to disable PREP checks

        # Flag to set once resume rename as occurred for the first time
        self.rename_occurred = False
        # Value gets set to false once prep has been ran for the first time after restarting klipper
        self.assignTcmd = True

    def handle_connect(self):
        """
        Handle the connection event.
        This function is called when the printer connects. It looks up AFC info
        and assigns it to the instance variable `self.AFC`.
        """
        self.AFC = self.printer.lookup_object('AFC')
        self.AFC.gcode.register_command('PREP', self.PREP, desc=None)

    def _rename_resume(self):
        """
            Helper function to check if renaming RESUME macro has occured and renames RESUME.
            Addes a new RESUME macro that points to AFC resume function
        """

        # Checking to see if rename has already been done, don't want to rename again if prep was already ran
        if not self.rename_occurred:
            self.rename_occurred = True
            # Renaming users Resume macro so that RESUME calls AFC_Resume function instead
            base_resume_name = "RESUME"
            prev_cmd = self.AFC.gcode.register_command(base_resume_name, None)
            if prev_cmd is not None:
                pdesc = "Renamed builtin of '%s'" % (base_resume_name,)
                self.AFC.gcode.register_command(self.AFC.ERROR.AFC_RENAME_RESUME_NAME, prev_cmd, desc=pdesc)
            else:
                self.AFC.gcode.respond_info("{}Existing command {} not found in gcode_macros{}".format("<span class=warning--text>", base_resume_name, "</span>",))

            self.AFC.gcode.register_command(base_resume_name, self.AFC.ERROR.cmd_AFC_RESUME, desc=self.AFC.ERROR.cmd_AFC_RESUME_help)

    def PREP(self, gcmd):
        while self.printer.state_message != 'Printer is ready':
            self.AFC.reactor.pause(self.AFC.reactor.monotonic() + 1)

        self._rename_resume()
        self.AFC.print_version()

        self.AFC.tool_cmds={}

        if self.enable == False:
            self.AFC.gcode.respond_info('Prep Checks Disabled')
            return
        elif len(self.AFC.units) >0:
            for keys, unit in self.AFC.units.items():
                logo=''
                logo_error = ''
                self.AFC.gcode.respond_info('{} {} Prepping lanes'.format(unit.full_name[0], unit.name))

                logo=unit.logo
                logo+='  ' + unit.name + '\n'
                logo_error=unit.logo_error
                logo_error+='  ' + unit.name + '\n'

                LaneCheck = True
                for LANE in unit.lanes.values():
                    if not unit.system_Test(LANE, self.delay, self.assignTcmd):
                        LaneCheck = False

                if LaneCheck:
                    self.AFC.gcode.respond_raw(logo)
                else:
                    self.AFC.gcode.respond_raw(logo_error)
            try:
                bypass = self.printer.lookup_object('filament_switch_sensor bypass').runout_helper
                if bypass.filament_present == True:
                    self.AFC.gcode.respond_info("Filament loaded in bypass, not doing toolchange")
            except: bypass = None

            for key, CUR_EXTRUDER in self.AFC.extruders.items():
                # CUR_EXTRUDER = self.printer.lookup_object('AFC_extruder ' + EXTRUDE)
                if CUR_EXTRUDER.tool_start_state == True and bypass != True:
                    if not CUR_EXTRUDER.lane_loaded:
                        self.AFC.gcode.respond_info("<span class=error--text>{} loaded with out identifying lane in AFC.vars.tool file<span>".format(EXTRUDE))

        # Defaulting to no active spool, putting at end so endpoint has time to register
        if self.AFC.current is None:
            self.AFC.SPOOL.set_active_spool( None )

        # Setting value to False so the T commands do try to get reassigned when users manually
        #   run PREP after it has already be ran once upon boot
        self.assignTcmd = False

        self.AFC.save_vars()

def load_config(config):
    return afcPrep(config)

