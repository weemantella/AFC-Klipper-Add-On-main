# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.
from extras.AFC import add_filament_switch

class AFCextruder:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.printer.register_event_handler("klippy:connect", self.handle_connect)

        self.AFC = self.printer.lookup_object("AFC")

        self.name = config.get_name().split(' ')[-1]
        self.tool_stn = config.getfloat("tool_stn", 72)
        self.tool_stn_unload = config.getfloat("tool_stn_unload", 100)
        self.tool_sensor_after_extruder = config.getfloat("tool_sensor_after_extruder", 0)
        self.tool_unload_speed = config.getfloat("tool_unload_speed", 25)
        self.tool_load_speed = config.getfloat("tool_load_speed", 25)
        ppins = self.printer.lookup_object('pins')
        self.gcode = self.printer.lookup_object('gcode')

        buttons = self.printer.load_object(config, "buttons")
        self.tool_start = config.get('pin_tool_start', None)
        self.tool_end = config.get('pin_tool_end', None)

        self.lane_loaded = ''
        self.enable_sensors_in_gui = config.getboolean("enable_sensors_in_gui", self.AFC.enable_sensors_in_gui)

        if self.tool_start is not None:
            if self.tool_start == "buffer":
                self.gcode.respond_info("Setting up as buffer")
            else:
                self.tool_start_state = False
                buttons.register_buttons([self.tool_start], self.tool_start_callback)
                if self.enable_sensors_in_gui:
                    self.tool_start_filament_switch_name = "filament_switch_sensor {}".format("tool_start")
                    self.fila_tool_start = add_filament_switch(self.tool_start_filament_switch_name, self.tool_start, self.printer )
        if self.tool_end is not None:
            self.tool_end_state = False
            buttons.register_buttons([self.tool_end], self.tool_end_callback)
            if self.enable_sensors_in_gui:
                self.tool_end_state_filament_switch_name = "filament_switch_sensor {}".format("tool_end")
                self.fila_avd = add_filament_switch(self.tool_end_state_filament_switch_name, self.tool_end, self.printer )

    def handle_connect(self):
        """
        Handle the connection event.
        This function is called when the printer connects. It looks up AFC info
        and assigns it to the instance variable `self.AFC`.
        """
        self.AFC = self.printer.lookup_object('AFC')
        self.reactor = self.AFC.reactor

        # Restores prev state
        if self.name in self.AFC.extruders_f:
            self.lane_loaded = self.AFC.extruders_f[self.name]['lane_loaded']
        self.AFC.extruders[self.name] = self

    def tool_start_callback(self, eventtime, state):
        self.tool_start_state = state

    def tool_end_callback(self, eventtime, state):
        self.tool_end_state = state

def load_config_prefix(config):
    return AFCextruder(config)
