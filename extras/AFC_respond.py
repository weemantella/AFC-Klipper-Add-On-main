# Armored Turtle Automated Filament Changer
#
# Copyright (C) 2024 Armored Turtle
#
# This file may be distributed under the terms of the GNU GPLv3 license.

class afcrespond:
    def __init__(self, config):
        self.printer = config.get_printer()
        self.gcode   = self.printer.lookup_object('gcode')

    def afc_msg_warning(self, msg):
        return '<span class=warning--text>{}</span>'.format(msg)

    def afc_msg_info(self, msg):
        pass

    def afc_msg_error(self, msg):
        pass


def load_config(config):
    return afcrespond(config)


