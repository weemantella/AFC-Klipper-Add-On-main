from configfile import error
class afcUnit:
    def __init__(self, config):
        self.printer    = config.get_printer()
        self.gcode      = self.printer.lookup_object('gcode')
        
        self.full_name  = config.get_name().split()
        self.name       = self.full_name[-1]
        self.hub        = config.getlists("hub", None)
        self.extruder   = config.get("extruder")
        self.buffer_name    = config.get('buffer', None)

        self.lanes      = {}

        # Objects
        self.buffer_obj = None

        self.printer.register_event_handler("klippy:connect", self.handle_connect)

    def handle_connect(self):
        self.AFC = self.printer.lookup_object('AFC')
        self.AFC.units[self.name] = self

        self.hub_array = {}
        if self.hub is not None:
            for hub in self.hub:
                h = self.printer.lookup_object("AFC_hub {}".format(hub))
                if h.unit is not None:
                    raise error("AFC_hub {} already has a unit {} assigned, can't assign {}. Only one unit can be assigned per AFC_hub".format(hub, " ".join(h.unit.full_name), " ".join(self.full_name)))
                else:
                    # TODO: Not sure what I was thinking for this array stuff, maybe just make it a object and can override at stepper level
                    # Maybe this was done to force the user to input all the hubs that will be in this unit?
                    self.hub_array[hub] = h
                    self.hub_array[hub].unit = self
        else:
            self.hub_array['hub'] = None
            # self.hub_array[].unit = None

        try:
            self.extruder_obj = self.printer.lookup_object("AFC_extruder {}".format(self.extruder))
        except:
            error_string = 'Error: No config found for extruder: {extruder} in [AFC_unit {unit}]. Please make sure [AFC_extruder {extruder}] section exists in your config'.format(
                            extruder=self.extruder, unit=self.name )
            raise error( error_string )

        if self.buffer_name is not None:
            self.buffer_obj = self.printer.lookup_object('AFC_buffer {}'.format(self.buffer_name))

        self.AFC.gcode.respond_info("AFC_{}:ready".format(self.name))

        # Send out event so lanes can store units object
        self.printer.send_event("{}:connect".format(self.name), self)

    def system_Test(self, CUR_LANE, delay, assignTcmd, enable_movement):
        msg = ''
        succeeded = True
        
        # Run test reverse/forward on each lane
        CUR_LANE.unsync_to_extruder(False)
        if enable_movement:
            CUR_LANE.move( 5, self.AFC.short_moves_speed, self.AFC.short_moves_accel, True)
            self.AFC.reactor.pause(self.AFC.reactor.monotonic() + delay)
            CUR_LANE.move( -5, self.AFC.short_moves_speed, self.AFC.short_moves_accel, True)
        else:
            self.AFC.reactor.pause(self.AFC.reactor.monotonic() + delay)

        if CUR_LANE.prep_state == False:
            if CUR_LANE.load_state == False:
                self.AFC.afc_led(self.AFC.led_not_ready, CUR_LANE.led_index)
                msg += 'EMPTY READY FOR SPOOL'
            else:
                self.AFC.afc_led(self.AFC.led_fault, CUR_LANE.led_index)
                msg +="<span class=error--text> NOT READY</span>"
                CUR_LANE.do_enable(False)
                msg = '<span class=error--text>CHECK FILAMENT Prep: False - Load: True</span>'
                succeeded = False

        else:
            self.AFC.afc_led(self.AFC.led_ready, CUR_LANE.led_index)
            msg +="<span class=success--text>LOCKED</span>"
            if CUR_LANE.load_state == False:
                msg +="<span class=error--text> NOT LOADED</span>"
                self.AFC.afc_led(self.AFC.led_not_ready, CUR_LANE.led_index)
                succeeded = False
            else:
                CUR_LANE.status = 'Loaded'
                msg +="<span class=success--text> AND LOADED</span>"

                if CUR_LANE.tool_loaded:
                    if CUR_LANE.get_toolhead_sensor_state() == True or CUR_LANE.extruder_obj.tool_start == "buffer":
                        if CUR_LANE.extruder_obj.lane_loaded == CUR_LANE.name:
                            CUR_LANE.sync_to_extruder()
                            msg +="<span class=primary--text> in ToolHead</span>"
                            if CUR_LANE.extruder_obj.tool_start == "buffer":
                                msg += "<span class=warning--text>\n Ram sensor enabled, confirm tool is loaded</span>"
                            self.AFC.SPOOL.set_active_spool(CUR_LANE.spool_id)
                            self.AFC.afc_led(self.AFC.led_tool_loaded, CUR_LANE.led_index)
                            CUR_LANE.status = 'Tooled'
                            if len(self.AFC.extruders) == 1:
                                self.AFC.current = CUR_LANE.name
                                CUR_LANE.enable_buffer()
                                CUR_LANE.extruder_obj.lane_loaded = CUR_LANE.name
                        else:
                            if CUR_LANE.get_toolhead_sensor_state() == True:
                                msg +="<span class=error--text> error in ToolHead. \nLane identified as loaded in AFC.vars.unit file\n but not identified as loaded in AFC.var.tool file</span>"
                                succeeded = False
                    else:
                        lane_check=self.AFC.ERROR.fix('toolhead',CUR_LANE)  #send to error handling
                        if not lane_check:
                            return False

        if assignTcmd: self.AFC.TcmdAssign(CUR_LANE)
        CUR_LANE.do_enable(False)
        self.AFC.gcode.respond_info( '{lane_name} tool cmd: {tcmd:3} {msg}'.format(lane_name=CUR_LANE.name.upper(), tcmd=CUR_LANE.map, msg=msg))
        CUR_LANE.set_afc_prep_done()

        return succeeded