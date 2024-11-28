import stepper, chelper, logging, configparser
from . import homing

# class error(Exception):
#     pass

class MultiFunctionEndstop:
    error = configparser.Error
    def __init__(self, config):
        self.printer = config.get_printer()

        # get gcode
        gcode = self.printer.lookup_object('gcode')
        gcode_macro = self.printer.load_object(config, 'gcode_macro')
        self.activate_gcode = gcode_macro.load_template(
            config, 'activate_gcode', '')
        self.deactivate_gcode = gcode_macro.load_template(
            config, 'deactivate_gcode', '')

        # get endstop_pin
        # Create an "endstop" object to handle the probe pin
        endstop_pin = config.get('endstop_pin')
        ppins = self.printer.lookup_object('pins')
        pin_params = ppins.parse_pin(endstop_pin, True, True)
        # Normalize pin name
        pin_name = "%s:%s" % (pin_params['chip_name'], pin_params['pin'])

        self.endstops = []
        # New endstop, register it
        self.mcu_endstop = ppins.setup_pin('endstop', endstop_pin)
        # name = stepper.get_name(short=True)
        name = "multi_function_endstop"
        self.endstops.append((self.mcu_endstop, name))
        query_endstops = self.printer.load_object(config, 'query_endstops')
        query_endstops.register_endstop(self.mcu_endstop, name)

        # get samples
        self.samples = config.getint('samples', 1, minval=1)
        self.sample_retract_dist = config.getfloat(
            'sample_retract_dist', 0.)
        self.sample_extend_compensation = config.getfloat(
            'sample_extend_compensation', 1.)
        logging.info("samples:%d" % (self.samples,))

        # get park_pos
        # self.park_pos = config.getlists('park_pos', seps=(',', '\n'),
        #                                 parser=float, count=3)
        # < 0:  Move in the negative direction, and the position information
        #       is reset after the end
        # > 0:  Move in the positive direction and record the position
        #       information after finishing
        
        self.stepper_map = {}
        self.stepper_objects = {}
        self.move_speed = config.getint('move_speed', 5)
        self.move_distance = config.getfloat('move_distance')

        # get mode
        self.mode = config.get('mode')
        logging.info("MultiFunctionEndstop mode:%s" % (self.mode,))
        # This mode is used to confirm the offset of
        # the manual stepper to endstop 
        # (if the manual stepper has already returned home).
        if self.mode == 'manual_stepper_offset':
            self.stepper_name = config.getlist('manual_stepper_name')
            self.stepper_len = len(self.stepper_name)
            logging.info("stepper number:%d" % (self.stepper_len,))
            for i in list(range(0, self.stepper_len, 1)):
                # self.stepper_pos[self.stepper_name[i]] = config.getfloatlist(
                #                             'park_pos_%d' % (i,), count=2)
                # self.stepper_offset[self.stepper_name[i]] = None
                self.stepper_objects[self.stepper_name[i]] = (
                    self.printer.lookup_object(
                        'manual_stepper '+self.stepper_name[i]))
                self.stepper_map[self.stepper_name[i]] = {
                    'park_pos': config.getfloatlist('park_pos_%d' % (i,),
                                                    count=2),
                    'move_distance': self.move_distance,
                    'offset': None,
                }
            logging.info("stepper_map: %s" % (self.stepper_map,))
            logging.info("stepper_objects: %s" % (self.stepper_objects,))
        elif self.mode == 'manual_axis_touch':
            self.axis = config.get('axis').lower()
            # self.axis = 'x'
            self.stepper_map[self.axis] = {
                'park_pos': config.getfloatlist('park_pos', count = 3),
                'move_distance': self.move_distance,
                'touch_pos': None,
            }
            logging.info("MultiFunctionEndstop touch_mode")

        gcode.register_command('MULTI_FUNCTION_ENDSTOP_AXIS_TOUCH',
            self.cmd_MULTI_FUNCTION_ENDSTOP_AXIS_TOUCH,
            desc=self.cmd_MULTI_FUNCTION_ENDSTOP_AXIS_TOUCH_help)
        # gcode.register_command('MULTI_FUNCTION_ENDSTOP_SET_POS',
        #     self.cmd_MULTI_FUNCTION_ENDSTOP_SET_POS,
        #     desc=self.cmd_MULTI_FUNCTION_ENDSTOP_SET_POS_help)

        gcode.register_command('MULTI_FUNCTION_ENDSTOP_START',
            self.cmd_START_CALIBRATION,
            desc=self.cmd_START_CALIBRATION_help)
        
        # Register event handlers
        self.printer.register_event_handler("klippy:connect",
                                            self._handle_connect)
        self.printer.register_event_handler('klippy:mcu_identify',
                                            self._handle_mcu_identify)
    
    # Register event handlers
    def _handle_connect(self):
        self.toolhead = self.printer.lookup_object('toolhead')
        # flag = 0
        # kin = self.toolhead.get_kinematics()
        # for stepper in kin.get_steppers():
        #     if stepper.is_active_axis(self.axis):
        #         self.stepper_objects[self.axis] = stepper
        #         self.mcu_endstop.add_stepper(stepper)
        #         flag = 1
        #         break
        # if not flag:
        #     # raise error("Can't find the %s axis!" % (self.axis,))
        #     self.stepper_objects[self.axis] = None
        #     raise self.printer.command_error(
        #             "Can't find the %s axis!" % (self.axis,))
    def _handle_mcu_identify(self):
        flag = 0
        kin = self.printer.lookup_object('toolhead').get_kinematics()
        for stepper in kin.get_steppers():
            if stepper.is_active_axis(self.axis):
                self.stepper_objects[self.axis] = stepper
                self.mcu_endstop.add_stepper(stepper)
                flag = 1
                break
        if not flag:
            # raise error("Can't find the %s axis!" % (self.axis,))
            self.stepper_objects[self.axis] = None
            raise self.printer.command_error(
                    "Can't find the %s axis!" % (self.axis,))

    # get status
    def get_status(self, eventtime):
        if ((self.mode == 'manual_stepper_offset') or 
            (self.mode == 'manual_axis_touch')):
            # return {'mode': self.mode,
            #         'position': self.stepper_pos,
            #         'offset': self.stepper_offset,}
            return self.stepper_map

    # cmd_MULTI_FUNCTION_ENDSTOP_SET_POS_help = ""
    # def cmd_MULTI_FUNCTION_ENDSTOP_SET_POS(self, gcmd):
    #     if self.mode == 'manual_stepper_offset':
            
    #     elif self.mode == 'manual_axis_touch':
            

    def _move(self, coord, speed):
        self.toolhead.move(coord, speed)

    cmd_START_CALIBRATION_help = "Perform calibration"
    def cmd_START_CALIBRATION(self, gcmd):
        self.activate_gcode.run_gcode_from_command()
        self.multi_function_endstop_begin(gcmd)
        self.deactivate_gcode.run_gcode_from_command()

    def manual_home_to_endstop(self, endstops, pos, speed, toolhead=None,
                               triggered=True, check_triggered=True):
        hmove = homing.HomingMove(self.printer, endstops, toolhead)
        # epos = None
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            raise
        if hmove.check_no_movement() is not None:
            raise self.printer.command_error(
                "Probe triggered prior to movement")
        return epos

    def multi_function_endstop_begin(self, gcmd):
        if self.mode == 'manual_stepper_offset':
            gcmd.respond_info("multi_function_endstop_begin")
            for i in list(range(0, self.stepper_len, 1)):
                stepper_object = self.stepper_objects[self.stepper_name[i]]
                self.mcu_endstop.add_stepper(stepper_object.get_steppers()[0])
                samples_sum = 0
                offset = 0.0
                retract_pos = 0.0
                pos = [self.move_distance, 0., 0., 0.]
                for num in list(range(0, self.samples, 1)):
                    epos = self.manual_home_to_endstop(list(self.endstops),
                            pos, self.move_speed, stepper_object, True, True)
                    samples_sum += epos[0]
                    logging.info("%s samples:%d, pos:%s" % 
                                 (self.stepper_name[i], num, epos[0],))
                    if self.sample_retract_dist > 0.0001:
                        retract_pos = epos[0] - self.sample_retract_dist
                        stepper_object.do_move(retract_pos, self.move_speed,
                                               stepper_object.accel)
                    # In multi-sample sampling,
                    # for a predetermined displacement,
                    # 'sample_extend_compensation' is employed to
                    # enhance movement efficiency.
                    pos[0] = epos[0] + self.sample_extend_compensation
                offset = samples_sum/self.samples
                self.stepper_map[self.stepper_name[i]]['offset'] = offset
                logging.info("%s: %s pos: %s" % (self.stepper_name[i], offset,
                                stepper_object.rail.get_commanded_position()))

    # def touch_move(self, pos, speed):
    #     phoming = self.printer.lookup_object('homing')
    #     return phoming.probing_move(self, pos, speed)
    # def probing_move(self, mcu_probe, pos, speed):
    #     hmove = HomingMove(self.printer, endstops)
    #     try:
    #         epos = hmove.homing_move(pos, speed, probe_pos=True)
    #     except self.printer.command_error:
    #         if self.printer.is_shutdown():
    #             raise self.printer.command_error(
    #                 "Probing failed due to printer shutdown")
    #         raise
    #     if hmove.check_no_movement() is not None:
    #         raise self.printer.command_error(
    #             "Probe triggered prior to movement")
    #     return epos

    cmd_MULTI_FUNCTION_ENDSTOP_AXIS_TOUCH_help = "Specify the axis to "\
                                                 "touch the endstop"
    def cmd_MULTI_FUNCTION_ENDSTOP_AXIS_TOUCH(self, gcmd):
        curtime = self.printer.get_reactor().monotonic()
        # stepper_object = self.stepper_objects[self.axis]
        stepper_object = self.toolhead
        gcode = self.printer.lookup_object('gcode')
        park_pos_x = gcmd.get_float('X', self.stepper_map[self.axis]['park_pos'][0])
        park_pos_y = gcmd.get_float('Y', self.stepper_map[self.axis]['park_pos'][1])
        park_pos_z = gcmd.get_float('Z', self.stepper_map[self.axis]['park_pos'][2])
        park_pos_f = gcmd.get_int('F', self.move_speed * 60)
        axis_minimum = self.toolhead.get_status(curtime)["axis_minimum"]
        axis_maximum = self.toolhead.get_status(curtime)["axis_maximum"]
        if self.stepper_objects[self.axis] is not None:
            # 是否全归位了
            if 'xyz' not in self.toolhead.get_status(curtime)["homed_axes"]:
                raise self.printer.command_error("Must home before probe")
            # 通过G0指令将打印头移动到指定位置
            gcode.run_script_from_command("G0 X%.2f Y%.2f Z%.2f F%d" % (
                park_pos_x,
                park_pos_y,
                park_pos_z,
                park_pos_f,
            ))
            gcode.run_script_from_command("M400")
            # 确定数据
            axis_to_num = ord(self.axis) - ord('x')
            pos = self.toolhead.get_position()
            logging.info("multi_function: %s" % (pos,))
            retract_pos = list(pos)
            touch_pos = list(pos)
            
            if (self.move_distance >= 0):
                touch_pos[axis_to_num] = min(
                    (touch_pos[axis_to_num] + self.move_distance),
                    axis_maximum[axis_to_num])
            else:
                touch_pos[axis_to_num] = max(
                    (touch_pos[axis_to_num] + self.move_distance),
                    axis_minimum[axis_to_num])
                
            pos_info = 0
            samples_sum = 0
            # 重复获取样本
            for num in list(range(0, self.samples, 1)):
                # 开始去触碰endstop
                try:
                    # epos = self.mcu_endstop.touch_move(
                    #     touch_pos, self.move_speed)
                    epos = self.manual_home_to_endstop(list(self.endstops),
                            touch_pos, self.move_speed)
                except self.printer.command_error as e:
                    reason = str(e)
                    if "Timeout during endstop homing" in reason:
                        reason += HINT_TIMEOUT
                    raise self.printer.command_error(reason)
                # Allow axis_twist_compensation to update results
                self.printer.send_event("probe:update_results", epos)
                # Report results
                gcode = self.printer.lookup_object('gcode')
                gcode.respond_info("%s trigger : %.3f"
                                % (self.axis, epos[axis_to_num],))
                samples_sum += epos[axis_to_num]
                # 回缩
                if (self.sample_retract_dist > 0.001):
                    retract_pos[axis_to_num] = (epos[axis_to_num] -
                                                self.sample_retract_dist)
                    self._move(retract_pos, self.move_speed)
                # 计算下一次触摸终结位置
                touch_pos[axis_to_num] = (epos[axis_to_num] + 
                                            self.sample_extend_compensation)
            
            # 记录位置信息, 打印信息?
            touch_pos[axis_to_num] = samples_sum/self.samples
            self.stepper_map[self.axis]['touch_pos'] = touch_pos[:]
            logging.info("%s axis touch pos %s" % (self.axis, self.stepper_map))
        else:
            self.printer.command_error(
                "Can't find the %s axis!" % (self.axis,))
        

def load_config(config):
    return MultiFunctionEndstop(config)
