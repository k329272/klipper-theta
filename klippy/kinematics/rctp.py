# Rigid table coordinate transform for a three- or four-support tilting bed.
# Copyright (C) 2026  OpenAI
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

import chelper

class RCTPAxis:
    """Virtual G-code angle axis; the Z supports produce the tilt."""
    def __init__(self, name, config):
        self.name = name
        self.max_velocity = config.getfloat(
            'max_angular_velocity', 1., above=0.)
        self.max_accel = config.getfloat(
            'max_angular_accel', 10., above=0.)
        self.instant_corner_v = config.getfloat(
            'instantaneous_corner_velocity', 1., minval=0.)
    def get_name(self):
        return 'rctp_' + self.name.lower()
    def get_axis_gcode_id(self):
        return self.name
    def process_move(self, print_time, move, ea_index):
        pass
    def check_move(self, move, ea_index):
        angle_d = abs(move.axes_d[ea_index])
        if not angle_d:
            return
        ratio = move.move_d / angle_d
        move.limit_speed(self.max_velocity * ratio,
                         self.max_accel * ratio)
    def calc_junction(self, prev_move, move, ea_index):
        diff_r = move.axes_r[ea_index] - prev_move.axes_r[ea_index]
        if diff_r:
            return (self.instant_corner_v / abs(diff_r))**2
        return move.max_cruise_v2


class RCTP:
    def __init__(self, toolhead, config):
        self.toolhead = toolhead
        self.printer = config.get_printer()
        self.config = config.getsection('rctp')
        self.pivot = self.config.getfloatlist('pivot', count=3)
        self.max_tilt = self.config.getfloat(
            'max_tilt', 30., above=0., maxval=89.)
        zpos = self.config.getfloatlist('z_positions')
        if len(zpos) not in (6, 8):
            raise self.config.error(
                "z_positions must contain three or four XY support pairs")
        self.supports = list(zip(zpos[::2], zpos[1::2]))
        if not self._supports_span_plane(self.supports):
            raise self.config.error(
                "z_positions must describe non-collinear support points")
        self.axes = [RCTPAxis('A', self.config),
                     RCTPAxis('B', self.config)]
        self.z_steppers = []
        self.z_offsets = []
        # Register after configuration is loaded, before the printer becomes
        # ready, so normal G-code position tracking sees A and B.
        self.printer.register_event_handler('klippy:connect', self._connect)
        self.printer.register_event_handler(
            'z_tilt:steppers_adjusted', self._handle_tramming)
    def _connect(self):
        registered = [ea.get_axis_gcode_id()
                      for ea in self.toolhead.get_extra_axes() if ea]
        if any(axis in registered for axis in ('A', 'B')):
            raise self.config.error(
                "RCTP uses virtual G-code axes A and B; remove other A/B "
                "extra axes")
        for axis in self.axes:
            self.toolhead.add_extra_axis(axis, 0.)
    def setup_rail(self, rail, types):
        """Install transformed solvers. `types` maps each rail motor to a mode."""
        steppers = rail.get_steppers()
        if len(types) != len(steppers):
            raise self.config.error(
                "RCTP needs a solver type for each stepper on a rail")
        for s, typ in zip(steppers, types):
            s.setup_itersolve('rctp_stepper_alloc', typ.encode(), *self.pivot,
                              0., 0.)
    def setup_z_rail(self, rail):
        steppers = rail.get_steppers()
        if len(steppers) != len(self.supports):
            raise self.config.error(
                "RCTP requires one stepper_z motor for each z_positions pair")
        for s, (sx, sy) in zip(steppers, self.supports):
            s.setup_itersolve('rctp_stepper_alloc', b'z', *self.pivot, sx, sy)
        self.z_steppers = steppers
        self.z_offsets = [0.] * len(steppers)
    def _handle_tramming(self, adjustments, steppers, tramming_offsets):
        if [s.get_name() for s in steppers] != [
                s.get_name() for s in self.z_steppers]:
            return
        if len(adjustments) != len(self.z_steppers):
            raise self.config.error(
                "Z tramming adjustment count does not match RCTP supports")
        if tramming_offsets is None \
                or len(tramming_offsets) != len(self.z_steppers):
            raise self.config.error(
                "Z tramming did not provide RCTP support offsets")
        # ZAdjustHelper measures the change in each motor's MCU-to-command
        # offset during its physical tram moves. Apply that change to the
        # support solver so the trammed bed plane remains in the RCTP frame.
        _ffi_main, ffi_lib = chelper.get_ffi()
        for i, (s, delta) in enumerate(zip(self.z_steppers,
                                           tramming_offsets)):
            self.z_offsets[i] += delta
            ffi_lib.rctp_stepper_set_offset(
                s.get_stepper_kinematics(), self.z_offsets[i])
        pos = self.toolhead.get_position()
        self.toolhead.set_position(pos)
        logging.info("RCTP tramming adjustments %s; support offsets %s",
                     adjustments, list(zip(
                         [s.get_name() for s in self.z_steppers],
                         self.z_offsets)))
        report = self.printer.lookup_object('gcode')
        report.respond_info("RCTP support offsets: %s" % (" ".join(
            "%s=%.6f" % (s.get_name(), off)
            for s, off in zip(self.z_steppers, self.z_offsets))))
    def position_from_steppers(self, x, y, z_positions):
        """Return bed-local XYZ and measured A/B from machine positions."""
        # Fit z = slope_x*x + slope_y*y + intercept.  With four motors this
        # least-squares fit averages small differences in their positions.
        count = len(self.supports)
        mean_x = sum(p[0] for p in self.supports) / count
        mean_y = sum(p[1] for p in self.supports) / count
        mean_z = sum(z_positions) / count
        xx = sum((x0-mean_x)**2 for x0, _ in self.supports)
        yy = sum((y0-mean_y)**2 for _, y0 in self.supports)
        xy = sum((x0-mean_x)*(y0-mean_y)
                 for x0, y0 in self.supports)
        xz = sum((x0-mean_x)*(z0-mean_z)
                 for (x0, _), z0 in zip(self.supports, z_positions))
        yz = sum((y0-mean_y)*(z0-mean_z)
                 for (_, y0), z0 in zip(self.supports, z_positions))
        det = xx*yy - xy*xy
        if abs(det) < 1.e-8:
            raise self.config.error(
                "z_positions must describe non-collinear support points")
        slope_x = (xz*yy - yz*xy) / det
        slope_y = (yz*xx - xz*xy) / det
        a = math.atan(slope_y)
        b = math.atan(-slope_x * math.cos(a))
        # The support plane gives the machine-space bed height at the XY point.
        z = mean_z + slope_x*(x-mean_x) + slope_y*(y-mean_y)
        px, py, pz = self.pivot
        ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
        x0, y0, z0 = x-px, y-py, z-pz
        # Inverse of R_x(A) R_y(B), applied about the configured pivot.
        lx = cb*x0 + sb*sa*y0 - sb*ca*z0
        ly = ca*y0 + sa*z0
        lz = sb*x0 - cb*sa*y0 + cb*ca*z0
        return [px+lx, py+ly, pz+lz, math.degrees(a), math.degrees(b)]
    def z_positions_from_steppers(self, rail, stepper_positions):
        return [stepper_positions[s.get_name()] for s in rail.get_steppers()]
    def _supports_span_plane(self, supports):
        mean_x = sum(p[0] for p in supports) / len(supports)
        mean_y = sum(p[1] for p in supports) / len(supports)
        xx = sum((x-mean_x)**2 for x, _ in supports)
        yy = sum((y-mean_y)**2 for _, y in supports)
        xy = sum((x-mean_x)*(y-mean_y) for x, y in supports)
        return xx*yy - xy*xy >= 1.e-8
    def check_move(self, move, x_range, y_range, z_rail,
                   max_z_velocity, max_z_accel):
        pos0, pos1 = move.start_pos, move.end_pos
        linear_span = max(abs(d) for d in move.axes_d[:3])
        angle_span = max(abs(d) for d in move.axes_d[4:6])
        samples = max(1, min(100, int(max(linear_span / 5., angle_span)) + 1))
        prev_supports = None
        for sample in range(samples + 1):
            t = sample / float(samples)
            x, y, z = (pos0[i] + t * move.axes_d[i] for i in range(3))
            a = pos0[4] + t * move.axes_d[4]
            b = pos0[5] + t * move.axes_d[5]
            support_heights = self._check_position(
                move, x, y, z, a, b, x_range, y_range, z_rail)
            if prev_supports is not None:
                segment_d = move.move_d / samples
                for prev_z, cur_z in zip(prev_supports, support_heights):
                    dz = abs(cur_z - prev_z)
                    if dz:
                        ratio = segment_d / dz
                        move.limit_speed(max_z_velocity * ratio,
                                         max_z_accel * ratio)
            prev_supports = support_heights
    def _check_position(self, move, x, y, z, a, b, x_range, y_range,
                        z_rail):
        px, py, pz = self.pivot
        a, b = math.radians(a), math.radians(b)
        ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
        tilt = math.degrees(math.acos(max(-1., min(1., ca*cb))))
        if tilt > self.max_tilt:
            raise move.move_error("RCTP bed tilt exceeds max_tilt")
        dx, dy, dz = x-px, y-py, z-pz
        machine_x = px + cb*dx + sb*dz
        machine_y = py + ca*dy + sa*sb*dx - sa*cb*dz
        if ((x_range[0] <= x_range[1]
             and not x_range[0] <= machine_x <= x_range[1])
            or (y_range[0] <= y_range[1]
                and not y_range[0] <= machine_y <= y_range[1])):
            raise move.move_error()
        zmin, zmax = z_rail.get_range()
        machine_z = pz + sa*dy + ca*(-sb*dx + cb*dz)
        support_heights = []
        for i, (sx, sy) in enumerate(self.supports):
            support_z = machine_z + math.tan(a)*(sy-machine_y) \
                        - math.tan(b)/ca*(sx-machine_x)
            support_z += self.z_offsets[i]
            if support_z < zmin or support_z > zmax:
                raise move.move_error()
            support_heights.append(support_z)
        return support_heights
