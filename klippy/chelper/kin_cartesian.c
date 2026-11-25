// Cartesian kinematics stepper pulse time generation
//
// Copyright (C) 2018-2019  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <stdlib.h> // malloc
#include <string.h> // memset
#include <math.h> // cos, sin
#include "compiler.h" // __visible
#include "itersolve.h" // struct stepper_kinematics
#include "pyhelper.h" // errorf
#include "trapq.h" // move_get_coord

static double
cart_stepper_x_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).x;
}

static double
cart_stepper_y_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).y;
}

static double
cart_stepper_z_calc_position(struct stepper_kinematics *sk, struct move *m
                             , double move_time)
{
    return move_get_coord(m, move_time).z;
}

struct stepper_kinematics * __visible
cartesian_stepper_alloc(char axis)
{
    struct stepper_kinematics *sk = malloc(sizeof(*sk));
    memset(sk, 0, sizeof(*sk));
    if (axis == 'x') {
        sk->calc_position_cb = cart_stepper_x_calc_position;
        sk->active_flags = AF_X;
    } else if (axis == 'y') {
        sk->calc_position_cb = cart_stepper_y_calc_position;
        sk->active_flags = AF_Y;
    } else if (axis == 'z') {
        sk->calc_position_cb = cart_stepper_z_calc_position;
        sk->active_flags = AF_Z;
    }
    return sk;
}

struct rctp_stepper {
    struct stepper_kinematics sk;
    char type;
    double pivot_x, pivot_y, pivot_z;
    double support_x, support_y;
    double support_offset;
};

static void
rctp_transform(struct coord p, double pivot_x, double pivot_y, double pivot_z
               , double *x, double *y, double *z)
{
    double a = p.a * (M_PI / 180.), b = p.b * (M_PI / 180.);
    double ca = cos(a), sa = sin(a), cb = cos(b), sb = sin(b);
    double px = p.x - pivot_x, py = p.y - pivot_y, pz = p.z - pivot_z;
    double rx = cb * px + sb * pz;
    double ry = ca * py - sa * (-sb * px + cb * pz);
    double rz = sa * py + ca * (-sb * px + cb * pz);
    *x = pivot_x + rx;
    *y = pivot_y + ry;
    *z = pivot_z + rz;
}

static double
rctp_stepper_calc_position(struct stepper_kinematics *sk, struct move *m
                           , double move_time)
{
    struct rctp_stepper *rs = container_of(
            sk, struct rctp_stepper, sk);
    struct coord p = move_get_coord(m, move_time);
    double x, y, z;
    rctp_transform(p, rs->pivot_x, rs->pivot_y, rs->pivot_z, &x, &y, &z);
    if (rs->type == 'x')
        return x;
    if (rs->type == 'y')
        return y;
    if (rs->type == '+')
        return x + y;
    if (rs->type == '-')
        return x - y;
    // Transform the support offset from the commanded bed point.  This
    // makes all three supports equal the requested Z at zero tilt.
    double a = p.a * (M_PI / 180.), b = p.b * (M_PI / 180.);
    double ca = cos(a), ta = tan(a), tb = tan(b);
    return z + ta * (rs->support_y - y)
           - tb / ca * (rs->support_x - x) + rs->support_offset;
}

struct stepper_kinematics * __visible
rctp_stepper_alloc(char type, double pivot_x, double pivot_y, double pivot_z
                   , double support_x, double support_y)
{
    struct rctp_stepper *rs = malloc(sizeof(*rs));
    memset(rs, 0, sizeof(*rs));
    rs->sk.calc_position_cb = rctp_stepper_calc_position;
    rs->sk.active_flags = AF_X | AF_Y | AF_Z | AF_A | AF_B;
    rs->type = type;
    rs->pivot_x = pivot_x;
    rs->pivot_y = pivot_y;
    rs->pivot_z = pivot_z;
    rs->support_x = support_x;
    rs->support_y = support_y;
    return &rs->sk;
}

void __visible
rctp_stepper_set_offset(struct stepper_kinematics *sk, double offset)
{
    struct rctp_stepper *rs = container_of(
            sk, struct rctp_stepper, sk);
    rs->support_offset = offset;
}
