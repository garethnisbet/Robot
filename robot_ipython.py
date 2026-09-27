#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot/Device — IPython Remote Control Terminal

Interactive IPython client for controlling the viewer via WebSocket API.
Works with any device config (Meca500, i16 diffractometer, etc.).
Exposes a `robot` object with methods for all commands — use full Python
syntax (loops, variables, etc.) alongside robot control.

Usage:
    pip install websockets ipython
    python robot_ipython.py [--url ws://localhost:8080/ws] [--config meca500_config.json]
"""

import argparse
import time

import numpy as np
from GNKinematics import kinematics
from RobotDefinitions import Meca500_kin, GP225_kin, GP180_120_kin, GP280_kin, MotoMini_kin
from robot_client import (
    RobotClient, _COLORS, _is_array_like,
    _bold, _bred, _cyan, _dim, _yellow,
)

# ═══════════════════════════════════════════════════════════════════════════════
#  IPython Integration
# ═══════════════════════════════════════════════════════════════════════════════

def _build_banner(robot):
    """Build the startup banner string."""
    name = "DLS Collision Model"
    lines = []
    if _COLORS:
        lines.append(_cyan(f"\n  \u2554{'=' * 42}\u2557"))
        lines.append(_cyan(f"  \u2551") + _bold(f"  {name:^38s}") + _cyan(f"  \u2551"))
        lines.append(_cyan(f"  \u255a{'=' * 42}\u255d"))
    else:
        lines.append(f"\n  {name}")
        lines.append(f"  {'=' * len(name)}")

    lines.append(f"\n  Remote Control Terminal v3.0 (IPython)")
    lines.append(f"  Server: {_bold(robot.url) if _COLORS else robot.url}")

    if robot._movable_joints:
        names = ", ".join(name for _, name in robot._movable_joints)
        lines.append(f"  Joints ({robot._n_movable}): {_dim(names)}")

    lines.append(f"\n  Python: {_bold('robot.<Tab>')}  |  Space-separated: {_bold('home')}, {_bold('joints 0 30 60 0 45 90')}, ...")
    lines.append(f"  Type {_bold('robot.help()')} or {_bold('rhelp')} for full docs.")
    lines.append("")
    return "\n".join(lines)


from IPython.terminal.prompts import Prompts


class _RobotPrompts(Prompts):
    """Custom IPython prompts showing the device name."""

    def __init__(self, shell):
        super().__init__(shell)

    def in_prompt_tokens(self):
        from pygments.token import Token
        robot = self.shell.user_ns.get("robot")
        name = robot.name.lower().replace(" ", "_") if robot else "robot"
        return [
            (Token.Prompt, f"{name} "),
            (Token.Prompt, "["),
            (Token.PromptNum, str(self.shell.execution_count)),
            (Token.Prompt, "]: "),
        ]

    def out_prompt_tokens(self):
        from pygments.token import Token
        return [
            (Token.OutPrompt, "Out"),
            (Token.OutPromptNum, f"[{self.shell.execution_count}]"),
            (Token.OutPrompt, ": "),
        ]

    def continuation_prompt_tokens(self, width=None):
        from pygments.token import Token
        if width is None:
            width = self._width()
        spaces = " " * (width - 5)
        return [
            (Token.Prompt, f"{spaces}...: "),
        ]

    def rewrite_prompt_tokens(self):
        from pygments.token import Token
        return [
            (Token.Prompt, ""),
        ]

    def _width(self):
        robot = self.shell.user_ns.get("robot")
        name = robot.name.lower().replace(" ", "_") if robot else "robot"
        count_str = str(self.shell.execution_count)
        return len(name) + 1 + len(count_str) + 4  # "name [N]: "


# ── Line magics (space-separated syntax) ─────────────────────────────────────

def _register_magics(ipython, robot):
    """Register line magics so the original space-separated syntax works.

    With automagic (on by default), the % prefix is optional:
        joints 0 30 60 0 45 90
        joint J1 45
        pos meca500 [0, 0, 0, 0, 0, 0]
        pos meca500 np.zeros(6)
        inc meca500 [0, 0, 0, 0, 0, 10]
        move 150 100 300
        scan J1 0 90 5
        scan J1 J2 polar_func()
    """
    reg = ipython.register_magic_function

    # ── Simple commands ──────────────────────────────────────────────

    def _m_state(line):
        robot.state()
    reg(_m_state, magic_name='state')

    def _m_home(line):
        robot.home()
    reg(_m_home, magic_name='home')

    def _m_fk(line):
        robot.fk()
    reg(_m_fk, magic_name='fk')

    def _m_ik(line):
        robot.ik()
    reg(_m_ik, magic_name='ik')

    def _m_demo(line):
        robot.demo()
    reg(_m_demo, magic_name='demo')

    # ── Joint commands ───────────────────────────────────────────────

    def _m_joints(line):
        """joints 0 30 60 0 45 90"""
        parts = line.split()
        if not parts:
            robot.state()
            return
        robot.joints(*[float(x) for x in parts])
    reg(_m_joints, magic_name='joints')

    def _m_joint(line):
        """joint J1 45"""
        parts = line.split()
        if len(parts) < 2:
            print(f"  {_yellow('Usage')}: joint <name> <angle>")
            return
        robot.joint(parts[0], float(parts[1]))
    reg(_m_joint, magic_name='joint')

    def _m_pos(line):
        """pos <device> <value>

        value is a Python expression evaluated in the IPython namespace:
        a list/array/callable for the full joint vector, OR a dict
        {axis: angle} to set individual axes (axis is index or joint name).

        Examples:
            pos meca500 [0, 0, 0, 0, 0, 0]
            pos meca500 np.zeros(6)
            pos meca500 my_pose_func()
            pos meca500 my_pose_func        # callable; invoked with no args
            pos meca500 {2: 45}             # set only axis index 2
            pos meca500 {'J4': 10, 'J6': -30}
        """
        stripped = line.strip()
        if not stripped:
            print(f"  {_yellow('Usage')}: pos <device> <list|array|dict|callable>")
            print(f"  Example: {_bold('pos meca500 [0,0,0,0,0,0]')}")
            print(f"  Example: {_bold('pos meca500 {2: 45}')}  # single axis")
            return
        parts = stripped.split(None, 1)
        if len(parts) < 2:
            print(f"  {_yellow('Usage')}: pos <device> <list|array|dict|callable>")
            return
        device, expr = parts[0], parts[1]
        from IPython import get_ipython
        ip = get_ipython()
        try:
            value = ip.ev(expr)
        except Exception as e:
            print(f"  {_bred('Error')}: failed to evaluate {expr!r}: {e}")
            return
        robot.set_pos(device, value)
    reg(_m_pos, magic_name='pos')

    def _m_inc(line):
        """inc <device> <value>

        Like pos, but adds the given deltas to the device's current joint angles.
        Accepts a list/array/callable for all axes, or a dict {axis: delta}
        to increment only individual axes (axis is index or joint name).

        Examples:
            inc meca500 [0, 0, 0, 0, 0, 10]
            inc meca500 np.array([1, -1, 0, 0, 0, 0])
            inc meca500 my_delta_func()
            inc meca500 {5: 10}              # move only axis index 5
            inc meca500 {'J6': 10, 'J4': -5}
        """
        stripped = line.strip()
        if not stripped:
            print(f"  {_yellow('Usage')}: inc <device> <list|array|dict|callable>")
            print(f"  Example: {_bold('inc meca500 [0,0,0,0,0,10]')}")
            print(f"  Example: {_bold('inc meca500 {5: 10}')}  # single axis")
            return
        parts = stripped.split(None, 1)
        if len(parts) < 2:
            print(f"  {_yellow('Usage')}: inc <device> <list|array|dict|callable>")
            return
        device, expr = parts[0], parts[1]
        from IPython import get_ipython
        ip = get_ipython()
        try:
            value = ip.ev(expr)
        except Exception as e:
            print(f"  {_bred('Error')}: failed to evaluate {expr!r}: {e}")
            return
        robot.inc_pos(device, value)
    reg(_m_inc, magic_name='inc')

    def _ee_move_magic(line, incremental):
        """Shared parser for eepos/eeinc: <expr> [--device name] [--space frame].

        Like pos/inc but Cartesian: the value is an [x,y,z,a,b,g] list/array,
        a {axis: value} dict (axis x,y,z,a,b,g), or a callable, solved with the
        Python IK. Targets the active device unless --device is given.
        """
        name = 'eeinc' if incremental else 'eepos'
        usage = f"{name} <list|dict|callable> [--device name] [--space world]"
        raw = line.split()
        space, device = 'local', None
        for flag, setter in (('--space', 'space'), ('--device', 'device')):
            if flag in raw:
                i = raw.index(flag)
                if i + 1 >= len(raw):
                    print(f"  {_yellow('Usage')}: {usage}")
                    return
                if setter == 'space':
                    space = raw[i + 1]
                else:
                    device = raw[i + 1]
                raw = raw[:i] + raw[i + 2:]
        if not raw:
            print(f"  {_yellow('Usage')}: {usage}")
            print(f"  Example: {_bold(name + ' [200,0,400,0,90,0]')}")
            print(f"  Example: {_bold(name + ' {' + chr(39) + 'z' + chr(39) + ': 450} --device GP180_120 --space world')}")
            return
        expr = ' '.join(raw)
        from IPython import get_ipython
        ip = get_ipython()
        try:
            value = ip.ev(expr)
        except Exception as e:
            print(f"  {_bred('Error')}: failed to evaluate {expr!r}: {e}")
            return
        (robot.eeinc if incremental else robot.eepos)(value, device=device, space=space)

    def _m_eepos(line):
        """eepos <list|dict|callable> [--device name] [--space local|world]

        Move an end-effector to an ABSOLUTE Cartesian pose [x,y,z,a,b,g]
        (mm, ZYX-Euler deg) via the Python IK. Unspecified components hold.

        Examples:
            eepos [200, 0, 400, 0, 90, 0]
            eepos {'z': 450}
            eepos {'x': 4334} --device GP180_120 --space world
            eepos my_pose_func()
        """
        _ee_move_magic(line, incremental=False)
    reg(_m_eepos, magic_name='eepos')

    def _m_eeinc(line):
        """eeinc <list|dict|callable> [--device name] [--space local|world]

        Like eepos, but ADD the values to the current EE pose (jog).

        Examples:
            eeinc [0, 0, 50]
            eeinc {'y': -100}
            eeinc {'a': 10} --device GP180_120 --space world
        """
        _ee_move_magic(line, incremental=True)
    reg(_m_eeinc, magic_name='eeinc')

    # ── IK commands ──────────────────────────────────────────────────

    def _m_move(line):
        """move x y z [a b g]"""
        parts = line.split()
        if len(parts) < 3:
            print(f"  {_yellow('Usage')}: move x y z [a b g]")
            return
        robot.move(*[float(x) for x in parts[:6]])
    reg(_m_move, magic_name='move')

    def _m_target(line):
        """target x y z [a b g]"""
        parts = line.split()
        if len(parts) < 3:
            print(f"  {_yellow('Usage')}: target x y z [a b g]")
            return
        robot.target(*[float(x) for x in parts[:6]])
    reg(_m_target, magic_name='target')

    # ── Collision ────────────────────────────────────────────────────

    def _m_collision(line):
        """collision [on|off|headless [on|off]|floor [on|off]]"""
        arg = line.strip().lower()
        if arg.startswith("headless"):
            rest = arg[len("headless"):].strip()
            if rest == "on":
                robot.collision_headless(True)
            elif rest == "off":
                robot.collision_headless(False)
            else:
                robot.collision_headless()
        elif arg.startswith("floor"):
            rest = arg[len("floor"):].strip()
            if rest == "on":
                robot.collision_floor(True)
            elif rest == "off":
                robot.collision_floor(False)
            else:
                robot.collision_floor()
        elif arg == "on":
            robot.collision(True)
        elif arg == "off":
            robot.collision(False)
        else:
            robot.collision()
    reg(_m_collision, magic_name='collision')

    def _m_collisions(line):
        robot.collisions()
    reg(_m_collisions, magic_name='collisions')

    # ── Object commands ──────────────────────────────────────────────

    def _m_objects(line):
        robot.objects()
    reg(_m_objects, magic_name='objects')

    def _m_obj(line):
        """obj <name|#idx>"""
        arg = line.strip()
        if not arg:
            print(f"  {_yellow('Usage')}: obj <name|#idx>")
            return
        robot.obj(arg)
    reg(_m_obj, magic_name='obj')

    def _m_objpos(line):
        """objpos <name|#idx> x y z"""
        parts = line.split()
        if len(parts) < 4:
            print(f"  {_yellow('Usage')}: objpos <name|#idx> x y z")
            return
        robot.objpos(parts[0], float(parts[1]), float(parts[2]), float(parts[3]))
    reg(_m_objpos, magic_name='objpos')

    def _m_objrot(line):
        """objrot <name|#idx> rx ry rz"""
        parts = line.split()
        if len(parts) < 4:
            print(f"  {_yellow('Usage')}: objrot <name|#idx> rx ry rz")
            return
        robot.objrot(parts[0], float(parts[1]), float(parts[2]), float(parts[3]))
    reg(_m_objrot, magic_name='objrot')

    def _m_objscale(line):
        """objscale <name|#idx> s [sy sz]"""
        parts = line.split()
        if len(parts) < 2:
            print(f"  {_yellow('Usage')}: objscale <name|#idx> s  or  <name|#idx> sx sy sz")
            return
        robot.objscale(parts[0], *[float(x) for x in parts[1:]])
    reg(_m_objscale, magic_name='objscale')

    def _m_objvis(line):
        """objvis <name|#idx> on|off"""
        parts = line.split()
        if len(parts) < 2 or parts[1].lower() not in ("on", "off"):
            print(f"  {_yellow('Usage')}: objvis <name|#idx> on|off")
            return
        robot.objvis(parts[0], parts[1].lower() == "on")
    reg(_m_objvis, magic_name='objvis')

    def _m_objresetrot(line):
        """objresetrot <name|#idx>"""
        arg = line.strip()
        if not arg:
            print(f"  {_yellow('Usage')}: objresetrot <name|#idx>")
            return
        robot.objresetrot(arg)
    reg(_m_objresetrot, magic_name='objresetrot')

    def _m_objresetscale(line):
        """objresetscale <name|#idx>"""
        arg = line.strip()
        if not arg:
            print(f"  {_yellow('Usage')}: objresetscale <name|#idx>")
            return
        robot.objresetscale(arg)
    reg(_m_objresetscale, magic_name='objresetscale')

    # ── Device / session commands ────────────────────────────────────

    def _m_devices(line):
        robot.devices()
    reg(_m_devices, magic_name='devices')

    def _m_device(line):
        """device <name>"""
        arg = line.strip()
        if not arg:
            print(f"  {_yellow('Usage')}: device <name>")
            print(f"  Type {_bold('devices')} to list available devices.")
            return
        robot.device(arg)
    reg(_m_device, magic_name='device')

    def _m_session(line):
        """session [id]"""
        arg = line.strip()
        if not arg:
            robot.session()
        else:
            robot.session(arg)
    reg(_m_session, magic_name='session')

    def _m_sessions(line):
        robot.sessions()
    reg(_m_sessions, magic_name='sessions')

    # ── Plan ─────────────────────────────────────────────────────────

    def _m_plan(line):
        """plan --start <axes> --end <axes> [--stepsize d] [--steptime ms]"""
        raw = line.split()
        if not raw:
            names_eg = ' '.join(
                f'{n.split()[0].lower()}=0' for _, n in robot._movable_joints[:3]
            )
            print(f"  {_yellow('Usage')}: plan "
                  f"{_cyan('--start')} {_cyan('<axes>')} "
                  f"{_cyan('--end')} {_cyan('<axes>')} "
                  f"[{_cyan('--stepsize')} {_cyan('<deg>')}] "
                  f"[{_cyan('--steptime')} {_cyan('<ms>')}]")
            print(f"  Positional: {_dim(' '.join(['0'] * robot._n_movable))}")
            print(f"  Named:      {_dim(names_eg + ' ...')}")
            return

        def _get_flag(flag, default=None):
            nonlocal raw
            if flag in raw:
                i = raw.index(flag)
                val = raw[i + 1]
                raw = raw[:i] + raw[i+2:]
                return val
            return default

        def _get_angle_list(flag):
            nonlocal raw
            if flag not in raw:
                return None
            i = raw.index(flag)
            tokens = []
            j = i + 1
            while j < len(raw) and not raw[j].startswith('--'):
                tokens.append(raw[j])
                j += 1
            raw = raw[:i] + raw[j:]
            if not tokens:
                return None
            if any('=' in t for t in tokens):
                vals = [0.0] * robot._n_movable
                for t in tokens:
                    if '=' not in t:
                        raise ValueError(f"Mix of positional and named axes: {t!r}")
                    name, val = t.split('=', 1)
                    idx = robot._resolve_axis_name(name.strip())
                    if idx is None:
                        names = ', '.join(n for _, n in robot._movable_joints)
                        raise ValueError(f"Unknown axis {name!r}. Available: {names}")
                    vals[idx] = float(val)
                return vals
            else:
                return [float(t) for t in tokens]

        try:
            stepsize_str = _get_flag('--stepsize')
            steptime_str = _get_flag('--steptime')
            start_vals = _get_angle_list('--start')
            end_vals = _get_angle_list('--end')
        except ValueError as e:
            print(f"  {_bred('Error')}: {e}")
            return

        if start_vals is None or end_vals is None:
            print(f"  {_yellow('Error')}: --start and --end are required")
            return

        stepsize = float(stepsize_str) if stepsize_str is not None else 5.0
        steptime = int(steptime_str) if steptime_str is not None else 80
        robot.plan(start_vals, end_vals, stepsize=stepsize, steptime=steptime)
    reg(_m_plan, magic_name='plan')

    # ── Scan ─────────────────────────────────────────────────────────

    def _m_scan(line):
        """scan <axis> <start> <end> <step> [<axis> ...] [--steptime ms] [--space local|world]
        scan <axis> [<axis> ...] func_or_expr() [--steptime ms] [--space local|world]
        scan <device> array_var [--steptime ms]

        Kappa virtual axes use a 'v:' prefix, e.g.:
            scan v:chi 0 90 5
            scan v:chi 0 90 5 v:theta 0 2

        Cartesian end-effector axes use an 'ee:' prefix (x,y,z mm; a,b,g deg),
        solved with the Python IK. --space picks the frame (default local).
        Prefix the axis with a device name (Device:ee:<axis>) to target a
        specific device or combine several in one scan:
            scan ee:x 150 250 10
            scan ee:x 150 250 5 ee:y -50 50 5
            scan ee:z 200 400 10 --space world
            scan GP180_120:ee:z 200 400 10
            scan GP180_120:ee:z 200 400 10 Meca500:ee:x 150 250 10
            scan GP180_120:ee:y 354 400 10 I16_diff:delta 0 120 10   # Cartesian + joint

        Vector scan with device name:
            scan GP180_120 scanpoints
            scan GP180_120 Meca500 combined_pts
            scan robot1 robot2 my_func()
        """
        raw = line.split()
        if not raw:
            robot.scan()
            return

        steptime = 80
        if '--steptime' in raw:
            i = raw.index('--steptime')
            steptime = int(raw[i + 1])
            raw = raw[:i] + raw[i+2:]

        space = 'local'
        if '--space' in raw:
            i = raw.index('--space')
            space = raw[i + 1]
            raw = raw[:i] + raw[i+2:]

        def _is_number(s):
            try:
                float(s)
                return True
            except ValueError:
                return False

        # Detect array scan: last token ends with "()" or "(args)"
        # e.g. "scan delta gamma polar_func()" or "scan delta gamma my_func(10)"
        last_token = raw[-1] if raw else ""
        paren_idx = last_token.find("(")
        if paren_idx > 0 and last_token.endswith(")"):
            # Array scan mode: tokens before the func call are axis names
            axis_names = raw[:-1]
            func_expr = last_token
            if len(axis_names) < 1:
                print(f"  {_yellow('Error')}: array scan needs at least one axis name before the function")
                return
            # Evaluate the function expression in the IPython namespace
            from IPython import get_ipython
            ip = get_ipython()
            try:
                data = ip.ev(func_expr)
            except Exception as e:
                print(f"  {_bred('Error')}: failed to evaluate {func_expr!r}: {e}")
                return
            robot.scan(*axis_names, data, steptime=steptime, space=space)
            return

        # Detect vector/array scan: last token is a variable that evaluates
        # to an array-like or callable, preceding tokens are axis or device names.
        # e.g. "scan GP180_120 scanpoints" or "scan J1 J2 my_array"
        if len(raw) >= 2 and not _is_number(raw[-1]):
            from IPython import get_ipython
            ip = get_ipython()
            try:
                data = ip.ev(raw[-1])
                if _is_array_like(data) or callable(data):
                    axis_names = raw[:-1]
                    robot.scan(*axis_names, data, steptime=steptime, space=space)
                    return
            except Exception:
                pass

        if len(raw) < 4:
            robot.scan()
            return

        groups = []
        i = 0
        while i < len(raw):
            axis_name = raw[i]
            i += 1
            nums = []
            while i < len(raw) and _is_number(raw[i]):
                nums.append(float(raw[i]))
                i += 1
            groups.append(tuple([axis_name] + nums))

        robot.scan(*groups, steptime=steptime, space=space)
    reg(_m_scan, magic_name='scan')

    # ── Help ─────────────────────────────────────────────────────────

    def _m_rhelp(line):
        robot.help()
    reg(_m_rhelp, magic_name='rhelp')


# ── Entry point ──────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Robot/Device IPython Remote Control Terminal",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="Examples:\n"
               "  python robot_ipython.py\n"
               "  python robot_ipython.py --config i16_config.json\n"
               "  python robot_ipython.py --url ws://192.168.1.100:8080/ws --config meca500_config.json\n"
               "  python robot_ipython.py --session ab12cd34\n",
    )
    parser.add_argument("--url", default="ws://localhost:8080/ws",
                        help="WebSocket URL (default: ws://localhost:8080/ws)")
    parser.add_argument("--config", default="meca500_config.json",
                        help="Path to device config JSON (default: meca500_config.json)")
    parser.add_argument("--session", default=None,
                        help="Session ID of the viewer instance to connect to.")
    args = parser.parse_args()

    url = args.url
    if args.session:
        sep = "&" if "?" in url else "?"
        url = f"{url}{sep}session={args.session}"

    robot = RobotClient(url=url, config=args.config)

    banner = _build_banner(robot)

    import IPython

    from traitlets.config import Config

    c = Config()
    c.TerminalInteractiveShell.banner1 = banner
    c.TerminalInteractiveShell.banner2 = ""
    c.TerminalInteractiveShell.confirm_exit = False
    c.TerminalInteractiveShell.prompts_class = _RobotPrompts
    c.InteractiveShellApp.exec_lines = [
        "_register_magics(get_ipython(), robot)",
    ]

    IPython.start_ipython(
        argv=[],
        user_ns={
            "robot": robot,
            "r": robot,
            "time": time,
            "np": np,
            "kinematics": kinematics,
            "Meca500_kin": Meca500_kin,
            "GP225_kin": GP225_kin,
            "GP180_120_kin": GP180_120_kin,
            "GP280_kin": GP280_kin,
            "MotoMini_kin": MotoMini_kin,
            "_register_magics": _register_magics,
        },
        config=c,
    )

    robot.disconnect()


if __name__ == "__main__":
    main()
