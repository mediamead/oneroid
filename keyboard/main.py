#!/usr/bin/env python

import time
from KBHit import KBHit
from manipulator import Manipulator
from scenario import Scenario

print("qa/ws/ed/rf/tg/yh/uj/ik - движение точки")
print("n - перейти к следующему сценарию")
print("V - сохранить сценарий")
print("L - перезагрузить сценарий")
# print("m - измерить время исполнения и подогнать задержки")
print("x - прогнать все точки")
print("? - показать точки текущего сценария")

def handle_keyboard(ch):
    # gotopos
    if ch >= '0' and ch <= '9':
        return ('goto', ord(ch) - ord('0'))
    elif ch == '?':
        return ('dump', None)
    elif ch == 'n':
        return ('next-scenario', None)
    elif ch == 'V':
        return ('write-scenario', None)
    elif ch == 'L':
        return ('reload-scenario', None)
    # elif ch == 'm':
    #     return ('measure-scenario', None)
    elif ch == 'x':
        return ('execute-scenario', None)
    elif ch == ' ':
        return ('grbl-status', None)

    if ch.isupper():
        step = 10
        ch = ch.lower()
    else:
        step = 1
    # 1
    if ch == 'q':
        axis = 0
        delta = step
    elif ch == 'a':
        axis = 0
        delta = -step
    elif ch == 'w':
        axis = 1
        delta = step
    elif ch == 's':
        axis = 1
        delta = -step
    # 2
    elif ch == 'e':
        axis = 2
        delta = step
    elif ch == 'd':
        axis = 2
        delta = -step
    elif ch == 'r':
        axis = 3
        delta = step
    elif ch == 'f':
        axis = 3
        delta = -step
    # 3
    elif ch == 't':
        axis = 4
        delta = step
    elif ch == 'g':
        axis = 4
        delta = -step
    elif ch == 'y':
        axis = 5
        delta = step
    elif ch == 'h':
        axis = 5
        delta = -step
    # 4
    elif ch == 'u':
        axis = 6
        delta = step
    elif ch == 'j':
        axis = 6
        delta = -step
    elif ch == 'i':
        axis = 7
        delta = step
    elif ch == 'k':
        axis = 7
        delta = -step
    else:
        print("#KBHIT %d" % ord(ch))
        return (None, None)

    # fall-through from axis movement commands
    return ('move', [axis, delta])

# def measure(scenario, m):
#     m.move(scenario.get_first_pos())
#     for pos in scenario.positions[1:]:
#         pos.timing1 = m.measure1(pos.pos)
#     m.move(scenario.get_first_pos())
#     for pos in scenario.positions[1:]:
#         pos.timing2 = m.measure2(pos.pos)
#     m.move(scenario.get_first_pos())

kb = KBHit()

m = Manipulator(homing=False, dry_run=False)
scenarios = [Scenario('scenario-1.csv'), Scenario('scenario-2.csv'), Scenario('scenario-3.csv')]
curr_scenario = 0

scenario = scenarios[curr_scenario]
pos = scenario.get_first_pos()
m.move(pos.pos)

while True:
    print("# %s line %d %s" % (scenario.file, pos.nline, pos.pos))

    while not kb.kbhit():
        time.sleep(0.1)

    ch = kb.getch()
    cmd, cmd_params = handle_keyboard(ch)
    if cmd == 'move':
        if cmd_params:
            pos.apply_delta(cmd_params[0], cmd_params[1])
            m.move(pos.pos)
    elif cmd == 'dump': # dump scenario
        scenario.print()
    elif cmd == 'goto':
        pos = scenario.get_pos(cmd_params)
        m.move(pos.pos)
    elif cmd == 'next-scenario':
        curr_scenario = (curr_scenario + 1) % len(scenarios)
        scenario = scenarios[curr_scenario]
        scenario.print()
        pos = scenario.get_first_pos()
        m.move(pos.pos)
    elif cmd == 'write-scenario':
        scenario.save()
    # elif cmd == 'measure-scenario':
    #     measure(scenario)
    elif cmd == 'execute-scenario':
        for pos in scenario.positions:
            m.move(pos.pos)
    elif cmd == 'reload-scenario':
        scenario.load()
        scenario.print()
        pos = scenario.get_first_pos()
        m.move(pos.pos)
    elif cmd == 'grbl-status':
        m.grbl_status()
