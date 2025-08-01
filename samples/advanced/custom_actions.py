"""
RobotDog Custom Actions Example
=============================
Advanced example for defining and using custom actions with RobotDog.
"""
#!/usr/bin/env python3
from byte import RobotDog
import curses
import curses_utils
from time import sleep

# init robotdog
# ======================================
my_dog = RobotDog()

# my_dog.legs_move([[0]*8], immediately=True, speed=60)
# my_dog.head_move([[0]*3], immediately=True, speed=60)
# my_dog.tail_move([[0]], immediately=True, speed=60)
# my_dog.wait_all_done()

my_dog.head_move([[0, 0, -35]], immediately=True, speed=5)
my_dog.do_action('sit', speed=92)
my_dog.wait_all_done()

# global variables
# ======================================
leg_angles = [0.0]*8
head_angles = [0.0]*3
tail_angle = [0.0]*1
current_servo = "1"

OFFSET_STEP = (180 / 2000) * (20000 / 4095)  # actual precision of steering gear
MAX_OFF = 90

# get_current_angles()
# ======================================
def get_current_angles():
    global leg_angles, head_angles, tail_angle
    leg_angles = list.copy(my_dog.leg_current_angles)
    head_angles = list.copy(my_dog.head_current_angles)
    tail_angle = list.copy(my_dog.tail_current_angles)
    for i, x in enumerate(leg_angles):
        leg_angles[i] = round(x, 2)
    for i, x in enumerate(head_angles):
        head_angles[i] = round(x, 2)
    tail_angle[0] = round(tail_angle[0], 2)

# constrain(), constrain value range
# ======================================
def constrain(val, min_val, max_val):

    if val < min_val: return min_val
    if val > max_val: return max_val
    return val


#
# ============================================================================
def display_title(subpad, color_pair):
    title = "RobotDog Calibration"
    # tip1
    curses_utils.clear_line(subpad, 0, color_pair=color_pair)
    subpad.addstr(0, int((curses_utils.PAD_X-len(title))/2), title, color_pair)

tip1 = [
    "Press key to select servo:",
    "1 ~ 8 : Leg servos",
    "9 : Head yaw    ",
    "0 : Head roll   ",
    "- : Head pitch  ",
    "= : Tail         ",
]
def display_tip1(subpad, color_pair):
    subpad.addstr(0, 0, tip1[0], color_pair | curses.A_BOLD | curses.A_REVERSE)
    for i in range(1, len(tip1)):
        subpad.addstr(i, 0, tip1[i], color_pair | curses.A_BOLD)

tip2 = [
    "Press key to adjust servo:",
    "W or A: increase angle ",
    "S or D: decreases angle",
]
def display_tip2(subpad, color_pair):
    subpad.addstr(0, 0, tip2[0], color_pair | curses.A_BOLD | curses.A_REVERSE)
    for i in range(1, len(tip2)):
        subpad.addstr(i, 0, tip2[i], color_pair | curses.A_BOLD)

body = [
    "        [9]        ",
    "   [-] ┌─┐  [0]  ",
    "       │ │       ",
    "[2][1]┌└─┘┐[3][4]",
    "      │   │      ",
    "      │   │      ",
    "[6][5]└─┬─┘[7][8]",
    "       [=]       ",
    "        /        ",
]
servo_pos = { # servo_pos in body
    "1": [3, 3], # ypos, xpos
    "2": [3, 0],
    "3": [3, 11],
    "4": [3, 14],
    "5": [6, 3],
    "6": [6, 0],
    "7": [6, 11],
    "8": [6, 14],
    "9": [0, 10],
    "0": [1, 12],
    "-": [1, 3],
    "=": [7, 7],
}
def display_dog_body(subpad, color_pair, color_pair_select):
    for i in range(len(body)):
        subpad.addstr(i, 0, body[i], color_pair | curses.A_BOLD)
    servo = current_servo
    if servo in servo_pos.keys():
        subpad.addstr(servo_pos[servo][0], servo_pos[servo][1], f"[{servo}]", color_pair_select | curses.A_BOLD)

tip3 = [
    "Ctrl+C: Quit    Space: Save",
]
def display_tip3(subpad, color_pair):
    curses_utils.clear_line(subpad, 0)
    subpad.addstr(0, 0, tip3[0], color_pair | curses.A_BOLD | curses.A_REVERSE)

def display_servo_num(subpad, color_pair, color_pair_select):
    subpad.addstr(0, 0, "Current Servo:", color_pair | curses.A_BOLD)
    subpad.addstr(0, 15, f"{current_servo}", color_pair_select | curses.A_BOLD)

def display_angles(subpad, color_pair, color_pair_select):
    servo_num = "1234567890-="
    comma = ""
    curses_utils.clear_line(subpad, 0)
    subpad.addstr(0, 0, f"leg_angles:", color_pair | curses.A_BOLD)
    for i, x in enumerate(leg_angles):
        comma = "," if i < len(leg_angles)-1 else ""
        if servo_num[i] == current_servo:
            subpad.addstr(f' {x:.2f}{comma}', color_pair_select | curses.A_BOLD)
        else:
            subpad.addstr(f' {x:.2f}{comma}', color_pair | curses.A_BOLD)
    # 
    curses_utils.clear_line(subpad, 1)
    subpad.addstr(1, 0, f"head_angles:", color_pair | curses.A_BOLD)
    for i, x in enumerate(head_angles):
        comma = "," if i < len(head_angles)-1 else ""
        if servo_num[i+8] == current_servo:
            subpad.addstr(f' {x:.2f}{comma}', color_pair_select | curses.A_BOLD)
        else:
            subpad.addstr(f' {x:.2f}{comma}', color_pair | curses.A_BOLD)
    #
    curses_utils.clear_line(subpad, 2)
    subpad.addstr(2, 0, f"tail_angle:", color_pair | curses.A_BOLD)
    for i, x in enumerate(tail_angle):
        if servo_num[i+11] == current_servo:
            subpad.addstr(f' {x:.2f}', color_pair_select | curses.A_BOLD)
        else:
            subpad.addstr(f' {x:.2f}', color_pair | curses.A_BOLD)


def main(stdscr):
    # global winlines, wincols
    global current_servo
    
    inc = 1  # 1 or -1, angle increase direction

    # winlines = curses.LINES
    # wincols = curses.LINES

    # reset screen
    stdscr.clear()
    stdscr.move(0, 0)
    stdscr.refresh()

    # disable cursor 
    curses.curs_set(0)

    # set colors
    curses.start_color()
    curses.use_default_colors()
    curses_utils.init_preset_color_pairs()

    # init pad    
    pad = curses.newpad(curses_utils.PAD_Y, curses_utils.PAD_X)
    # pad.box()

    # get the offset
    get_current_angles()

    # init subpad
    title_pad = pad.subpad(1, curses_utils.PAD_X, 0, 0)
    tip1_pad = pad.subpad(len(tip1), len(tip1[0]), 1, 0)
    # tip2_pad = pad.subpad(len(tip2), len(tip2[0]), 1, PAD_X-len(tip2[0])-1)
    tip2_pad = pad.subpad(len(tip2), len(tip2[0]), len(tip1)+2, 0)
    # body_pad = pad.subpad(len(body), len(body[0]), 2, int((PAD_X-len(body[0]))/2)-2)
    body_pad = pad.subpad(len(body), len(body[0]), 2, curses_utils.PAD_X-len(body[0])-20)
    if curses.COLS < curses_utils.PAD_X - 20:
        body_pad.move(2, 1)
    tip3_pad = pad.subpad(len(tip3), curses_utils.PAD_X, len(body)+3, 0)
    servo_num_pad = pad.subpad(1, curses_utils.PAD_X, len(body)+4, 0)
    offsets_pad = pad.subpad(3, curses_utils.PAD_X, len(body)+5, 0)

    display_title(title_pad, curses_utils.CYAN| curses.A_REVERSE)
    display_tip1(tip1_pad, curses_utils.WHITE)
    display_tip2(tip2_pad, curses_utils.WHITE)
    display_dog_body(body_pad, curses_utils.WHITE, curses_utils.CYAN)
    display_tip3(tip3_pad, curses_utils.WHITE)
    display_servo_num(servo_num_pad, curses_utils.WHITE, curses_utils.CYAN)
    display_angles(offsets_pad, curses_utils.WHITE, curses_utils.CYAN)

    curses_utils.pad_refresh(pad)

    stdscr.nodelay(True) # set non-blocking mode for getch()
    stdscr.timeout(50) # set timeout when no key is pressed

    while True:
        try:
            key = stdscr.getch()
            if key == curses.ERR: # if no key
                continue
            # ---- resize window ----
            if key == curses.KEY_RESIZE:
                curses_utils.pad_ypos = 0
                curses_utils.pad_xpos = 0
                curses.update_lines_cols()
                # if curses.COLS < PAD_X - 20:
                    # body_pad.refresh(2, curses.COLS-1-len(body[0]))
                # body_pad.erase() # ok
                curses_utils.pad_refresh(pad)
                sleep(0.5)
            # ---- select the servo ----
            elif chr(key) in ('1234567890-='):
                current_servo = chr(key)
                display_dog_body(body_pad, curses_utils.WHITE, curses_utils.CYAN)
                display_servo_num(servo_num_pad, curses_utils.WHITE, curses_utils.CYAN)
                display_angles(offsets_pad, curses_utils.WHITE, curses_utils.CYAN)
                curses_utils.clear_line(pad, 17)
                curses_utils.pad_refresh(pad)
            # ---- move ----
            elif chr(key) in ('wsadWSAD'):
                if chr(key) in ('wWdD'):
                    inc = 1
                else:
                    inc = -1
                # control legs
                if current_servo in ('12345678'):
                    # get index
                    index = ('12345678').index(current_servo)
                    # 
                    leg_angles[index] += inc*OFFSET_STEP
                    # move servos
                    my_dog.legs_simple_move(leg_angles)
                # control head
                elif current_servo in ('90-'):
                    index = ('90-').index(current_servo)
                    # 
                    head_angles[index] += inc*OFFSET_STEP
                    my_dog.head_move_raw([head_angles], True, 80)
                # control tail
                elif current_servo == '=':
                    tail_angle[0] += inc*OFFSET_STEP
                    my_dog.tail_move([tail_angle], True, 80) 
                # display offsets 
                display_angles(offsets_pad, curses_utils.WHITE, curses_utils.CYAN)
                curses_utils.clear_line(pad, 17)
                curses_utils.pad_refresh(pad)

        except KeyboardInterrupt:
            # ---- exit and remind to save calibration ----
            break


if __name__ == '__main__':
    try:
        curses.wrapper(main)
    except Exception as e:
        print(f"\033[31mERROR: {e}\033[m")
    finally:
        my_dog.close()
