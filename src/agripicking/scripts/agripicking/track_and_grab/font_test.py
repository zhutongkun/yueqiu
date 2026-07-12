from enum import Enum
class EchoColor(Enum):
    RED = 1
    GREEN = 2
    BLUE = 3

def echo_str(str,font_color):
    if font_color == EchoColor.BLUE:
        # blue
        print(f"\033[34m{str}\033[0m")
    elif font_color == EchoColor.RED:
        # red
        print(f"\033[31m\033[42m{str}\033[0m")
    elif font_color == EchoColor.GREEN:
        # green
        print(f"\033[32m\033[42m{str}\033[0m")


echo_str('for math ',EchoColor.GREEN)