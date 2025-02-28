import board
import digitalio
import neopixel
import usb_hid
from hid_gamepad import Gamepad


def printStatus():
    print(
        "\nReef Level: "
        + reefLevels[reefLevel]
        + "\nReef Side: "
        + reefSides[reefSide]
        + "\nMode: "
        + gpModes[gpMode]
    )


gp = Gamepad(usb_hid.devices)

# Create some buttons. The physical buttons are connected
# to ground on one side and these and these pins on the other.
# print(dir(board))

led = neopixel.NeoPixel(board.NEOPIXEL, 12)
led.brightness = 1

button_pins = (
    board.KEY1,
    board.KEY2,
    board.KEY3,
    board.KEY4,
    board.KEY5,
    board.KEY6,
    board.KEY7,
    board.KEY8,
    board.KEY9,
    board.KEY10,
    board.KEY11,
    board.KEY12,
)

# Map the buttons to button numbers on the Gamepad.
# gamepad_buttons[i] will send that button number when buttons[i]
# is pushed.
gamepad_buttons = (1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12)

buttons = [digitalio.DigitalInOut(pin) for pin in button_pins]
for button in buttons:
    button.direction = digitalio.Direction.INPUT
    button.pull = digitalio.Pull.UP

statusArray = [True, True, True, True, True, True, True, True, True, True, True, True]
algaeMode = False

reefLevels = ["L1", "L2", "L3", "L4"]
reefSides = ["Left", "Right"]
gpModes = ["Coral", "Algae"]
reefLevel = 0
reefSide = 0
gpMode = 0

gp.press_buttons(7)
led[6] = (0, 80, 255)
led[11] = (255, 0, 100)
printStatus()

while True:
    # Buttons are grounded when pressed (.value = False).
    for i, button in enumerate(buttons):
        gamepad_button_num = gamepad_buttons[i]

        if (
            (i == 2 or i == 5 or i == 8 or i == 11)
            and not button.value
            and statusArray[i]
        ):
            statusArray[i] = False
            for j in range(2, 12, 3):
                if j != i:
                    led[j] = (0, 0, 0)
                    gp.release_buttons(gamepad_buttons[j])
                else:
                    led[i] = (255, 0, 100)
                    if i == 2:
                        reefLevel = i + 1
                    if i == 5:
                        reefLevel = i - 3
                    if i == 8:
                        reefLevel = i - 7
                    if i == 11:
                        reefLevel = i - 11
            gp.press_buttons(gamepad_button_num)

        if (
            (i == 2 or i == 5 or i == 8 or i == 11)
            and button.value
            and not statusArray[i]
        ):
            statusArray[i] = True
            printStatus()

        if 6 <= i <= 7 and not button.value and statusArray[i]:
            statusArray[i] = False
            for j in range(6, 8):
                if j != i:
                    led[j] = (0, 0, 0)
                    gp.release_buttons(gamepad_buttons[j])
                else:
                    led[i] = (0, 80, 255)
                reefSide = i - 6
            gp.press_buttons(gamepad_button_num)

        if 6 <= i <= 7 and button.value and not statusArray[i]:
            statusArray[i] = True
            printStatus()

        if i == 9 and not button.value and statusArray[i]:
            statusArray[i] = False
            algaeMode = not algaeMode
            if algaeMode:
                led[i] = (255, 0, 0)
                gp.press_buttons(i + 1)
                gpMode = 1
            else:
                led[i] = (0, 0, 0)
                gp.release_buttons(i + 1)
                gpMode = 0

        elif i == 9 and button.value and not statusArray[i]:
            statusArray[i] = True
            printStatus()

        if i == 10 and not button.value and statusArray[i]:
            statusArray[i] = False
            led[i] = (255, 150, 0)
            gp.press_buttons(i + 1)
        elif i == 10 and button.value and not statusArray[i]:
            statusArray[i] = True
            gp.release_buttons(i + 1)
            led[i] = (0, 0, 0)
