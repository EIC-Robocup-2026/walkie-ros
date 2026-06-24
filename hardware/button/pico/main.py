import usb_hid
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keycode import Keycode
import board
import digitalio
import time

kbd = Keyboard(usb_hid.devices)

button = digitalio.DigitalInOut(board.GP21)
button.direction = digitalio.Direction.INPUT
button.pull = digitalio.Pull.UP # pull up to active low

was_pressed = False

while True:
    is_pressed = not button.value # Low = pressed
    
    if is_pressed and not was_pressed:
        kbd.press(Keycode.F16)
        was_pressed = True
        print("Button pressed — sending F16")
        
    elif not is_pressed and was_pressed:
        kbd.release(Keycode.F16)
        was_pressed = False
        
        print("Button released")
    time.sleep(0.1)
    



