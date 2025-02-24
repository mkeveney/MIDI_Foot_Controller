Import("env")

board_config = env.BoardConfig()
#
# I discovered by experimentation that I had to change the USB VID/PID
# in order to have the MIDI name reflected in the OS and WebMIDI.
# Not sure why.
#
# I'm re-using vendor id that came with the board, (belonging to Seeed studio)
# and simply hoping that my chosen PIDs will never conflict.  A better
# solution would be to get my own VID/PID pair
#
board_config.update("build.hwids", [
   ["0x2886", "0xFFFF"]
])
board_config.update("build.usb_product", "MIDI Foot Controller")

