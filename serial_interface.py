#!/usr/bin/env python3

"""Arduino - PC Interface for Data Transfer"""

import json
from time import sleep, time
from serial import Serial


class SerialInterface:
    """Creates a Serial Interface with the specified parameters and allows to read from
    and write to it."""
    def __init__(self, port="COM5", baud=9600):
        self.no_response = False
        self.ser = Serial(port, baudrate=baud)
        sleep(2)

    def read_msg(self):
        """Reads a line from the serial buffer,
        decodes it and returns its contents as a dict."""
        if self.ser.in_waiting == 0:
            # Nothing received
            self.no_response = True
            return None

        incoming = self.ser.readline().decode("utf-8")
        resp = None
        self.no_response = False

        try:
            resp = json.loads(incoming)
        except json.JSONDecodeError:
            print("\n")
            print("~~~Error decoding message~~~")
            print(incoming)
            print("~~~")
            print("\n")

        return resp

    def write_msg(self, message=None):
        """Sends a JSON-formatted command to the serial
        interface."""
        if self.no_response:
            # If no response was received last time, we don't send another request
            return

        try:
            json_msg = json.dumps(message)
            self.ser.write(json_msg.encode("utf-8"))
        except TypeError:
            print("Unable to serialize message.")

    def close(self):
        """Close the Serial connection."""
        self.ser.close()


if __name__ == "__main__":
    # Example usage
    iface = SerialInterface()
    msg = {"msg": "REQ"}  # Define your JSON message

    RESP_COUNT = 0

    while RESP_COUNT < 10:
        # Run until 10 messages have been received back
        iface.write_msg(msg)  # Send it
        response = iface.read_msg()  # Read the response
        if response is not None:
            print(f"Received: {response['msg']}")
            RESP_COUNT += 1

    iface.close()
