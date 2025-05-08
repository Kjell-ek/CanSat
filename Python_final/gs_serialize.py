#!/usr/bin/env python3

"""Data serialization for Arduino Uno - PC Interface"""
# Requirements
# python-numpy
# python-matplotlib
# python-pandas
# python-pyserial
# python-reportlab ???
# python-capng ???

from time import sleep, perf_counter
from serial_interface import SerialInterface
import sys

# types of payload, see types.h in arduino code
t_launch = 0
t_orbit = 1
t_accmag = 2
t_outofrange = 3
t_crash = 4
t_received = 5 # not a payload type
# also defined in types.h in arduino code
invalid_data = -32000

# write csv line
def wl(resp, keys, extra=""):
    return ";".join([str(resp[k]) for k in keys]) + extra + "\n"

if __name__ == "__main__":
    # Example usage
    if len(sys.argv) != 2:
        print("Usage: python gs_serialize.py <port>")
        sys.exit(1)

    iface = SerialInterface(port=sys.argv[1], baud=115200)

    while True:
        response = iface.read_msg()  # Read the response
        try:
            if response is None:
                continue
            if not isinstance(response, dict):
                continue

            print(response)

            with open("all.json", "a") as myfile:
                myfile.write(str(response) + "\n")

            if response["type"] == t_launch:
                with open("time_gas_temp.csv", "a") as myfile:
                    # we removed celcius from launch payload, so we add 0 to make it 3 columns
                    myfile.write(wl(response, ["time", "gas"], ";0"))
                with open("time_temp_hum.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "tah_celcius", "tah_humidity"]))
                with open("time_dis_dis_pir.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "us_distance", "us_distance2", "pir"]))

            if response["type"] == t_orbit:
                with open("time_gas_temp.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "gas", "celcius"]))
                with open("time_temp_hum.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "tah_celcius", "tah_humidity"]))

            if response["type"] == t_accmag:
                with open("time_accx_accy_accz.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "acc_x", "acc_y", "acc_z"]))
                    myfile.write(wl(response, ["time2", "acc_x2", "acc_y2", "acc_z2"]))
                with open("time_vx_vy_vz.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "vx", "vy", "vz"]))
                    myfile.write(wl(response, ["time2", "vx2", "vy2", "vz2"]))
                with open("time_x_y_z.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "x", "y", "z"]))
                    myfile.write(wl(response, ["time2", "x2", "y2", "z2"]))
                with open("time_magx_magy_magz.csv", "a") as myfile:
                    if response["mag_x"]!=invalid_data:
                        myfile.write(wl(response, ["time", "mag_x", "mag_y", "mag_z"]))
                    if response["mag_x2"]!=invalid_data:
                        myfile.write(wl(response, ["time2", "mag_x2", "mag_y2", "mag_z2"]))
                with open("time_headmag_headtrue.csv", "a") as myfile:
                    if response["headingMagnetic"]!=invalid_data:
                        myfile.write(wl(response, ["time", "headingMagnetic", "headingTrue"]))
                    if response["headingMagnetic2"]!=invalid_data:
                        myfile.write(wl(response, ["time2", "headingMagnetic2", "headingTrue2"]))

            if response["type"] == t_outofrange:
                with open("time_gas_temp.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "gas", "celcius"]))
                with open("time_accx_accy_accz.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "acc_x", "acc_y", "acc_z"]))
                with open("time_vx_vy_vz.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "vx", "vy", "vz"]))
                with open("time_x_y_z.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "x", "y", "z"]))

            if response["type"] == t_crash:
                with open("time_accx_accy_accz.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "acc_x", "acc_y", "acc_z"]))
                with open("time_vx_vy_vz.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "vx", "vy", "vz"]))
                with open("time_x_y_z.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "x", "y", "z"]))
            if response["type"] == 5:
                with open("receive_time_counter.csv", "a") as myfile:
                    myfile.write(wl(response, ["time", "counter"]))

            print("---")
            sleep(0.3) # 1 second
        except Exception as e:
            with open("error.csv", "a") as myfile:
                myfile.write(type(e).__name__ + ": " + str(e) + "\n")
                myfile.write(";".join([k + ":" + str(response[k]) for k in response]) + "\n")
                myfile.write("\n")
    iface.close()
