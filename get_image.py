import argparse
import time
import socket
import struct
import multiprocessing as mp
import cflib
import os
import numpy as np
from datetime import datetime
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
import cv2

# Crazyflie URI
URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Flight parameters
DEFAULT_HEIGHT = 0.5
BOX_LIMIT = 0.5

# Initialize the low-level drivers (don't list the debug drivers)
cflib.crtp.init_drivers(enable_debug_driver=False)

# Function to handle image streaming
def image_streaming():
    # Args for setting IP/port of AI-deck. Default settings are for when
    # AI-deck is in AP mode.
    parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
    parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
    parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
    parser.add_argument('--save', action='store_true', help="Save streamed images")
    args = parser.parse_args()

    deck_port = args.p
    deck_ip = args.n

    print("Connecting to socket on {}:{}...".format(deck_ip, deck_port))
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((deck_ip, deck_port))
    print("Socket connected")

    imgdata = None
    data_buffer = bytearray()

    def rx_bytes(size):
        data = bytearray()
        while len(data) < size:
            data.extend(client_socket.recv(size-len(data)))
        return data

    import cv2

    start = time.time()
    count = 0

    while (1):
        # First get the info
        packetInfoRaw = rx_bytes(4)
        #print(packetInfoRaw)
        [length, routing, function] = struct.unpack('<HBB', packetInfoRaw)
        #print("Length is {}".format(length))
        #print("Route is 0x{:02X}->0x{:02X}".format(routing & 0xF, routing >> 4))
        #print("Function is 0x{:02X}".format(function))

        imgHeader = rx_bytes(length - 2)
        #print(imgHeader)
        #print("Length of data is {}".format(len(imgHeader)))
        [magic, width, height, depth, format, size] = struct.unpack('<BHHBBI', imgHeader)

        if magic == 0xBC:
            #print("Magic is good")
            #print("Resolution is {}x{} with depth of {} byte(s)".format(width, height, depth))
            #print("Image format is {}".format(format))
            #print("Image size is {} bytes".format(size))

            # Now we start rx the image, this will be split up in packages of some size
            imgStream = bytearray()

            while len(imgStream) < size:
                packetInfoRaw = rx_bytes(4)
                [length, dst, src] = struct.unpack('<HBB', packetInfoRaw)
                #print("Chunk size is {} ({:02X}->{:02X})".format(length, src, dst))
                chunk = rx_bytes(length - 2)
                imgStream.extend(chunk)
            
            count = count + 1
            meanTimePerImage = (time.time()-start) / count
            print("{}".format(meanTimePerImage))
            print("{}".format(1/meanTimePerImage))

            if format == 0:
                bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
                bayer_img.shape = (244, 324)
                color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                cv2.imwrite(f"stream_out/raw/img_{count:06d}.png", bayer_img)
            else:
                with open("img.jpeg", "wb") as f:
                    f.write(imgStream)
                nparr = np.frombuffer(imgStream, np.uint8)
                decoded = cv2.imdecode(nparr,cv2.IMREAD_UNCHANGED)
                cv2.imshow('JPEG', decoded)

# Function to handle flight control
def flight_control():
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        scf.cf.platform.send_arming_request(True)
        time.sleep(1.0)
        with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
            mc.forward(1)
            time.sleep(1)
            mc.up(0.5)
            time.sleep(1)
            mc.circle_right(0.5,0.3,180)
            time.sleep(1)
            mc.down(0.5)
            time.sleep(1)
            mc.circle_right(0.5,0.3,45)
            time.sleep(1)
            mc.forward(0.5)
            time.sleep(1)
            mc.stop()

            #stop_event.set()

if __name__ == '__main__':
    #stop_event = threading.Event()
    # Create threads for image streaming and flight control
    #image_thread = threading.Thread(target=image_streaming, args=(stop_event,))
    #flight_thread = threading.Thread(target=flight_control, args=(stop_event,))
    image_process = mp.Process(target=image_streaming, daemon=True)
                                     
    # Start the threads
    image_process.start()
    flight_control()