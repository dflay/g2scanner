import io
import time
import threading

import cv2

import picamera
import picamera.array

import numpy as np
from math import sqrt

####################
# HELPER FUNCTIONS #
####################

# for retrieving the time (in seconds) since 
# the script was executed 
INIT_TIME = time.time()
def systime():
    return time.time() - INIT_TIME

#############
# CONSTANTS #
#############

# if true, when G2Scanner.configure() is called, 
# output image with lines showing bin locations.
OUTPUT_BIN_IMAGE = False

# POSSIBLE VALUES (0,1,2,3)
DEBUG_LEVEL = 1

# number of concurrent threads to use
# 4 is recommended for maximum speed on the Raspberry Pi B+
NUM_THREADS = 4

# number of barcodes to scan before terminating the script
NUM_SCANS = 30

####################################
# THREAD-SHARED (GLOBAL) VARIABLES #
####################################

# scanning stops whenever done set to True
done = False

# locks are used for atomicity within the ImageProcessor threads
# see standard library python 'threading' module 
lock = threading.Lock()

# list that holds the ImageProcessor threads
# It grows/shrinks as threads are called to scan 
# the barcode images
pool = []

# when an ImageProcessor begins to scan/process an
# image, the current time is added to this list.
image_times = []

# always contains the latest scanned/decoded bit pattern
current_bit_pattern = []

# the number of scans taken since the script was excuted
taken = 0

###########
# CLASSES #
###########

class ImageProcessor(threading.Thread):
    def __init__(self, camera):

        # 'super' runs thread.Thread __init__ as well
        super(ImageProcessor, self).__init__()
        
        if DEBUG_LEVEL >= 2: 
            print(str(self.name) + " initialized at " + str(systime()) + " sec.")

        self.stream = picamera.array.PiRGBArray(camera)
        self.event = threading.Event()
        self.terminated = False
        self.start()

    def run(self):
        # This method runs in a separate thread
        global done
        global pool
        global lock
        global image_times
        global taken
        global current_bit_pattern

        

        while not self.terminated:
            # Wait for an image to be written to the stream
            # print("waiting...") TEST THIS!!!!
            if self.event.wait(1):
                # Now an image has been taken by camera and loaded into the 
                # PiRGBArray, now ready for processing.
                try:
                    with lock: 
                        image_times.append(systime())
                        self.scan_number = taken
                        taken += 1
                        if taken >= NUM_SCANS:
                            done=True

                    # reset thread-specific internal variables
                    self.bin_luminosity = []
                    self.bin_pixels = []
                    self.bit_pattern = []
                    self.scan_column = 8

                    self._calibrate()

                    ROWS = len(self.stream.array)
                    COLS = len(self.stream.array[0])

                    # fill bit_pattern array
                    for i in range(len(self.bin_pixels)-1):
                        b,g,r = np.average(self.stream.array[self.bin_pixels[i]:self.bin_pixels[i+1]][self.scan_column],0)
                        if ( (r*299 + g*587 + b*114) / 1000.0 < 95 ):
                            self.bit_pattern.append(1)
                        else: self.bit_pattern.append(0)
                    #with lock: current_bit_pattern = self.bit_pattern
                    print self.bit_pattern

                # scan/processing is now complete
                finally:
                    if DEBUG_LEVEL >= 3:
                        self.print_diagnostics()
                    self._rewind() 
                           

    def _rewind(self):
        # Reset the stream and event
        self.stream.seek(0)
        self.stream.truncate()
        self.event.clear()

        # Return ourselves to the pool
        global pool
        with lock:
            pool.append(self)

    def _calibrate(self):
    # locate the top/bottom black bars of a bar/grey code,
        top_bar_pixel = None
        bottom_bar_pixel = None

        ROWS = len(self.stream.array)
        COLS = len(self.stream.array[0])

        # fill luminosity array
        for i in range(ROWS):
            b = self.stream.array[i][self.scan_column][0]
            g = self.stream.array[i][self.scan_column][1]
            r = self.stream.array[i][self.scan_column][2]
            # rgb_avg = np.average(self.stream.array[i][self.scan_column])
            rgb_avg = (r + b + g) / 3.0
            self.bin_luminosity.append( rgb_avg )

        BLACK_BRIGHTNESS_MAX = 50.0
        for i in range(ROWS):
            if self.bin_luminosity[i] < BLACK_BRIGHTNESS_MAX:
                top_bar_pixel = i
                break

        for i in range(ROWS-1, -1, -1):
            if self.bin_luminosity[i] < BLACK_BRIGHTNESS_MAX:
                bottom_bar_pixel = i
                break

        # TODO: exit if top or bottom = 0 as well (or any known-wrong value)
        if (bottom_bar_pixel == None) or (top_bar_pixel == None):
            print("Couldn't locate top/bottom bar pixel(s)! Skipping this frame.")
            self.stream.close()
            self.close()
            exit()

        dp = abs((bottom_bar_pixel - top_bar_pixel) / 20.0)

        self.bin_pixels = np.arange(top_bar_pixel, bottom_bar_pixel + dp, dp).tolist()[0:21]

        for i in range(len(self.bin_pixels)):
            self.bin_pixels[i] = int(round(self.bin_pixels[i]))

        
        if OUTPUT_BIN_IMAGE:
            cv2.line(self.stream.array, (0,top_bar_pixel),(COLS, top_bar_pixel) ,(0,0,255),1)
            cv2.line(self.stream.array, (0,bottom_bar_pixel),(COLS, bottom_bar_pixel) ,(0,255,0),1)
            for i in self.bin_pixels[1:-1]:
                cv2.line(self.stream.array, (0,i), (COLS,i), (255,0,0), 1)
            cv2.line(self.stream.array, (self.scan_column,0),(self.scan_column, ROWS) ,(255,255,255),1)

            with lock:
                cv2.imwrite(str(self.scan_number) + ".bmp", self.stream.array)

    def print_diagnostics(self):
        global lock
        with lock:
            print("#### START THREAD DIAGNOSTICS ####")
            print("Name: " + str(self.name))
            print("Scan Number: " + str(self.scan_number))
            print("Scan Column: " + str(self.scan_column))
            print("Bin Pixels: " + str(self.bin_pixels))
            print("#### END THREAD DIAGNOSTICS ####")


class G2Scanner(object):
    def __init__(self):

        try:
            self.camera = picamera.PiCamera()
            self.camera.resolution = (16,300)
            # self.camera.contrast = -5
            # self.camera.brightness = 51
            # self.camera.color_effects=(128,128)
            self.camera.framerate = 30
            self.camera.shutter_speed = 15000
            self.camera.video_stabilization = False
            self.camera.video_denoise = False

            if DEBUG_LEVEL >= 2:
                print("Camera initalized at " + str(systime()) + " sec.\n")

            if DEBUG_LEVEL >= 1:
                print("#### START CAMERA PROPERTIES ####")
                print("CAMERA RESOLUTION (width, height): " + str(self.camera.resolution))
                print("CAMERA FRAMERATE: " + str(self.camera.framerate) + " Hz")
                print("CAMERA SHUTTER SPEED: " + str(self.camera.shutter_speed) + " us")
                print("CAMERA VIDEO STABILIZATION: " + str(self.camera.video_stabilization))
                print("CAMERA VIDEO DENOISE: " + str(self.camera.video_denoise))
                print("CAMERA SENSOR MODE: " + str(self.camera.sensor_mode))
                print("#### END CAMERA PROPERTIES ####\n")

            # let camera 'warm up' (IMPORTANT)
            if DEBUG_LEVEL >= 2: print("Warming up camera for 2 seconds...")
            time.sleep(2)
            if DEBUG_LEVEL >= 2: print("Finished warming up camera.\n")

        except:
            print "G2Scanner: failed to initialize raspberry pi camera"

        # initialize the pool of threads
        if DEBUG_LEVEL >= 2: 
            print("#### START THREAD INITILIZATION ####")
        global pool 
        pool = [ImageProcessor(self.camera) for i in range(NUM_THREADS)]
        if DEBUG_LEVEL >= 2: 
            print("#### END THREAD INITILIZATION ####\n")

    def streams(self):
        global done
        global lock
        global pool

        while not done:
            with lock:
                if pool:
                    processor = pool.pop()
                else:
                    processor = None
            if processor:
                yield processor.stream
                processor.event.set()
            else:
                # When the pool is starved, wait a bit for it to refill
                time.sleep(0.005)

    def scan(self): 
        self.camera.capture_sequence( self.streams(), use_video_port=True, format='bgr')

    def close(self):
        self.camera.close()

        # wait for all ImageProcessor threads to finish processing
        while len(pool) != NUM_THREADS:
            time.sleep(0.1)

        # Shut down the processors in an orderly fashion
        while pool:
            with lock:
                processor = pool.pop()
            processor.stream.close()
            processor.terminated = True
            processor.join()

        global taken
        if DEBUG_LEVEL >= 1 and NUM_SCANS > 1:
            global image_times
            t0 = image_times[0]
            for i in range(taken):
                image_times[i] -= t0
            print("AVERAGE SCAN RATE: " + str(taken / image_times[-1]) + " Hz")

g2 = G2Scanner()
g2.scan()
g2.close()
