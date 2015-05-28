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
OUTPUT_BIN_IMAGE = True

# if true, when each ImageProcessor is done scanning,
# output a visualization of the scan.
OUTPUT_SCAN_IMAGES = False

# POSSIBLE VALUES (0,1,2,3)
DEBUG_LEVEL = 2

# number of concurrent threads to use
# 4 is recommended for maximum speed on the Raspberry Pi B+
NUM_THREADS = 1

# number of barcodes to scan before terminating the script
NUM_SCANS = 40

# determines which column to scan when decoding the bar code
SCAN_COLUMN = 8


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

# pixel locations of the start of each bin
bin_pixels = []

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
        global bin_pixels
        global current_bit_pattern
        global SCAN_COLUMN

        while not self.terminated:
            # Wait for an image to be written to the stream
            if self.event.wait(1):
                # An image has been taken by camera and loaded into the 
                # PiRGBArray, now ready for processing.
                try:
                    with lock: 
                        image_times.append(systime())
                        taken += 1
                        if taken >= NUM_SCANS:
                            done=True

                    luminosities = []
                    
                    ROWS = len(self.stream.array)
                    COLS = len(self.stream.array[0])
                    
                    # fill luminosity array
                    brightness = []
                    for i in range(len(bin_pixels)-1):
                        b,g,r = np.average(self.stream.array[bin_pixels[i]:bin_pixels[i+1],SCAN_COLUMN],0)
                        #print r+g+b
                        if (r+g+b < 175):
                            brightness.append(1)
                        else: brightness.append(0)
                    with lock: current_bit_pattern = brightness
                    #print current_bit_pattern

                # scan/processing is now complete
                finally:
                    # Reset the stream and event
                    self.stream.seek(0)
                    self.stream.truncate()
                    self.event.clear()
                    # Return ourselves to the pool
                    with lock:
                        pool.append(self)

class G2Scanner(object):
    def __init__(self):

        try:
            self.camera = picamera.PiCamera()
            self.camera.resolution = (16,300)
            self.camera.framerate = 30
            self.camera.shutter_speed = 10000
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
                time.sleep(0.01)

    def scan(self): 
        self._calibrate()
        self.camera.capture_sequence( self.streams(), use_video_port=True, format='bgr')


    def _calibrate(self):
    # locate the top/bottom black bars of a bar/grey code,
    #    then calculate the bin locations and fill the bin_pixels list 
    #    arr: 3D nparray of [x][y][color] data '''

        cfg_stream = picamera.array.PiRGBArray(self.camera)
        self.camera.capture(cfg_stream, format='bgr')

        top_bar_pixel = None
        bottom_bar_pixel = None
        luminosities = []

        ROWS = len(cfg_stream.array)
        COLS = len(cfg_stream.array[0])

        # fill luminosity array
        for i in range(ROWS):
            r = cfg_stream.array[i][SCAN_COLUMN][0]
            g = cfg_stream.array[i][SCAN_COLUMN][1]
            b = cfg_stream.array[i][SCAN_COLUMN][2]
            luminosities.append(sqrt(r**2 + b**2 + g**2))

        BLACK_BRIGHTNESS_MAX = 75.0
        for i in range(ROWS):
            if luminosities[i] < BLACK_BRIGHTNESS_MAX:
                top_bar_pixel = i
                break

        for i in range(ROWS-1, -1, -1):
            if luminosities[i] < BLACK_BRIGHTNESS_MAX:
                bottom_bar_pixel = i
                break

        if (bottom_bar_pixel == None) or (top_bar_pixel == None):
            print("Couldn't locate top/bottom bar pixel(s)! Closing camera...")
            cfg_stream.close()
            self.close()
            exit()

        dp = abs((bottom_bar_pixel - top_bar_pixel) / 20.0)

        global bin_pixels
        bin_pixels.append(top_bar_pixel)
        bin_pixels += np.arange(top_bar_pixel + dp, bottom_bar_pixel, dp).tolist()

        for i in range(len(bin_pixels)):
            bin_pixels[i] = int(round(bin_pixels[i]))

        if bin_pixels[-1] != bottom_bar_pixel:
            bin_pixels.append(bottom_bar_pixel)


        if DEBUG_LEVEL >= 2:
            print("#### START BIN DATA ####")
            print("Top bar pixel: " + str(top_bar_pixel))
            print("Bottom bar pixel: " + str(bottom_bar_pixel))
            print("Bin Pixels: " + str(bin_pixels))
            print("#### END BIN DATA ####\n")

        if OUTPUT_BIN_IMAGE:
            cv2.line(cfg_stream.array, (0,top_bar_pixel),(COLS, top_bar_pixel) ,(0,0,255),1)
            cv2.line(cfg_stream.array, (0,bottom_bar_pixel),(COLS, bottom_bar_pixel) ,(0,255,0),1)
            for i in bin_pixels[1:-1]:
                cv2.line(cfg_stream.array, (0,i), (COLS,i), (255,0,0), 1)
            cv2.line(cfg_stream.array, (SCAN_COLUMN,0),(SCAN_COLUMN, ROWS) ,(255,255,255),1)

            cv2.imwrite("bins.bmp", cfg_stream.array)

        cfg_stream.close()

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
        if DEBUG_LEVEL >= 2 and NUM_SCANS > 1:
            global image_times
            t0 = image_times[0]
            for i in range(taken):
                image_times[i] -= t0
            print("AVERAGE SCAN RATE: " + str(taken / image_times[-1]) + " Hz")

g2 = G2Scanner()
g2.scan()
g2.close()
