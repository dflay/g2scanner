import numpy as np
import time
import picamera
import picamera.array
import cv2

DEBUG_LEVEL = 0
OUTPUT_BIN_IMAGE = True

class G2Scanner(picamera.array.PiRGBAnalysis):
    def __init__(self, camera):
    
        # init PiRGBAnalysis class
        super(G2Scanner, self).__init__(camera)

        # initialize variables
        self.INIT_TIME = time.time()
        self.scan_times = []
        self.num_scans = 0

    def analyse(self, arr):

        # TODO: DONT RECORD THESE ONCE PROGRAM IS SET, 
        # WOULD TAKE UP TOO MUCH MEMORY ON A LONG RUN
        self.scan_times.append(time.time() - self.INIT_TIME)
        self.num_scans += 1

        ###################
        ### CALIBRATION ###
        ###################

        ROWS = len(arr)
        COLS = len(arr[0])

        top_bar_pixel = None
        bottom_bar_pixel = None

        # used for finding the top/bottom bars and
        # calculating bin_pixels only
        temp_lums = np.sum(arr[:,8,:],1)

        BLACK_LUM_MAX = 100
        
        for i in range(ROWS):
            if temp_lums[i] < BLACK_LUM_MAX:
                top_bar_pixel = i
                break

        for i in range(ROWS-1,-1,-1):
           if temp_lums[i] < BLACK_LUM_MAX:
               bottom_bar_pixel = i
               break
       
        if top_bar_pixel == None or bottom_bar_pixel == None:
            print("couldn't find top/bottom!")
            return

        bin_pixels = np.linspace(top_bar_pixel, bottom_bar_pixel, 21)
        bin_pixels = np.vectorize(lambda x: int(round(x)))(bin_pixels)


        ######################
        ### EDGE DETECTION ###
        ######################

        edges = cv2.Canny(arr,60,250) 

        max_edge = np.argmax(np.sum(edges, axis=0))

        if max_edge > 9:
            scan_column = max_edge - 4
        else: 
            scan_column = max_edge + 4

        lums = np.sum(arr[:,scan_column,:],1)
        
        bin_bits = []
        for i in range(len(bin_pixels)-1):
            tp = bin_pixels[i]
            bp = bin_pixels[i+1]
            if np.sum(lums[tp:bp]/(bp-tp)) < 2*BLACK_LUM_MAX:
                bin_bits.append(1)
            else: 
                bin_bits.append(0)

    
        if OUTPUT_BIN_IMAGE:
              cv2.line(arr, (0,top_bar_pixel),(COLS, top_bar_pixel) ,(0,0,255),1)
              cv2.line(arr, (0,bottom_bar_pixel),(COLS, bottom_bar_pixel) ,(0,255,0),1)
              for i in bin_pixels[1:-1]:
                  cv2.line(arr, (0,i), (COLS,i), (255,0,0), 1)
              for i in range(20):
                  y = 2 + (bin_pixels[i+1] + bin_pixels[i]) / 2
                  if scan_column < 8:
                      x = 12
                  else: x = 2
                  cv2.putText(arr, str(bin_bits[i]), (x,y), cv2.FONT_HERSHEY_PLAIN,0.4,(255,0,255))
              cv2.line(arr, (scan_column,0),(scan_column, ROWS) ,(255,255,255),1)
              cv2.imwrite(str(self.num_scans) + ".bmp", arr)
        
with picamera.PiCamera() as camera:
    time.sleep(0.5)
    with G2Scanner(camera) as output:
        camera.resolution = (16, 300)
        camera.framerate = 10
        camera.start_recording(output , format='bgr')
        camera.wait_recording(4)
        camera.stop_recording()
        print output.num_scans / (output.scan_times[-1] - output.scan_times[0])
