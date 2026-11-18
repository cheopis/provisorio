import cv2
import numpy as np

from math import radians


class LineDetector:
    def __init__(self):
        # Class parameters
        self.img = None         # Original image to process
        self.gray = None       # Cany detection image
        self.lines = None       # Lines detected
        self.lines_list = []    # List to organize the lines detected 
        self.thresh = None

        self.method = 'H'       # 'H' for hough or 'M' for morphology 
        self.print = True
        self.depth = False

    def pre_process_image(self):
        if self.depth:
            cv_image_array = np.array(self.img, dtype = np.dtype('f8'))
            cv_image_norm = cv2.normalize(cv_image_array, cv_image_array, 0, 700, cv2.NORM_MINMAX)
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(cv_image_norm, alpha=0.5), cv2.COLORMAP_JET)
            cv_image_resized = cv2.resize(depth_colormap, (480,270), interpolation = cv2.INTER_CUBIC)
            self.img = cv_image_resized

        # Convert image to grayscale and blur
        #ajust = cv2.convertScaleAbs(self.img, alpha=-1.5, beta=50)
        gray = cv2.cvtColor(self.img,cv2.COLOR_BGR2GRAY)
        self.gray = cv2.GaussianBlur(gray, (1, 3), 0) 
        #self.print_image(self.edges)

        # cv2.imshow("asasd", ajust)
        # cv2.waitKey(1)

    def apply_morphology(self):
        try:
            gray = cv2.bitwise_not(self.gray)
            #gray = cv2.normalize(self.gray, self.gray, 0, 1, cv2.NORM_MINMAX)
            
            self.thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)[1]
            #self.thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_MEAN_C, \
            #                                    cv2.THRESH_BINARY, 15, -2)

            vertical_kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (1,self.img.shape[0] // 30))
            detect_vertical = cv2.morphologyEx(self.thresh, cv2.MORPH_OPEN, vertical_kernel, iterations=5)
            cnts = cv2.findContours(detect_vertical, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
            cnts = cnts[0] if len(cnts) == 2 else cnts[1]

            if self.print:
                for c in cnts:
                    cv2.drawContours(self.img, [c], -1, (36,255,12), 1)
                    #print(c)
            return cnts[0]
        
        except:
            pass

    def detect_vertical_lines(self, cnts):
        self.lines_list = []
        try:
            rect = cv2.minAreaRect(cnts)
            box = cv2.boxPoints(rect)
            box = np.int0(box)

            for i in range(0,2):
                (x1,y1) = (box[3*i-1][0],box[3*i-1][1])
                (x2,y2) = (box[i][0],box[i][1])

                theta = radians(rect[2])

                center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

                if self.print:
                    cv2.line(self.img,(x1,y1),(x2,y2),(i*255,0,255),2)

                self.lines_list.append([(x1,y1),(x2,y2),theta, center])

        except:
            pass   
    
    def apply_hough(self):  
        # Use canny edge detection
        edges = cv2.Canny(self.gray,30,150)  

        # lines = cv2.HoughLinesP(
        #     edges, # Input edge image
        #     1, # Distance resolution in pixels
        #     np.pi/180, # Angle resolution in radians
        #     threshold=5, # Min number of votes for valid line
        #     minLineLength=5, # Min allowed length of line
        #     maxLineGap=1 # Max allowed gap between line for joining them
        #     )

        lines = cv2.HoughLines(edges,1,np.pi/180,100)
        
        self.lines = lines
    
    def iterate_points(self):
        self.lines_list = []
        try: 
            for line in self.lines:
                for rho,theta in line:
                    # Remove horizontal lines
                    if theta > 0.1: # in radians
                        continue
                    # Extracted points nested in the list
                    a = np.cos(theta)
                    b = np.sin(theta)
                    x0 = a*rho
                    y0 = b*rho
                    x1 = int(x0 + 1000*(-b))
                    y1 = int(y0 + 1000*(a))
                    x2 = int(x0 - 1000*(-b))
                    y2 = int(y0 - 1000*(a))

                    # Draw the lines joing the points on the original image
                    cv2.line(self.img,(x1,y1),(x2,y2),(0,255,0),2)

                    center = ((x1 + x2) / 2.0, (y1 + y2) / 2.0)

                    # Maintain a simples lookup list for points
                    self.lines_list.append([(x1,y1),(x2,y2),theta, center])
        except:
            pass

    def print_image(self, img):
        cv2.imshow("Result", img)
        cv2.waitKey(1)

    def detect_lines(self, image):
        # Read image
        self.img = np.array(image, dtype=np.uint8)

        # Preprocess image
        self.pre_process_image()

        if self.method == 'H':
            # Get lines and points
            self.apply_hough()

            self.iterate_points()

        elif self.method == 'M':
            c = self.apply_morphology()

            self.detect_vertical_lines(c)

        else: 
            print("Wrong method, closing line detection node")

        cv2.imshow("Result", self.img)
        cv2.waitKey(1)

        return self.lines_list, self.img