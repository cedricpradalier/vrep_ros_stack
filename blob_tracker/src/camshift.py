#!/usr/bin/env python
import roslib
roslib.load_manifest('blob_tracker')

import cv2.cv as cv
import rospy
from sensor_msgs.msg import RegionOfInterest, Image
from cv_bridge import CvBridge


def is_rect_nonzero(r):
    (_,_,w,h) = r
    return (w > 0) and (h > 0)

class CamShiftDemo:

    def __init__(self):
        # self.capture = cv.CaptureFromCAM(0)
        self.frame = cv.CreateImage( (320,200), 8, 3)
        self.backproject = cv.CreateImage( (320,200), 8, 3)
        self.br = CvBridge()
        self.header = None
        cv.NamedWindow( "CamShiftDemo", 1 )
        cv.NamedWindow( "Histogram", 1 )
        cv.SetMouseCallback( "CamShiftDemo", self.on_mouse)

        self.drag_start = None      # Set to (x,y) when mouse starts drag
        self.track_window = None    # Set to rect when the mouse drag finishes
        self.pause = False

        print( "Keys:\n"
            "    ESC - quit the program\n"
            "    b - switch to/from backprojection view\n"
            "    p - pause processing\n"
            "To initialize tracking, drag across the object with the mouse\n" )

    def hue_histogram_as_image(self, hist):
        """ Returns a nice representation of a hue histogram """

        histimg_hsv = cv.CreateImage( (320,200), 8, 3)

        mybins = cv.CloneMatND(hist.bins)
        cv.Log(mybins, mybins)
        (_, hi, _, _) = cv.MinMaxLoc(mybins)
        cv.ConvertScale(mybins, mybins, 255. / hi)

        w,h = cv.GetSize(histimg_hsv)
        hdims = cv.GetDims(mybins)[0]
        for x in range(w):
            xh = (180 * x) / (w - 1)  # hue sweeps from 0-180 across the image
            val = int(mybins[int(hdims * x / w)] * h / 255)
            cv.Rectangle( histimg_hsv, (x, 0), (x, h-val), (xh,255,64), -1)
            cv.Rectangle( histimg_hsv, (x, h-val), (x, h), (xh,255,255), -1)

        histimg = cv.CreateImage( (320,200), 8, 3)
        cv.CvtColor(histimg_hsv, histimg, cv.CV_HSV2BGR)
        return histimg

    def on_mouse(self, event, x, y, flags, param):
        if event == cv.CV_EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
        if event == cv.CV_EVENT_LBUTTONUP:
            self.drag_start = None
            self.track_window = self.selection
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax - xmin, ymax - ymin)

    def detect_and_draw(self,imgmsg):
        if self.pause:
            return
        # frame = cv.QueryFrame( self.capture )
        frame = self.br.imgmsg_to_cv(imgmsg, "bgr8")


        # Convert to HSV and keep the hue
        hsv = cv.CreateImage(cv.GetSize(frame), 8, 3)
        cv.CvtColor(frame, hsv, cv.CV_BGR2HSV)
        self.hue = cv.CreateImage(cv.GetSize(frame), 8, 1)
        cv.Split(hsv, self.hue, None, None, None)

        # Compute back projection
        backproject = cv.CreateImage(cv.GetSize(frame), 8, 1)

        # Run the cam-shift
        cv.CalcArrBackProject( [self.hue], backproject, self.hist )
        if self.track_window and is_rect_nonzero(self.track_window):
            crit = ( cv.CV_TERMCRIT_EPS | cv.CV_TERMCRIT_ITER, 10, 1)
            (iters, (area, value, rect), track_box) = cv.CamShift(backproject, self.track_window, crit)
            self.track_window = rect
            x,y,w,h = rect
            self.bbpub.publish(RegionOfInterest(x,y, w,h,False))
            proba_msg = self.br.cv_to_imgmsg(backproject)
            proba_msg.header = imgmsg.header
            self.bppub.publish(proba_msg)

        # If mouse is pressed, highlight the current selected rectangle
        # and recompute the histogram

        if self.drag_start and is_rect_nonzero(self.selection):
            sub = cv.GetSubRect(frame, self.selection)
            save = cv.CloneMat(sub)
            cv.ConvertScale(frame, frame, 0.5)
            cv.Copy(save, sub)
            x,y,w,h = self.selection
            cv.Rectangle(frame, (x,y), (x+w,y+h), (255,255,255))

            sel = cv.GetSubRect(self.hue, self.selection )
            cv.CalcArrHist( [sel], self.hist, 0)
            (_, max_val, _, _) = cv.GetMinMaxHistValue( self.hist)
            if max_val != 0:
                cv.ConvertScale(self.hist.bins, self.hist.bins, 255. / max_val)
        elif self.track_window and is_rect_nonzero(self.track_window):
            cv.EllipseBox( frame, track_box, cv.CV_RGB(255,0,0), 3, cv.CV_AA, 0 )

        self.frame = frame
        self.backproject = backproject



    def run(self):
        self.backproject_mode = False
        self.hist = cv.CreateHist([180], cv.CV_HIST_ARRAY, [(0,180)], 1 )

        rospy.init_node('blob_tracker')
        image_topic = rospy.resolve_name("image")
        self.bbpub = rospy.Publisher("~blob",RegionOfInterest)
        self.bppub = rospy.Publisher("~backproject",Image)
        self.disp_hist = rospy.get_param("~display_histogram",True)
        rospy.Subscriber(image_topic, Image, self.detect_and_draw)
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            # rospy.spin_once()
            if not self.pause:
                if not self.backproject_mode:
                    cv.ShowImage( "CamShiftDemo", self.frame )
                else:
                    cv.ShowImage( "CamShiftDemo", self.backproject)
                if self.disp_hist:
                    cv.ShowImage( "Histogram", self.hue_histogram_as_image(self.hist))
            c = cv.WaitKey(7) & 0x0FF
            if c == 27:
                rospy.signal_shutdown("OpenCV said so")
            elif c == ord("p"):
                self.pause = not self.pause
            elif c == ord("b"):
                self.backproject_mode = not self.backproject_mode

if __name__=="__main__":
    demo = CamShiftDemo()
    demo.run()
    cv.DestroyAllWindows()