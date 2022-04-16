import cv2
import numpy as np

# global variables go here:
testVar = 0

# To change a global variable inside a function,
# re-declare it the global keyword
def incrementTestVar():
    global testVar
    testVar = testVar + 1
    if testVar == 100:
        print("test")
    if testVar >= 200:
        print("print")
        testVar = 0

def drawDecorations(image):
    cv2.putText(image, 
        'Limelight python script!', 
        (0, 230), 
        cv2.FONT_HERSHEY_SIMPLEX, 
        .5, (0, 255, 0), 1, cv2.LINE_AA)
    
# runPipeline() is called every frame by Limelight's backend.
def runPipeline(image, llrobot):
    try:
        print (cv2.__version__)
        detector = cv2.SimpleBlobDetector()
        img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        img_threshold_blue = cv2.inRange(img_hsv, (60, 70, 70), (85, 255, 255))
        img_threshold_red = cv2.inRange(img_hsv, (60, 70, 70), (85, 255, 255))

        # blueBall = findLargestBall(img_threshold_blue, detector)
        # redBall = findLargestBall(img_threshold_red, detector)

        # if blueBall is not None:
        #     print ("{} {}".format(blueBall.x, blueBall.y))
        # if redBall is not None:
        #    print ("{} {}".format(redBall.x, redBall.y))
        # pass
    except Exception as e:
        print (e)
        pass
    finally:
        largestContour = np.array([[]])
        llpython = [0,0,0,0,0,0,0,0]
        return largestContour, image, llpython

def findLargestBall (img_threshold, detector): 
    keypoints = detector.detect(img_threshold)

    largestKeypoint = None
    for keypoint in keypoints:
        if largestKeypoint is None or largestKeypoint.size > keypoint.size:
            largestKeypoint = keypoint

    return largestKeypoint