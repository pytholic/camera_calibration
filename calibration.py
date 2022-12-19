import cv2
import argparse
import os
from utils import Calibrator

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", help="Input source (camera index/video path)", default='0')
    parser.add_argument("-c", "--checkerboard_size", help="Checkboard size (number of columns x number of rows) (default is 6,4)", type=lambda s: [int(item) for item in s.split(',')], default = [6, 4])
    parser.add_argument("-m", "--model", help="Camera model (0: pinhole (default), 1: fisheye)", type=int, default=0)
    parser.add_argument("-t", "--thermal", help="Is thermal camera (default is false)", action='store_true')
    parser.add_argument("-l", "--low_res", help="Is low-resolution stream", action='store_true')
    parser.add_argument("-p", "--param_thres", help="Param threshold for evaluating good detected corners (default is 0.2)",
                    type=float, default=0.2)
    parser.add_argument("-q", "--quantity_thres", help="Minimum number of good images for calibration (default is 40)",
                    type=int, default=20)
    parser.add_argument("-o", "--output_path", help="Path to the output OpenCV calibration file (.DAT file)",
                    type=str, default='output.dat')
    parser.add_argument("-g", "--good", help="Directory to save good images", type=str, default="")
    parser.add_argument("-d", "--detect", help="Directory to save detect images", type=str, default="")

    args = parser.parse_args()
    print(args)
    source = args.index
    cam = cv2.VideoCapture(eval(source) if source.isnumeric() else source)
    OUTPUT_FILE = args.output_path
    calibrator = Calibrator(args.model, args.param_thres, args.quantity_thres) 
    calibrator.setCheckerboard(args.checkerboard_size, args.low_res) # args.thermal or 
    
    good = args.good 
    if good != "" and os.path.isdir(good) is False:
        os.mkdir(good)
        
    detect = args.detect 
    if detect != "" and os.path.isdir(detect) is False:
        os.mkdir(detect)
    
    if not source.isnumeric():
        frameNum = cam.get(cv2.CAP_PROP_FRAME_COUNT)
    idx = 0
    while True:
        haveFrame, frame = cam.read()
        if not haveFrame:
            if source.isnumeric() or cam.get(cv2.CAP_PROP_POS_FRAMES) >= frameNum:
                break
            else:
                continue
        
        raw = frame.copy()
        err, img = calibrator.detectCorner(frame, args.thermal)

        label = "Accumulating..."
        n = calibrator.getAccumulatedLen()
        # if success:
        #     if calibrator.isGoodEnough():
        #         print("Detect corners successfully. Accumulated {} images. Enough images for calibration!".format(calibrator.getAccumulatedLen()))
        #     else:
        #         print("Detect corners successfully. Accumulated {} images".format(calibrator.getAccumulatedLen()))
        if not err:
            print("Detected good corners. Accumulated {} images.".format(n)) 
            if good != "":
                cv2.imwrite("{}/img{}.jpg".format(good, idx), raw)
            if detect != "":
                result = img.copy()
                cv2.imwrite("{}/img{}.jpg".format(detect, idx), result)
            idx += 1

        elif err == 1:
            print("Detected bad corners. Accumulated {} images.".format(n)) 
        else:
            print("Detect corners failed. Accumulated {} images".format(n))
        if calibrator.isGoodEnough():
            print("Enough image for calibration!")
        
        cv2.imshow("Result", img)
        
        # idx += 1
        
        if cv2.waitKey(1) == ord('q'):# and calibrator.goodenough:
            break
        
    if calibrator.isGoodEnough():
        calibrator.calibrate()
        calibrator.saveCalib(OUTPUT_FILE)
    else:
        print("Not enough images for calibration!")