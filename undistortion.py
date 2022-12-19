import cv2
import argparse
from utils import Calibrator

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", "--index", help="Input source (camera index/video path)", default='0')
    parser.add_argument("-m", "--model", help="Camera model (0: pinhole (default), 1: fisheye)", type=int, default=0)
    parser.add_argument("-c", "--calibration_path", help="Path to the OpenCV calibration file (.DAT file)",
                    type=str, required=True)
    parser.add_argument("-z", "--zoom", help="Zoom value, \
        range from 0 (zoomed in, all pixels in calibrated image are valid) \
        to 1 (zoomed out, all pixels in original image are in calibrated image). \
            Default is 0.",
                    type=float, default=0.0)
    parser.add_argument("-s", "--show_original", help="Show original video (1: yes - default, 0: no)",
                    type=int, default=1)
    
    args = parser.parse_args()
    
    calibrator = Calibrator(args.model)
    calibrator.getCalib(args.calibration_path)
    calibrator.initRectifyMap(args.zoom)
    
    source = args.index
    cap = cv2.VideoCapture(eval(source) if source.isnumeric() else source)
    
    while True:
        
        # Capture frame-by-frame
        success, frame = cap.read()
        
        if success:
            # undistort
            dst = calibrator.undistort(frame)
            
            # Display the resulting frame
            cv2.imshow("undistorted", dst)
            if args.show_original != 0:
                cv2.imshow("original", frame)
        else:
            # print("Cannot open stream")
            break
        
        # Press Q on keyboard to  exit
        if cv2.waitKey(25) & 0xFF == ord('q'):
            break
    
    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()
    
    