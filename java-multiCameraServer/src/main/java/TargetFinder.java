import java.lang.reflect.Array;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;
import java.util.HashMap;

import edu.wpi.first.vision.VisionPipeline;

import org.opencv.core.*;
import org.opencv.core.Core.*;
import org.opencv.features2d.FeatureDetector;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.*;
import org.opencv.calib3d.*;
import org.opencv.objdetect.*;
import java.util.Vector;

public class TargetFinder{
    public void ProcessImage(Mat camera_frame, List<MatOfPoint> contours){
        
        for (int i = 0; i < contours.size(); i++){}
            if (contours[i] > 
        }
    }

    public void contour_center_width(contour){
        Rect r = Imgproc.boundingRect(camera_frame.filterContoursOutput().get(0));
        x, y, w, h = cv2.boundingRect(contour)
    }
    public void computeOutputValues(Mat rvec, Mat tvec){
        /*
            *Diagram of Variables*

               |
               |x
           ╔═══╗
           ║cam║-------- z
           ╚═══╝ \a1|
                  \ |
          distance \
                    \__ |
                     \a2| 
                      \ |                    
                     ╔══════╗
                     ║target║
                     ╚══════╝     
        
        */
        
        double x = tvec[0];
        double z = tvec[2];
        

        // distance in the horizontal plane between camera and target
        double distance = Math.sqrt(Math.pow(x,2) + Math.pow(z,2));
       
        // horizontal angle between camera center line and target
        double angle1 = Math.atan2(x,z);

        Mat rot = new Mat();
        Calib3d.Rodrigues(rvec, rot);
        Mat rot_inv = rot.t();
        Mat pzero_world = tvec* rot_inv;
        Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    }
  }