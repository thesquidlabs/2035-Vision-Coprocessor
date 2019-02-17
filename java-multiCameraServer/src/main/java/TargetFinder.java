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
    
    public static final double CAMERA_TILT = Math.toRadians(0);
    public static final double CAMERA_OFFSET_X = 0;
    public static final double CAMERA_OFFSET_Z = 0;

    public void contour_center_width(MatOfPoint contour){
       // Rect r = Imgproc.boundingRect(camera_frame.filterContoursOutput().get(0));
        //x, y, w, h = cv2.boundingRect(contour);
    }
    public static void computeOutputValues(Mat rvec, Mat tvec){
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
        
        double x = tvec.get(0, 0)[0] + CAMERA_OFFSET_X; //for some reason get outputs a double[] of length 1, so just get the first value
        double z = Math.sin(CAMERA_TILT) *  tvec.get(1, 0)[0] + Math.cos(CAMERA_TILT) * tvec.get(2,0)[0] +  CAMERA_OFFSET_Z; //ditto for here, but factor in tilt offset
        
        System.out.println("x = " + x + " z = " + z);

        // distance in the horizontal plane between camera and target
        double distance = Math.sqrt(Math.pow(x,2) + Math.pow(z,2));
       
        System.out.println("distance = " + distance);


        // horizontal angle between camera center line and target
        double angle1 = Math.atan2(x,z);

        System.out.println("angle1 = " + angle1);


        Mat rot = new Mat();
        Calib3d.Rodrigues(rvec, rot);
        System.out.println("rot = " + rot.dump());

        Mat rot_inv = rot.t();
        System.out.println("rot_inv = " + rot_inv.dump());
        
        //Mat pzero_world = -tvec* rot_inv;
        //Calib3d.solvePnP(objectPoints, imagePoints, cameraMatrix, distCoeffs, rvec, tvec);
    }
  }
