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

    public static RelativePose computeOutputValues(Mat rvec, Mat tvec){
        /*
            *Diagram of Variables*

               |
               |x
           ╔═══╗
           ║cam║-------- z
           ╚═══╝ \hd|
                  \ |
          distance \|
                    \__ |
                     \oy|
                      \ |
                     ╔══════╗
                     ║target║
                     ╚══════╝
        
        */
        
        double x = tvec.get(0, 0)[0] + CAMERA_OFFSET_X; //for some reason get outputs a double[] of length 1, so just get the first value
        double z = Math.sin(CAMERA_TILT) *  tvec.get(1, 0)[0] + Math.cos(CAMERA_TILT) * tvec.get(2,0)[0] +  CAMERA_OFFSET_Z; //ditto for here, but factor in tilt offset
        

        //get the distance in the horizontal plane between camera and target
        double distance = Math.sqrt(Math.pow(x,2) + Math.pow(z,2));
       

        //get the horizontal angle between camera center line and target
        double heading = Math.atan2(x,z);


        Mat rot = new Mat();
        Calib3d.Rodrigues(rvec, rot); //unpacks rot matrix from rved
        System.out.println("rot = " + rot.dump());

        double[] angles = rotToEulerAngles(rot); //gets euler angles of object from rot matrix
        double objectYaw = angles[1];
        
        return new RelativePose(heading, distance, objectYaw);
    }

    private static double[] rotToEulerAngles(Mat rotationMatrix)
    {
        double r00 = rotationMatrix.get(0, 0)[0];
        double r10 = rotationMatrix.get(1, 0)[0];
        double sy = Math.sqrt(r00 * r00 + r10 * r10);
        double x, y, z;
        if (sy >= 1e-6)
        {
            x = Math.atan2(rotationMatrix.get(2, 1)[0], rotationMatrix.get(2, 2)[0]);
            y = Math.atan2(-rotationMatrix.get(2, 0)[0], sy);
            z = Math.atan2(r10, r00);
        }
        else
        {
            x = Math.atan2(-rotationMatrix.get(1, 2)[0], rotationMatrix.get(1, 1)[0]);
            y = Math.atan2(-rotationMatrix.get(2, 0)[0], sy);
            z = 0;
        }
        return new double[] { Math.toDegrees(x), Math.toDegrees(y), Math.toDegrees(z) };
    }
}


