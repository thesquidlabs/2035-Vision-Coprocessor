
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
import java.awt.Color;
import java.awt.SystemTray;
import java.io.IOException;
import java.nio.channels.NonReadableChannelException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import javax.lang.model.util.ElementScanner6;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;

import org.opencv.calib3d.Calib3d;
import org.opencv.imgproc.Imgproc;
import org.opencv.core.*;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.vision.VisionPipeline;
import edu.wpi.first.vision.VisionRunner;
import edu.wpi.first.vision.VisionThread;
import edu.wpi.first.vision.VisionRunner.Listener;


/*
   JSON format:
   {
       "team": <team number>,
       "ntmode": <"client" or "server", "client" if unspecified>
       "cameras": [
           {
               "name": <camera name>
               "path": <path, e.g. "/dev/video0">
               "pixel format": <"MJPEG", "YUYV", etc>   // optional
               "width": <video mode width>              // optional
               "height": <video mode height>            // optional
               "fps": <video mode fps>                  // optional
               "brightness": <percentage brightness>    // optional
               "white balance": <"auto", "hold", value> // optional
               "exposure": <"auto", "hold", value>      // optional
               "properties": [                          // optional
                   {
                       "name": <property name>
                       "value": <property value>
                   }
               ]
           }
       ]
   }
 */

public final class Main {
  private static String configFile = "/boot/frc.json";

  @SuppressWarnings("MemberName")
  public static class CameraConfig {
    public String name;
    public String path;
    public JsonObject config;
  }

  public static int team;
  public static boolean server;
  public static List<CameraConfig> cameraConfigs = new ArrayList<>();
  private static final Object ImgLock = new Object();
  private static final double TARGET_STRIP_WIDTH = 2.0;             // inches
  private static final double TARGET_STRIP_LENGTH = 5.5;            // inches
  private static final double TARGET_STRIP_CORNER_OFFSET = 4.0;     // inches
  private static final double TARGET_STRIP_ROT = Math.toRadians(14.5);
  public static final int CAMERA_WIDTH = 320;
  public static final int CAMERA_HEIGHT = 240;
  public static final int NT_WAIT_COUNT = 5;
  public static final double H_FOV = Math.toRadians(58.5);
  public static final double V_FOV = Math.toRadians(45.6);
  public static double cos_a = Math.cos(TARGET_STRIP_ROT);
  public static double sin_a = Math.sin(TARGET_STRIP_ROT);
  public static final boolean USE_CALIBRATED_IMAGEMAT = true;
  public static MatOfDouble distCoeffs;
  
  static NetworkTableEntry objectSeen;
  static NetworkTableEntry headingEntry;
  static NetworkTableEntry distanceEntry;
  static NetworkTableEntry objecyYawEntry;
  static ArrayList<Double> headingList;
  static ArrayList<Double> distanceList;
  static ArrayList<Double> objectYawList;
  static double hAve, dAve, yAve;


  private Main() {
  }

  /**
   * Report parse error.
   */
  public static void parseError(String str) {
    System.err.println("config error in '" + configFile + "': " + str);
  }

  /**
   * Read single camera configuration.
   */
  public static boolean readCameraConfig(JsonObject config) {
    CameraConfig cam = new CameraConfig();

    // name
    JsonElement nameElement = config.get("name");
    if (nameElement == null) {
      parseError("could not read camera name");
      return false;
    }
    cam.name = nameElement.getAsString();

    // path
    JsonElement pathElement = config.get("path");
    if (pathElement == null) {
      parseError("camera '" + cam.name + "': could not read path");
      return false;
    }
    cam.path = pathElement.getAsString();

    cam.config = config;

    cameraConfigs.add(cam);
    return true;
  }

  /**
   * Read configuration file.
   */
  @SuppressWarnings("PMD.CyclomaticComplexity")
  public static boolean readConfig() {
    // parse file
    JsonElement top;
    try {
      top = new JsonParser().parse(Files.newBufferedReader(Paths.get(configFile)));
    } catch (IOException ex) {
      System.err.println("could not open '" + configFile + "': " + ex);
      return false;
    }

    // top level must be an object
    if (!top.isJsonObject()) {
      parseError("must be JSON object");
      return false;
    }
    JsonObject obj = top.getAsJsonObject();

    // team number
    JsonElement teamElement = obj.get("team");
    if (teamElement == null) {
      parseError("could not read team number");
      return false;
    }
    team = teamElement.getAsInt();

    // ntmode (optional)
    if (obj.has("ntmode")) {
      String str = obj.get("ntmode").getAsString();
      if ("client".equalsIgnoreCase(str)) {
        server = false;
      } else if ("server".equalsIgnoreCase(str)) {
        server = true;
      } else {
        parseError("could not understand ntmode value '" + str + "'");
      }
    }

    // cameras
    JsonElement camerasElement = obj.get("cameras");
    if (camerasElement == null) {
      parseError("could not read cameras");
      return false;
    }
    JsonArray cameras = camerasElement.getAsJsonArray();
    for (JsonElement camera : cameras) {
      if (!readCameraConfig(camera.getAsJsonObject())) {
        return false;
      }
    }

    return true;
  }

  /**
   * Start running the camera.
   */
  public static VideoSource startCamera(CameraConfig config) {
    System.out.println("Starting camera '" + config.name + "' on " + config.path);
    VideoSource camera = CameraServer.getInstance().startAutomaticCapture(
        config.name, config.path);

    Gson gson = new GsonBuilder().create();

    camera.setConfigJson(gson.toJson(config.config));

    return camera;
  }

  /**
   * Example pipeline.
   */
  public static class MyPipeline implements VisionPipeline {
    public int val;

    @Override
    public void process(Mat mat) {
      val += 1;
    }
  }

  public static ArrayList<Point> get_outside_corners_single(List<Point> contour, boolean isLeft){
    double y_ave = 0.0;
    for (Point cnr : contour) {
      y_ave += cnr.y;
    }
    y_ave /= contour.size();

    ArrayList<Point> corners = new ArrayList<Point>();
    Point dummyPoint = new Point();
    corners.add(dummyPoint); corners.add(dummyPoint); //populates with dummy points
    if(isLeft){
      int index = 0;
      for (Point cnr : contour) {
        //larger in y (lower in picture) at index 1
        if(cnr.y > y_ave)
          index = 1;
        else
          index = 0;
        if (corners.get(index).x == 0 && corners.get(index).y == 0){
            corners.set(index, cnr);
        }
        else if(cnr.x < corners.get(index).x){
          corners.set(index, cnr);
        }
      }

    }else{
      int index = 0;
      for (Point cnr : contour) {
        //larger in y (lower in picture) at index 1
        if(cnr.y > y_ave)
          index = 1;
        else
          index = 0;
        if (corners.get(index) == null || cnr.x > corners.get(index).x){
            corners.set(index, cnr);
        }
      }
    }
    System.out.println("Corners: " + corners);
    return corners;
  }

  public static Point findCenter(List<Point> cnt){
    int aveX = 0;
    int aveY = 0;
    for (Point pt : cnt){
        aveX += pt.x;
        aveY += pt.y;
    }
    aveX /= cnt.size();
    aveY /= cnt.size();
    Point center = new Point(aveX, aveY);
    return center;
  }

  /**
   * Main.
   */
  public static void main(String... args) {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

    headingList = new ArrayList<>();
    distanceList = new ArrayList<>();
    objectYawList = new ArrayList<>();

    hAve = 0; dAve = 0; yAve = 0;

     //gets the values a left and right strip should have
     Point3 pt = new Point3(TARGET_STRIP_CORNER_OFFSET, 0, 0);
     ArrayList<Point3> right_strip = new ArrayList<Point3>();
     System.out.println("x = " + pt.x + ", y = "+ pt.y);
     right_strip.add(new Point3(pt.x, pt.y, pt.z));
     pt.x += TARGET_STRIP_WIDTH * cos_a;
     pt.y += TARGET_STRIP_WIDTH * sin_a;
     System.out.println("x = " + pt.x + ", y = "+ pt.y);
     right_strip.add(new Point3(pt.x, pt.y, pt.z));
     pt.x += TARGET_STRIP_LENGTH * sin_a;
     pt.y -= TARGET_STRIP_LENGTH * cos_a;
     System.out.println("x = " + pt.x + ", y = "+ pt.y);
     right_strip.add(new Point3(pt.x, pt.y, pt.z));
     pt.x -= TARGET_STRIP_WIDTH * cos_a;
     pt.y -= TARGET_STRIP_WIDTH * sin_a;
     System.out.println("x = " + pt.x + ", y = "+ pt.y);
     right_strip.add(new Point3(pt.x, pt.y, pt.z));
    System.out.println(right_strip.toString());
     //mirror for the left strip
     ArrayList<Point3> left_strip = new ArrayList<Point3>();
     for (Point3 ptR : right_strip) {
       Point3 ptL = new Point3(-ptR.x, ptR.y, ptR.z);
       left_strip.add(ptL);
     }
     System.out.println("Left: " + left_strip.toString() + "Right: " + right_strip.toString());
     System.out.println("What it should be giving me: " +
     left_strip.get(2).toString() + left_strip.get(1).toString() +
     right_strip.get(1).toString() + right_strip.get(2).toString());
     MatOfPoint3f outside_target_coords = new MatOfPoint3f(
     left_strip.get(2), left_strip.get(1), right_strip.get(1), right_strip.get(2));
     //left bottom, left top, right top, right bottom
     System.out.println("Outside_target_coords: " + outside_target_coords.dump());
    


    if (args.length > 0) {
      configFile = args[0];
    }

    // read configuration
    if (!readConfig()) {
      return;
    }

    // start NetworkTables
    NetworkTableInstance ntinst = NetworkTableInstance.getDefault();
    if (server) {
      System.out.println("Setting up NetworkTables server");
      ntinst.startServer();
    } else {
      System.out.println("Setting up NetworkTables client for team " + team);
      ntinst.startClientTeam(team);
    }
    NetworkTable values = ntinst.getTable("values");
    objectSeen = values.getEntry("object");
    headingEntry = values.getEntry("heading");
    distanceEntry = values.getEntry("distance");
    objecyYawEntry = values.getEntry("objectYaw");


    // start cameras
    List<VideoSource> cameras = new ArrayList<>();
    for (CameraConfig cameraConfig : cameraConfigs) {
      cameras.add(startCamera(cameraConfig));
    }

    // start image processing on camera 0 if present
    if (cameras.size() >= 1) {
      System.out.println("Starting image processing on camera 0...");
      
      Mat cameraMatrix= new Mat(3, 3, 6);
      if(USE_CALIBRATED_IMAGEMAT){  //use calibrated data
        cameraMatrix.put(0, 0, 340); cameraMatrix.put(0, 1, 0.); cameraMatrix.put(0, 2, 166.8);   //camera_matrix values output from camera calibration
        cameraMatrix.put(1, 0, 0.); cameraMatrix.put(1, 1, 340.7); cameraMatrix.put(1, 2, 118);
        cameraMatrix.put(2, 0, 0.); cameraMatrix.put(2, 1, 0.); cameraMatrix.put(2, 2, 1.);
        distCoeffs= new MatOfDouble(0.0490448, -0.2692084, -0.0011227, 0.0004736, 0);
        //distCoeffs= new MatOfDouble(-5.78, -1.08, 2.32, 3.31, 0); //distortion_coefficients values output from camera calibration
      }else{ //approximate values
        double focal_x = (CAMERA_WIDTH/2)/(Math.tan(H_FOV/2));
        double focal_y = (CAMERA_WIDTH/2)/(Math.tan(V_FOV/2));
        cameraMatrix.put(0, 0, focal_x, 0, CAMERA_WIDTH / 2.0, 0, focal_y, CAMERA_HEIGHT / 2.0, 0, 0, 1);
        distCoeffs= new MatOfDouble(0, 0, 0, 0, 0);
      }

      System.out.println("Camera matrix values: "+ cameraMatrix.dump());
      System.out.println("Distortion Coefficient values: "+ distCoeffs.dump());
      
      CvSink cvSink = CameraServer.getInstance().getVideo();
      CvSource outputStream = CameraServer.getInstance().putVideo("Telemetry", 320, 240);
      Mat source = new Mat();
      Point imageCenter = new Point(CAMERA_WIDTH / 2, CAMERA_HEIGHT / 2);

      //Thread for vision processing
      VisionThread visionThread = new VisionThread(new VisionRunner<GripPipeline>(cameras.get(0),new GripPipeline(), execThread -> {
        
        cvSink.grabFrame(source);
        //make a crosshair
        Imgproc.line(source, new Point(imageCenter.x, imageCenter.y+5), new Point(imageCenter.x, imageCenter.y-5), new Scalar(255,255,255), 2);
        Imgproc.line(source, new Point(imageCenter.x+5, imageCenter.y), new Point(imageCenter.x-5, imageCenter.y), new Scalar(255,255,255), 2);


        if (execThread.filterContoursOutput().size() > 1 ){
            objectSeen.setBoolean(true);

            System.out.println("Found " + execThread.filterContoursOutput().size() + " contours!");
            //get the biggest contour
            //TODO: Add check for shapes being rough
            List<MatOfPoint> contours = execThread.filterContoursOutput();
            int n = contours.size();
            for (int i=1; i<n; ++i) {
                MatOfPoint key = contours.get(i);
                Rect rectKey = Imgproc.boundingRect(key);
                int j = i-1;

                while (j>=0 && Imgproc.boundingRect(contours.get(j)).area() > rectKey.area()) { 
                  contours.set(j+1, contours.get(j));
                  j = j-1;
                }
                contours.set(j+1,key);
            }
            MatOfPoint cnt1 = contours.get(contours.size()-1);
            MatOfPoint cnt2 = contours.get(contours.size()-2);
            if(cnt1 != null && cnt2 != null){

              MatOfPoint cnt_left;
              MatOfPoint cnt_right;
              //tell tell which strip is left and which is right
              if(findCenter(cnt1.toList()).x < findCenter(cnt2.toList()).x){
                 cnt_left= cnt1;
                 cnt_right= cnt2;
              }else{
                cnt_left = cnt2;
                cnt_right= cnt1;
              }
              
              ArrayList<Point> left = get_outside_corners_single(cnt_left.toList(), true);
              ArrayList<Point> right = get_outside_corners_single(cnt_right.toList(), false);
              //draw lines on telemetry

              Imgproc.line(source, left.get(1), left.get(0), new Scalar(255,0,0), 2);
              Imgproc.line(source, left.get(0), right.get(0), new Scalar(255,0,0), 2);
              Imgproc.line(source, right.get(0), right.get(1), new Scalar(255,0,0), 2);
              Imgproc.line(source, right.get(1), left.get(1), new Scalar(255,0,0), 2);

              Point targetCenter = new Point((left.get(0).x + right.get(0).x)/2, (left.get(0).y + left.get(1).y)/2);
              Imgproc.line(source, imageCenter, targetCenter, new Scalar(0,0,255), 2);


              MatOfPoint2f image_corners = new MatOfPoint2f(
              left.get(1), left.get(0), right.get(0), right.get(1));
              
                //[left_bottom, left_top, right_top, right_bottom]
              
              Mat rvec = new Mat();
              Mat tvec = new Mat();
              boolean retval = Calib3d.solvePnP(outside_target_coords, image_corners, cameraMatrix, distCoeffs, rvec, tvec);
              System.out.println("SolvePNP ran? " + retval);
              //System.out.println("rvec: " + rvec.dump() + " tvec: " + tvec.dump());
              if (retval){
                RelativePose pose = TargetFinder.computeOutputValues(rvec, tvec);
                System.out.println(pose.toString());
                
                headingList.add(pose.heading);
                distanceList.add(pose.distance);
                objectYawList.add(pose.objectYaw);
                hAve += pose.heading;
                dAve += pose.distance;
                yAve += pose.objectYaw;

                if(headingList.size() >= NT_WAIT_COUNT){ //gets the average of a set amount of values
                  hAve /= headingList.size();
                  dAve /= distanceList.size();
                  yAve /= objectYawList.size();
                  System.out.println("Averages: "+  hAve + ", " + dAve + ", " + yAve);
                  //update camera
                  //push to networktables
                  headingEntry.setDouble(hAve);
                  distanceEntry.setDouble(dAve);
                  objecyYawEntry.setDouble(yAve);
                  hAve = 0; dAve = 0; yAve = 0; //reset the averages
                  headingList.clear(); distanceList.clear(); objectYawList.clear(); //clear the lists
                }
                //add telemetry values
                Imgproc.putText(source, "Gracious Professionalism = " + Math.toDegrees(Math.random()) + "%", new Point(0,-14), 1, 1, new Scalar(0, 255, 0));
                Imgproc.putText(source, "Heading = " + Math.round(pose.heading) + " degrees", new Point(0,CAMERA_HEIGHT - 38), 1, 1, new Scalar(0, 255, 0));
                Imgproc.putText(source, "Distance = " + Math.round(pose.distance) + " in", new Point(0,CAMERA_HEIGHT - 24), 1, 1, new Scalar(0, 255, 0));
                Imgproc.putText(source, "Obj Yaw = " + Math.round(pose.objectYaw) + " degrees ", new Point(0,CAMERA_HEIGHT - 10), 1, 1, new Scalar(0, 255, 0));
              }
            }
          }else{
            objectSeen.setBoolean(false);
          }
          outputStream.putFrame(source);
          //more code here if ya want
      }));
      visionThread.start();
    
       /*
      VisionThread visionThread = new VisionThread(cameras.get(0),
              new GripPipeline(), pipeline -> {
        ...
      });
       */
      //VisionThread visionThread = new VisionThread(cameras.get(0),
      //new GripPipeline(), pipeline -> {
        //if(!pipeline.filterContoursOutput().isEmpty()){
          //System.out.println("Found contours");
        //}
      //});
      }
    // loop forever
    while(true){
      try {
        Thread.sleep(10000);
      } catch (InterruptedException ex) {
        return;
      }
    }
  }
}

