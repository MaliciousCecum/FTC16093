package org.firstinspires.ftc.teamcode.OpenCVTest;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

@Config
public class XCYAimPipeline extends OpenCvPipeline {
   Mat process_mat = new Mat();
   Mat output_mat = new Mat();
   List<MatOfPoint> contoursList = new ArrayList<>();

   private boolean is_detected = false;
   private double cone_x_offset = 0;
   private double angle_factor = 0;
   public static int coneAimThresh = 170;
   public static int junctionThresh = 70;
   public static int blur_pix = 5;
   public static int min_detect_area = 500;

   private boolean isSideRed = false;
   private boolean junctionMode = false;

   public static boolean cb_blur_img = false;
   public static boolean thresh_img = false;
   public static boolean contour = false;
   public static boolean data = false;


   public void setSideRed(boolean isRed) {
      isSideRed = isRed;
      junctionMode = false;
   }

   @Override
   public Mat processFrame(Mat input) {
      contoursList.clear();
      Imgproc.cvtColor(input, process_mat, Imgproc.COLOR_RGB2YCrCb);

      if (isSideRed && !junctionMode)
         Core.extractChannel(process_mat, process_mat, 1);
      else
         Core.extractChannel(process_mat, process_mat, 2);

      Imgproc.blur(process_mat, process_mat, new Size(blur_pix, blur_pix));

      input.copyTo(output_mat);

      if (cb_blur_img)
         process_mat.copyTo(output_mat);

      if (junctionMode)
         Imgproc.threshold(process_mat, process_mat, junctionThresh, 255, Imgproc.THRESH_BINARY_INV);
      else
         Imgproc.threshold(process_mat, process_mat, coneAimThresh, 255, Imgproc.THRESH_BINARY);

      if (thresh_img)
         process_mat.copyTo(output_mat);

      Imgproc.findContours(process_mat, contoursList, new Mat(), Imgproc.RETR_LIST, Imgproc.CHAIN_APPROX_SIMPLE);

      if (contour)
         Imgproc.drawContours(output_mat, contoursList, -1, new Scalar(0, 0, 255), 2, 8);

      contoursList.sort((o1, o2) -> (int) (Imgproc.contourArea(o2) - Imgproc.contourArea(o1)));

      if (contoursList.size() > 0 && Imgproc.contourArea(contoursList.get(0)) > min_detect_area) {
         RotatedRect rec = Imgproc.minAreaRect(new MatOfPoint2f(contoursList.get(0).toArray()));
         Point[] boxPoints = new Point[4];
         rec.points(boxPoints);
         double angle;
         Point pointA = midpoint(boxPoints[0], boxPoints[1]);
         Point pointB = midpoint(boxPoints[1], boxPoints[2]);
         Point pointC = midpoint(boxPoints[2], boxPoints[3]);
         Point pointD = midpoint(boxPoints[3], boxPoints[0]);
         if (getDistance(pointA, pointC) > getDistance(pointB, pointD))
            angle = Math.atan2(pointA.y - pointC.y, pointA.x - pointC.x);
         else
            angle = Math.atan2(pointB.y - pointD.y, pointB.x - pointD.x);

         setData(true,
                 process_mat.cols() * 0.5 - rec.center.x,
                 AngleUnit.normalizeDegrees(-90 - Math.toDegrees(angle))
         );
      } else {
         setData(false, 0, 0);
      }

      if (data) {
         for (int c = 0; c < contoursList.size(); c++) {
            int area = (int) Imgproc.contourArea(contoursList.get(c));
            if (area < min_detect_area) {
               continue;
            }

            RotatedRect rec = Imgproc.minAreaRect(new MatOfPoint2f(contoursList.get(c).toArray()));
            Point[] boxPoints = new Point[4];
            rec.points(boxPoints);
            if (c == 0) {
               Imgproc.line(output_mat,new Point(rec.center.x,0),new Point(rec.center.x,480),new Scalar(127, 255, 0));
               Imgproc.line(output_mat,new Point(0,rec.center.y),new Point(640,rec.center.y),new Scalar(127, 255, 0));
            }
            Point junctionTextPos = new Point(500, 200);
            Point coneStackTextPos = new Point(500, 280);
            for (int i = 0; i <= 3; i++) {
               if (c == 0) {
                  Imgproc.line(output_mat, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(127, 255, 0), 1);
                  Imgproc.line(output_mat, boxPoints[i], junctionMode ? junctionTextPos : coneStackTextPos, new Scalar(127, 255, 0), 2);
               }
               Imgproc.line(output_mat, boxPoints[i], boxPoints[(i + 1) % 4], new Scalar(150, 150, 150));
            }
            Imgproc.putText(output_mat, "i=" + c + ",a=" + area,
                    boxPoints[2], Imgproc.FONT_HERSHEY_COMPLEX, 0.5, new Scalar(0, 0, 255));
            Imgproc.putText(output_mat, "连接点", junctionTextPos, Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(127, 255, 0));
            Imgproc.putText(output_mat, "锥筒", coneStackTextPos, Imgproc.FONT_HERSHEY_PLAIN, 2, new Scalar(127, 255, 0));
         }
      }
      return output_mat;
   }

   private synchronized void setData(boolean detected, double offset, double angle) {
      is_detected = detected;
      cone_x_offset = offset;
      angle_factor = angle;
   }

   public synchronized boolean isDetected() {
      return is_detected;
   }

   public synchronized double getOffset() {
      return cone_x_offset;
   }

   public synchronized double getAngle() {
      return angle_factor;
   }

   private Point midpoint(Point ptA, Point ptB) {
      return new Point((ptA.x + ptB.x) * 0.5, (ptA.y + ptB.y) * 0.5);
   }

   private int getDistance(Point pointA, Point pointB) {
      return (int) (Math.hypot(pointA.x - pointB.x, pointA.y - pointB.y));
   }

   public void openJunctionMode() {
      this.junctionMode = true;
   }
}
