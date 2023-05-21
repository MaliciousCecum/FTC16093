package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.control.PIDFController;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.OpenCVTest.XCYConeAimPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Config
public class AutoMecanumDrive extends FTC16093MecanumDrive{
   OpenCvWebcam webcam;
   XCYConeAimPipeline pipeline;
   public static final int WIDTH = 640;
   public static final int HEIGHT = 480;
   private boolean isRed;
   public static PIDCoefficients lfPIDCoeff = new PIDCoefficients(0.5, 0, 0.01);
   private final PIDFController lfPID;

   public AutoMecanumDrive(HardwareMap hardwareMap) {
      super(hardwareMap);
      int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
      pipeline = new XCYConeAimPipeline();
      webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
         @Override
         public void onOpened() {
            webcam.setPipeline(pipeline);
            webcam.startStreaming(WIDTH, HEIGHT, OpenCvCameraRotation.UPSIDE_DOWN);
         }

         @Override
         public void onError(int errorCode) {
         }
      });
      FtcDashboard.getInstance().startCameraStream(webcam, 10);
      webcam.getExposureControl().setMode(ExposureControl.Mode.Manual);
      webcam.getWhiteBalanceControl().setMode(WhiteBalanceControl.Mode.AUTO);
      lfPID = new PIDFController(lfPIDCoeff);
      lfPID.setTargetPosition(0);
   }

   public void setRed(boolean red) {
      isRed = red;
   }

   public boolean isDetected(){
      return pipeline.isDetected();
   }
   public void lineFollowPeriod(double powerX){
      setDrivePower(new Pose2d(
              powerX,
              0,
              lfPID.update(-pipeline.getOffset()/250)
      ));
   }
}
