package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Configurations.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public abstract class AutoMaster extends LinearOpMode {

   public enum Junction {
      SIDE_HIGH, MIDDLE_HIGH, LOW
   }

   private ElapsedTime runtime;
   public static final int RIGHT = -1;
   public static final int LEFT = 1;
   public static boolean DEBUG = true;
   public static int startTimeDelay = 300, cycleDelay = -1;

   public static final boolean RED = true;
   public static final boolean BLUE = false;

   protected Junction firstJunctionPos;
   protected int startSide;
   protected boolean side_color;

   public static double x_axis = 1300;

   private AutoMecanumDrive drive;
   private SuperStructure upper;

   Pose2d[] RIGHT_END_POSITIONS = {
           new Pose2d(x_axis, -300, Math.toRadians(-90)),
           new Pose2d(x_axis, -300, Math.toRadians(-90)),
           new Pose2d(x_axis, -900, Math.toRadians(-90)),
           new Pose2d(x_axis, -1500, Math.toRadians(-90))
   };

   Pose2d[] LEFT_END_POSITIONS = {
           new Pose2d(x_axis, 300, Math.toRadians(90)),
           new Pose2d(x_axis, 1500, Math.toRadians(90)),
           new Pose2d(x_axis, 900, Math.toRadians(90)),
           new Pose2d(x_axis, 300, Math.toRadians(90))
   };
   Trajectory startToEject;

   Pose2d MIDDLE_EJECT_FIRST_POS, GRAB_POS, SIDE_POS, startPos;

   int end_pos_index;
   protected int cone_index;

   protected void initHardware() throws InterruptedException {
//      PhotonCore.enable();
      telemetry.addLine("init: webcam");
      telemetry.update();
//      OpenCvWebcam webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),
//              hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()));
//      AutoDetectionPipeline pipeline = new AutoDetectionPipeline(0.045, 578.272, 578.272, 402.145, 221.506);
//      webcam.setPipeline(pipeline);
//      webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
//         @Override
//         public void onOpened() {
//            webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
//         }
//
//         @Override
//         public void onError(int errorCode) {
//         }
//      });
      MIDDLE_EJECT_FIRST_POS = new Pose2d(1200, 174 * startSide, Math.toRadians(-125) * startSide);

      GRAB_POS = new Pose2d(x_axis, 1460 * startSide, Math.toRadians(90) * startSide);

      SIDE_POS = new Pose2d(x_axis, 995 * startSide, Math.toRadians(91.7) * startSide);
      startPos = new Pose2d(0, 900 * startSide, 0);
      end_pos_index = 0;
//      FtcDashboard.getInstance().startCameraStream(webcam, 10);
      telemetry.update();
      telemetry.addLine("init: drive");
      drive = new AutoMecanumDrive(hardwareMap);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.setPoseEstimate(startPos);
      drive.update();
      drive.getLocalizer().setPoseEstimate(startPos);
      drive.update();
      drive.getLocalizer().setPoseEstimate(startPos);
      drive.setSimpleMoveTolerance(30, Math.toRadians(10));
      telemetry.addLine("init: superstructure");
      telemetry.update();
      upper = new SuperStructure(
              this,
              drive::update
      );
      upper.toSeeJunction();
      upper.setSideIsRed(side_color);
      telemetry.addLine("init: trajectory");
      telemetry.update();
      if (firstJunctionPos == Junction.SIDE_HIGH) {
         startToEject = drive.trajectoryBuilder(startPos)
                 .lineToLinearHeading(new Pose2d(x_axis - 300, 900 * startSide, 0))
                 .splineToSplineHeading(new Pose2d(x_axis, 550 * startSide, Math.toRadians(-90) * startSide), Math.toRadians(-90) * startSide)
                 .splineToSplineHeading(MIDDLE_EJECT_FIRST_POS, MIDDLE_EJECT_FIRST_POS.getHeading())
                 .build();
      } else {
         startToEject = drive.trajectoryBuilder(startPos)
                 .lineToSplineHeading(SIDE_POS)
                 .build();
      }
      telemetry.addLine("init: pipeline");
      telemetry.update();
      runtime = new ElapsedTime();
      runtime.reset();
      sleep(500);
      upper.closeHand();
      while (!opModeIsActive()) {
//         int id = pipeline.getId();
//         end_pos_index = id == 0 ? end_pos_index : id;
         end_pos_index = 0;
         long time = System.currentTimeMillis();
         telemetry.addData("pos", end_pos_index);
         telemetry.update();
         sleep(15);
         while (System.currentTimeMillis() - time < 100 && opModeInInit()) idle();
         if (isStopRequested()) throw new InterruptedException();
      }
      waitForStart();
      runtime.reset();
      drive.setSimpleMovePower(0.7);

//      webcam.closeCameraDeviceAsync(() -> {
//      });
   }

   protected void longMoveNormal() throws Exception {
      if (isStopRequested()) return;
      upper.closeHand();
      drive.followTrajectoryAsync(startToEject);
      if (firstJunctionPos == Junction.SIDE_HIGH) {
         while (opModeIsActive() && Math.abs(drive.getPoseEstimate().getY() - MIDDLE_EJECT_FIRST_POS.getY()) > 150) {
            drive.update();
         }
         upper.toHighJunction();
         drive.waitForIdle();
         drive.initSimpleMove(MIDDLE_EJECT_FIRST_POS);
         drive.waitForIdle();
      } else {
         while (opModeIsActive() && drive.getPoseEstimate().getX() < 900) {
            drive.update();
         }
      }
      drive.moveForTime(startTimeDelay);
   }

   protected void intake(int index, Junction lastJunction) throws Exception {
      drive.setSimpleMovePower(0.5);
      upper.toSeeJunction();
      //手放到可以看摄像头的地方
      drive.initSimpleMove(new Pose2d(x_axis, drive.getSimpleMovePosition().getY(), Math.toRadians(90) * startSide));
      //让车回到x中轴线上，y按照之前的
      drive.waitForIdle();
      drive.moveForTime(200);//停稳
      drive.stopSimpleMove();
      while (drive.getPoseEstimate().getY()*startSide<1100) {
         drive.update();//持续更新车的位置
         drive.lineFollowPeriod(0.5);
      }
      upper.toAim(index);//把手放到合适位置
      drive.setDrivePower(new Pose2d(0, 0.25, 0));
      while (!drive.isColorDetected()) {
         drive.update();
      }
      drive.getLocalizer().setPoseEstimate(new Pose2d(GRAB_POS.vec(),drive.getPoseEstimate().getHeading()));
      drive.initSimpleMove(GRAB_POS);
      drive.moveForTime(150);
      if (cone_index < 2) {
         upper.closeHand();//夹起
         drive.stopSimpleMove();
         drive.setDrivePower(new Pose2d(-0.3, 0, 0));
         drive.moveForTime(200);//后退
         upper.setArm(0.36);
      } else upper.verticalGrab();
   }

   protected void moveToEject() {
      drive.setSimpleMovePower(1);
      drive.initSimpleMove(new Pose2d(x_axis, 950 * startSide, Math.toRadians(90) * startSide));
      drive.waitForIdle();
      drive.setSimpleMovePower(0.7);
      drive.initSimpleMove(MIDDLE_EJECT_FIRST_POS);
      upper.toHighJunction();
      drive.waitForIdle();
   }

   protected void eject(Junction pos, int stable_time) throws Exception {
      drive.moveForTime(stable_time);
      upper.armChange(0.1);
      drive.moveForTime(100);
      upper.openHand();
      drive.moveForTime(200);
      upper.post_eject();
      drive.moveForTime(100);
   }

   protected void intakeSave() throws Exception {
      drive.stopSimpleMove();
      drive.setDrivePower(new Pose2d(0.2));
      drive.moveForTime(200);
      upper.grab();
   }

   /**
    * 停靠
    */
   protected void park() {
      Pose2d endPos;
      if (startSide == RIGHT) {
         endPos = (RIGHT_END_POSITIONS[end_pos_index]);
      } else {
         endPos = (LEFT_END_POSITIONS[end_pos_index]);
      }
      drive.initSimpleMove(endPos);
      drive.waitForIdle();
      drive.moveForTime(200);
   }

   protected void savePosition() {
      save_pos_in_csv(drive.getPoseEstimate());
   }

   protected void breakPoint() throws InterruptedException {
      if (DEBUG) throw new InterruptedException();
   }
}
