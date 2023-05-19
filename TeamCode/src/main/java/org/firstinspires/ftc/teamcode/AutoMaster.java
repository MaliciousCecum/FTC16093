package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Configurations.*;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Config
public abstract class AutoMaster extends LinearOpMode {

   public enum Junction {
      SIDE, MIDDLE, EMPTY
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

   private FTC16093MecanumDrive drive;
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

   Pose2d MIDDLE_EJECT_POS, GRAB_POS, SIDE_POS, startPos;

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
      MIDDLE_EJECT_POS = new Pose2d(1200, 174 * startSide, Math.toRadians(-117) * startSide);

      GRAB_POS = new Pose2d(x_axis, 1440 * startSide, Math.toRadians(90) * startSide);

      SIDE_POS = new Pose2d(x_axis, 995 * startSide, Math.toRadians(91.7) * startSide);
      startPos = new Pose2d(0, 900 * startSide, Math.toRadians(180) * startSide);
      end_pos_index = 0;
//      FtcDashboard.getInstance().startCameraStream(webcam, 10);
      telemetry.update();
      telemetry.addLine("init: drive");
      drive = new FTC16093MecanumDrive(hardwareMap);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.setPoseEstimate(startPos);
      drive.update();
      drive.getLocalizer().setPoseEstimate(startPos);
      drive.update();
      drive.getLocalizer().setPoseEstimate(startPos);
      telemetry.addLine("init: superstructure");
      telemetry.update();
      upper = new SuperStructure(
              this,
              drive::update
      );
      upper.setArm(0.36);
      upper.setSideIsRed(side_color);
      telemetry.addLine("init: trajectory");
      telemetry.update();
      if (firstJunctionPos == Junction.MIDDLE) {
         startToEject = drive.trajectoryBuilder(startPos)
                 .lineToLinearHeading(new Pose2d(1000, 930 * startSide, Math.toRadians(-180) * startSide))
                 .splineToSplineHeading(new Pose2d(x_axis, 560 * startSide, Math.toRadians(-90) * startSide), Math.toRadians(-90) * startSide)
                 .splineToSplineHeading(MIDDLE_EJECT_POS, MIDDLE_EJECT_POS.getHeading())
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
      drive.setSimpleMovePower(0.5);

//      webcam.closeCameraDeviceAsync(() -> {
//      });
   }

   protected void longMoveNormal() throws Exception {
      if (isStopRequested()) return;
      upper.closeHand();
      drive.followTrajectoryAsync(startToEject);
      if (firstJunctionPos == Junction.MIDDLE) {
         while (opModeIsActive() && Math.abs(drive.getPoseEstimate().getY() - MIDDLE_EJECT_POS.getY()) > 150) {
            drive.update();
         }
         upper.toHighJunction();
         drive.waitForIdle();
         drive.initSimpleMove(MIDDLE_EJECT_POS);
         drive.waitForIdle();
      } else {
         while (opModeIsActive() && drive.getPoseEstimate().getX() < 900) {
            drive.update();
         }
      }
      drive.moveForTime(startTimeDelay);
   }

   //到位等待，进行后面动作
   protected void longMoveDefensive() throws Exception {
      drive.initSimpleMove(new Pose2d(1410, startPos.getY(), startPos.getHeading()));
      drive.waitForIdle();
      drive.initSimpleMove(MIDDLE_EJECT_POS);
      drive.waitForIdle();
      drive.moveForTime(500);
      upper.toHighJunction();
   }

   Pose2d SIDE_ATTACK_POS;

   protected void firstConeAttack() throws Exception {
      SIDE_ATTACK_POS = new Pose2d(x_axis, startPos.getY(), startPos.getHeading());
      drive.initSimpleMove(SIDE_ATTACK_POS);

      while (opModeIsActive() && drive.getPoseEstimate().getX() < 500) {
         drive.update();
      }

      drive.initSimpleMove(new Pose2d(x_axis, 790 * startSide, startPos.getHeading()));

      upper.toHighJunction();
      drive.waitForIdle();
      drive.moveForTime(100);
      upper.armChange(0.06);
      drive.initSimpleMove(SIDE_POS);
      drive.waitForIdle();
      drive.moveForTime(300);
      upper.openHand();
      upper.post_eject();
   }

   protected void intake(int index, Junction lastJunction) throws Exception {
      upper.toOrigional();
      drive.initSimpleMove(new Pose2d(x_axis, MIDDLE_EJECT_POS.getY(),Math.toRadians(90)));
      drive.waitForIdle();
      drive.moveForTime(200);
      drive.initSimpleMove(GRAB_POS);
      upper.toAim(index);
      while (drive.isBusy()) {
         drive.update();
//         if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
      }
      drive.moveForTime(200);
      drive.stopSimpleMove();
      drive.setDrivePower(new Pose2d(0, 0, 0));
      if (cone_index < 2) {
         upper.closeHand();
         drive.setDrivePower(new Pose2d(-0.2, 0, 0));
         drive.moveForTime(250);
         upper.setArm(0.36);
      } else upper.grab();
      drive.setDrivePower(new Pose2d(-0.2, 0, 0));
      drive.moveForTime(200);
   }

   protected void moveToEject(){
      drive.initSimpleMove(new Pose2d(x_axis, MIDDLE_EJECT_POS.getY()+150*startSide,Math.toRadians(90)));
      drive.waitForIdle();
      drive.initSimpleMove(new Pose2d(
              x_axis, MIDDLE_EJECT_POS.getY()+150*startSide,
              MIDDLE_EJECT_POS.getHeading()));
      upper.toHighJunction();
      drive.waitForIdle();
      drive.initSimpleMove(MIDDLE_EJECT_POS);
      drive.waitForIdle();
      drive.moveForTime(200);
   }

   protected void eject(Junction pos, int stable_time) throws Exception {
//      if (runtime.seconds() > 28.8) throw new GlobalTimeoutException();
      drive.initSimpleMove(pos == Junction.MIDDLE ? MIDDLE_EJECT_POS : SIDE_POS);
      drive.waitForIdle();
      drive.moveForTime(stable_time);
      upper.armChange(0.1);
      drive.moveForTime(150);
      upper.openHand();
      drive.moveForTime(200);
      upper.post_eject();
      drive.moveForTime(200);
//      if (runtime.seconds() > 28.5) throw new GlobalTimeoutException();
   }

   protected void intakeSave() throws Exception {
      drive.moveForTime(200);
      upper.grab();
   }

   //倾覆
   private void tiltSave() throws InterruptedException {
      drive.setDrivePower(new Pose2d());
      drive.stopTrajectory();
      drive.update();
      throw new InterruptedException();
   }

   //停靠，起始位置x=1380
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
