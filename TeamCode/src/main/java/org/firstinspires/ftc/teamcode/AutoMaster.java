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

   Pose2d MIDDLE_FIRST_EJECT_POS, MIDDLE_EJECT_POS, GRAB_POS, SIDE_EJECT_POS, SIDE_FIRST_EJECT_POS, startPos;

   int end_pos_index;
   protected int cone_index;

   protected void initHardware() throws InterruptedException {
      MIDDLE_FIRST_EJECT_POS = new Pose2d(x_axis - 60, 110 * startSide, Math.toRadians(-135) * startSide);
      MIDDLE_EJECT_POS = new Pose2d(x_axis - 140, 0, Math.toRadians(180) * startSide);
      GRAB_POS = new Pose2d(x_axis, 1520 * startSide, Math.toRadians(90) * startSide);

      SIDE_EJECT_POS = new Pose2d(x_axis + 140, 600 * startSide, Math.toRadians(0) * startSide);
      SIDE_FIRST_EJECT_POS = new Pose2d(x_axis + 180, (900 - 170) * startSide, Math.toRadians(-45) * startSide);
      startPos = new Pose2d(0, 900 * startSide, 0);
      end_pos_index = 0;
//      PhotonCore.enable();
      telemetry.addLine("init: drive");
      telemetry.update();
      drive = new AutoMecanumDrive(hardwareMap);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.setPoseEstimate(startPos);
      drive.update();
      drive.getLocalizer().setPoseEstimate(startPos);
      drive.update();
      drive.getLocalizer().setPoseEstimate(startPos);
      drive.setSimpleMoveTolerance(50, Math.toRadians(3));
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
      if (firstJunctionPos == Junction.MIDDLE_HIGH) {
         startToEject = drive.trajectoryBuilder(startPos)
                 .lineToLinearHeading(new Pose2d(x_axis - 300, 900 * startSide, 0))
                 .splineToSplineHeading(new Pose2d(x_axis, 550 * startSide, Math.toRadians(-90) * startSide), Math.toRadians(-90) * startSide)
                 .splineToSplineHeading(MIDDLE_FIRST_EJECT_POS, MIDDLE_FIRST_EJECT_POS.getHeading())
                 .build();
      } else {
         startToEject = drive.trajectoryBuilder(startPos)
                 .lineToLinearHeading(new Pose2d(x_axis - 300, 900 * startSide, 0))
                 .splineToSplineHeading(SIDE_FIRST_EJECT_POS, SIDE_FIRST_EJECT_POS.getHeading())
                 .build();
      }
      telemetry.addLine("init: pipeline");
      telemetry.update();
      sleep(1000);
      AutoDetectionPipeline pipeline = new AutoDetectionPipeline(0.045, 578.272, 578.272, 402.145, 221.506);
      drive.webcam.setPipeline(pipeline);


      upper.closeHand();
      while (!opModeIsActive()) {
         int id = pipeline.getId();
         end_pos_index = id == 0 ? end_pos_index : id;
         long time = System.currentTimeMillis();
         telemetry.addData("pos", end_pos_index);
         telemetry.update();
         sleep(15);
         while (System.currentTimeMillis() - time < 100 && opModeInInit()) idle();
         if (isStopRequested()) throw new InterruptedException();
      }
      waitForStart();
      runtime = new ElapsedTime();
      runtime.reset();
      drive.setSimpleMovePower(0.7);
   }

   protected void longMoveNormal(int stableTime) throws Exception {
      if (isStopRequested()) return;
      upper.closeHand();
      drive.setJunctionMode();
      if (firstJunctionPos == Junction.MIDDLE_HIGH) {
         drive.followTrajectoryAsync(startToEject);
         while (opModeIsActive() && Math.abs(drive.getPoseEstimate().getY() - MIDDLE_FIRST_EJECT_POS.getY()) > 150) {
            drive.update();
         }
         upper.toHighJunction();
         drive.waitForIdle();
         drive.initSimpleMove(MIDDLE_FIRST_EJECT_POS);
      } else {
         drive.initSimpleMove(new Pose2d(x_axis, startPos.getY()));
         while (opModeIsActive() && drive.getPoseEstimate().getX() < 900) {
            drive.update();
         }
         drive.setSimpleMovePower(0.5);
         drive.initSimpleMove(SIDE_FIRST_EJECT_POS);
         upper.guideOut();
         upper.toHighJunction();
      }
      drive.waitForIdle();
      drive.moveForTime(stableTime);
   }

   protected void longMoveAttack() { //TODO
      drive.initSimpleMove(new Pose2d(x_axis, startPos.getY()));
      while (opModeIsActive() && drive.getPoseEstimate().getX() < 900) {
         drive.update();
      }
      drive.initSimpleMove(new Pose2d(x_axis + 300, startPos.getY(), Math.toRadians(-90) * startSide));
      drive.waitForIdle();
      upper.toHighJunction();
      drive.moveForTime(2000);
      drive.initSimpleMove(new Pose2d(x_axis + 300, startPos.getY() - 60 * startSide, Math.toRadians(-90) * startSide));
      upper.guideOut();
      drive.waitForIdle();
      drive.moveForTime(500);
   }

   protected void longMoveDefence() {
      drive.initSimpleMove(new Pose2d(x_axis, startPos.getY()));
      while (opModeIsActive() && drive.getPoseEstimate().getX() < 900) {
         drive.update();
      }
      drive.initSimpleMove(new Pose2d(x_axis + 130, startPos.getY(), Math.toRadians(-90) * startSide));
      drive.waitForIdle();
      drive.initSimpleMove(SIDE_FIRST_EJECT_POS);
      upper.toHighJunction();
      upper.guideOut();
      drive.waitForIdle();
      drive.moveForTime(200);
   }

   protected void intake(int index, Junction lastJunction) throws Exception {
      drive.setSimpleMovePower(0.5);
      //手放到可以看摄像头的地方
      drive.initSimpleMove(new Pose2d(x_axis, drive.getSimpleMovePosition().getY() + 200 * startSide, Math.toRadians(90) * startSide));
      drive.setRed(side_color);
      upper.toSeeJunction();
      //让车回到x中轴线上，y按照之前的
      drive.waitForIdle();
      drive.stopSimpleMove();
      while (lastJunction == Junction.SIDE_HIGH && drive.getPoseEstimate().getY() * startSide < 360) {
         drive.update();
         drive.setDrivePower(new Pose2d(0.3));
      }
      while (drive.getPoseEstimate().getY() * startSide < 1250) {
         drive.update();//持续更新车的位置
         if (drive.isWebcamDetected())
            drive.lineFollowPeriod(0.5);
         else
            drive.setDrivePower(new Pose2d(0.15));
      }
      drive.initSimpleMove(drive.getPoseEstimate());
      upper.toAim(index);//把手放到合适位置
      drive.stopSimpleMove();
      drive.setDrivePower(new Pose2d(0.1));
      while (!drive.isColorDetected()) {
         drive.update();
      }
      drive.setDrivePower(new Pose2d());
      drive.moveForTime(100);
      drive.getLocalizer().setPoseEstimate(new Pose2d(GRAB_POS.vec(), drive.getPoseEstimate().getHeading()));
      drive.stopSimpleMove();
      drive.setDrivePower(new Pose2d());
      upper.grab();
   }

   protected void moveToEject(Junction targetJunction) throws InterruptedException {
      drive.setSimpleMovePower(0.7);
      drive.setJunctionMode();
      drive.setSimpleMoveTolerance(50, Math.toRadians(5));
      if (targetJunction == Junction.MIDDLE_HIGH) {
         drive.initSimpleMove(new Pose2d(x_axis, 300 * startSide, Math.toRadians(90) * startSide));
         while (drive.getPoseEstimate().getY() * startSide > 900) {
            drive.update();
         }
         drive.setSimpleMovePower(0.6);
         drive.initSimpleMove(new Pose2d(x_axis, 0, Math.toRadians(180) * startSide));
         upper.toHighJunction();
         drive.waitForIdle();
         drive.initSimpleMove(MIDDLE_EJECT_POS);
      } else {
         drive.initSimpleMove(new Pose2d(x_axis, 600 * startSide, Math.toRadians(0) * startSide));
         upper.toHighJunction();
         drive.waitForIdle();
         drive.initSimpleMove(SIDE_EJECT_POS);
      }
      upper.guideOut();
      drive.setSimpleMoveTolerance(25, Math.toRadians(5));
      drive.waitForIdle();
      drive.setSimpleMoveTolerance(70, Math.toRadians(15));
//      drive.stopSimpleMove();
//      drive.setDrivePower(new Pose2d(0.3));
   }

   protected void eject(int stable_time) throws Exception {
      drive.moveForTime(stable_time);
      upper.armChange(0.1);
      upper.guideBack();
      drive.moveForTime(60);
      upper.openHand();
      drive.moveForTime(100);
      upper.post_eject();
   }

   /**
    * intake超时执行
    */
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
      drive.setSimpleMovePower(1);
      drive.initSimpleMove(endPos);
      drive.waitForIdle();
      drive.moveForTime(200);
   }

   /**
    * 坐标存入SCV文件
    */
   protected void savePosition() {
      save_pos_in_csv(drive.getPoseEstimate());
   }

   protected void breakPoint() throws InterruptedException {
      if (DEBUG) throw new InterruptedException();
   }
}
