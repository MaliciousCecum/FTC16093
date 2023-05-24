package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class TestTeleOp extends LinearOpMode {
   DcMotorEx leftLift;
   DcMotorEx rightLift;
   Servo rightClaw;
   Servo leftClaw;
   Servo hand;
   public static double armPos = 0.68;
   public static double speed = 0.3;

   @Override
   public void runOpMode() {
      leftLift = hardwareMap.get(DcMotorEx.class, "leftLift");
      rightLift = hardwareMap.get(DcMotorEx.class, "rightLift");
      leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      leftLift.setDirection(DcMotorSimple.Direction.REVERSE);
      XCYBoolean drive_auto_move = new XCYBoolean(() -> gamepad1.back);
      AutoMecanumDrive drive = new AutoMecanumDrive(hardwareMap);
      rightClaw = hardwareMap.get(Servo.class, "rightClaw");
      leftClaw = hardwareMap.get(Servo.class, "leftClaw");
      hand = hardwareMap.get(Servo.class, "hand");
      waitForStart();
      Pose2d drive_pos = new Pose2d();
      drive.setPoseEstimate(new Pose2d(0, 0, 0));
      while (opModeIsActive()) {
         if (gamepad2.start && drive.isWebcamDetected()) {
            drive.lineFollowPeriod(gamepad2.left_stick_y*-0.25);
         } else {
            double x = -gamepad1.left_stick_y;
            double y = -gamepad1.right_stick_x;
            double turn = gamepad1.left_trigger - gamepad1.right_trigger;
            drive.setDrivePower(new Pose2d(x, y, turn).times(speed));
         }
//         DcMotorEx motor = hardwareMap.get(DcMotorEx.class,"rf");
//         motor.setPower(gamepad1.right_stick_y);

//         if (gamepad1.a) {
//            leftLift.setPower(0.8);
//            rightLift.setPower(0.8);
//         } else if (gamepad1.b) {
//            leftLift.setPower(-0.5);
//            rightLift.setPower(-0.5);
//         } else {
//            leftLift.setPower(0);
//            rightLift.setPower(0);
//         }
         if(gamepad1.touchpad){
            drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
         }
         if (gamepad1.x) {
            leftClaw.setPosition(0.27);
            rightClaw.setPosition(0.8);
         } else if (gamepad1.y) {
            leftClaw.setPosition(0.51);
            rightClaw.setPosition(0.56);
         }

         if (gamepad1.dpad_up) {
            setLifter(1000);
         } else if (gamepad1.dpad_down) {
            setLifter(10);
         }

         if (gamepad1.left_bumper) {
            hand.setPosition(armPos);
         } else if (gamepad1.right_bumper) {
            hand.setPosition(0.35);
         }

         drive.update();
         XCYBoolean.bulkRead();
         if (gamepad1.start) {
            drive.setPoseEstimate(new Pose2d());
            drive.getLocalizer().setPoseEstimate(new Pose2d());
            drive_pos = drive.getPoseEstimate();
         }
         if (drive_auto_move.toTrue()) {
            drive.initSimpleMove(drive_pos);
            while (drive_auto_move.get()) {
               XCYBoolean.bulkRead();
               drive.update();
            }
            drive.stopSimpleMove();
         }
      }
   }

   private void setLifter(int pos) {
      rightLift.setTargetPosition(pos);
      rightLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      rightLift.setPower(0.6);
      leftLift.setTargetPosition(pos);
      leftLift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      leftLift.setPower(0.6);
   }
}
