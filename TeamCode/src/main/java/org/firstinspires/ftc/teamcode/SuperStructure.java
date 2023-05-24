package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class SuperStructure {
   public static double ARM_INTAKE = 0.71;
   public static double ARM_FRONT = 0.49;
   public static double ARM_MIDDLE = 0.36;

   public static int LIFT_MIN = 0;
   public static int LIFT_LOW = 350;
   public static int LIFT_MID = 850;
   public static int LIFT_HIGH = 1325;
   public static int LIFT_ADD_PER_CONE = 50;

   public static final int LIFT_TOLERANCE = 15;

   private final DcMotorEx liftMotorLeft, liftMotorRight;
   private final Servo leftClaw, rightClaw, arm;
   private Runnable drive_period;
   private final LinearOpMode opMode;

   private int liftLocation;

   public SuperStructure(LinearOpMode opMode,
                         Runnable drivePeriod) {
      drive_period = drivePeriod;
      this.opMode = opMode;
      HardwareMap hardwareMap = opMode.hardwareMap;

      liftMotorLeft = hardwareMap.get(DcMotorEx.class, "leftLift");
      liftMotorRight = hardwareMap.get(DcMotorEx.class, "rightLift");
      liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

      liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      liftMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

      liftLocation = 0;
      leftClaw = hardwareMap.get(Servo.class, "leftClaw");
      rightClaw = hardwareMap.get(Servo.class, "rightClaw");
      arm = hardwareMap.get(Servo.class, "hand");
   }

   public void toOrigional() {
      liftLocation = 0;
      setLifterPosition(LIFT_MIN, 1);
      setArm(ARM_MIDDLE);
      while (getLifterPos() > LIFT_MID && opMode.opModeIsActive()) {
         drive_period.run();
      }
   }

   public void runtimeResetLifter() {
      setLifterPower(-0.2);
      sleep_with_drive(200);
      setLifterPower(0);
      sleep_with_drive(50);
      setLifterPower(0);
      liftMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      liftMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      setLifterPosition(0, 1);
   }

   public void toAim() {
      liftLocation = 0;
      setArm(ARM_INTAKE);
      openHand();
      setLifterPosition(LIFT_MIN, 1);
      sleep_with_drive(100);
   }

   public void toAim(int index) {
      liftLocation = 0;
      setArm(ARM_INTAKE);
      openHand();
      setLifterPosition(index * LIFT_ADD_PER_CONE, 1);
      sleep_with_drive(100);
   }

   public void toGroundJunction() {
      liftLocation = 0;
      setLifterPosition(LIFT_MIN, 1);
      while (Math.abs(getLifterPos() - LIFT_MIN) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
      setArm(ARM_INTAKE - 0.02);
   }

   public void toLowJunction() {
      liftLocation = 1;
      setLifterPosition(LIFT_LOW, 0.5);
      while (Math.abs(getLifterPos() - LIFT_LOW) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
      setArm(ARM_FRONT);
   }

   public void toMidJunction() {
      liftLocation = 2;
      setLifterPosition(LIFT_MID, 1);
      while (Math.abs(getLifterPos() - LIFT_MID) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
      setArm(ARM_FRONT);
   }

   public void toHighJunction() {
      liftLocation = 3;
      setLifterPosition(LIFT_HIGH, 1);
      while (Math.abs(getLifterPos() - LIFT_HIGH) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
      setArm(ARM_FRONT);
   }

   /**
    *  抬起滑轨，露出摄像头
    */
   public void toSeeJunction() {
      setArm(ARM_MIDDLE);
      setLifterPosition(200, 1);
      while (Math.abs(getLifterPos() - 200) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
      openHand();
   }

   /**
    * 夹取
    */
   public void grab() {
      closeHand();
      sleep_with_drive(150);
      setArm(ARM_MIDDLE);
   }

   public void verticalGrab(){
      closeHand();
      sleep_with_drive(150);
      setLifterPosition((int)getLifterPos()+200,1);
      while (Math.abs(getLifterPos() - 200) > LIFT_TOLERANCE && opMode.opModeIsActive()) {
         drive_period.run();
      }
      setArm(ARM_MIDDLE);
   }

   public void intakeSave(){
      armChange(-0.05);
      sleep_with_drive(80);
      openHand();
      sleep_with_drive(40);
      armChange(0.05);
      sleep_with_drive(50);
      closeHand();
   }

   public void openHand() {
      leftClaw.setPosition(0.27);
      rightClaw.setPosition(0.8);
   }

   public void closeHand() {
      leftClaw.setPosition(0.51 + 0.02);
      rightClaw.setPosition(0.56 - 0.02);
   }

   public void coneSaveR() {
      leftClaw.setPosition(0.5);
      rightClaw.setPosition(0.8);
      arm.setPosition(0.68);
   }

   public void coneSaveL() {
      leftClaw.setPosition(0.27);
      rightClaw.setPosition(0.56);
      arm.setPosition(0.68);
   }

   public void post_eject() {
      setLifterPower(0);
      setArm(ARM_MIDDLE);
   }

   public void setDrivePeriod(Runnable drivePeriod) {
      drive_period = drivePeriod;
   }

   public void setSideIsRed(boolean isRed) {

   }

   public void armChange(double val) {
      setArm(getArmPos() + val);
   }

   private void setLifterPosition(int pos, double power) {
      liftMotorLeft.setTargetPosition(pos);
      liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      liftMotorLeft.setPower(power);
      liftMotorRight.setTargetPosition(pos);
      liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      liftMotorRight.setPower(power);
   }

   private void setLifterPower(double power) {
      liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotorLeft.setPower(power);
      liftMotorRight.setPower(power);
      drive_period.run();
      liftMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
      liftMotorLeft.setPower(power);
      liftMotorRight.setPower(power);
   }

   public double getLifterPos() {
      return (double) (liftMotorLeft.getCurrentPosition() + liftMotorRight.getCurrentPosition()) / 2;
   }

   public void setArm(double val) {
      arm.setPosition(val);
   }

   public double getArmPos() {
      return arm.getPosition();
   }

   public void sleep_with_drive(double time_mm) {
      long start_time = System.currentTimeMillis();
      while (opMode.opModeIsActive() && System.currentTimeMillis() - start_time < time_mm) {
         drive_period.run();
      }
   }
}
