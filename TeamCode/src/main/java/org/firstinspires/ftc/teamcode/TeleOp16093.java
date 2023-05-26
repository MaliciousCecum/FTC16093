package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Configurations.clamp;
import static org.firstinspires.ftc.teamcode.Configurations.get_pos_from_csv;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@Config
@TeleOp
public class TeleOp16093 extends LinearOpMode {

   private static final double max_turn_assist_power = 0.4;
   private NanoClock time;

   private double global_drive_power = 1;
   private double global_drive_turn_ratio = 1;

   public static  double x_static_compensation = 0.06;
   public static double y_static_compensation = 0.06;
   public static double webcamPower = 0.5;

   private AutoMecanumDrive drive;
   private Pose2d current_pos;
   private Sequence sequence;
   private boolean isConeSaveMode;

   enum Sequence {
      EMPTY_MIDDLE, EMPTY_DOWN, HOLDING_AWAIT, HOLDING_UP
   }

   @Override
   public void runOpMode() throws InterruptedException {
      time = NanoClock.system();
      sequence = Sequence.EMPTY_MIDDLE;
      SuperStructure upper = new SuperStructure(
              this,
              () -> {
                 logic_period();
                 drive_period();
              });
      drive = new AutoMecanumDrive(hardwareMap);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
      drive.setJunctionMode();
      drive.getLocalizer().setPoseEstimate(get_pos_from_csv());
      boolean holding = false;
      isConeSaveMode = false;
      XCYBoolean color_detected = new XCYBoolean(() -> drive.isColorDetected());
      XCYBoolean resetIntake = new XCYBoolean(() -> gamepad1.touchpad && gamepad1.triangle);
      XCYBoolean cone_save = new XCYBoolean(() -> gamepad1.start);
      XCYBoolean intake_save = new XCYBoolean(() -> gamepad1.share);
      XCYBoolean arm_down = new XCYBoolean(() -> (gamepad1.cross || (gamepad1.right_bumper && sequence == Sequence.EMPTY_MIDDLE)) && !gamepad1.share);
      XCYBoolean intake_action = new XCYBoolean(() -> (gamepad1.circle || (gamepad1.right_bumper && sequence == Sequence.EMPTY_DOWN)) && !gamepad1.share);
      XCYBoolean upper_release = new XCYBoolean(() -> (gamepad1.square || (gamepad1.right_bumper && sequence == Sequence.HOLDING_UP)) && !gamepad1.share);
      XCYBoolean high_j = new XCYBoolean(() -> (gamepad1.triangle || (gamepad1.right_bumper && sequence == Sequence.HOLDING_AWAIT)) && !gamepad1.share);
      XCYBoolean low_j = new XCYBoolean(() -> !gamepad1.share && gamepad1.dpad_down);
      XCYBoolean mid_j = new XCYBoolean(() -> !gamepad1.share && gamepad1.dpad_up);
      XCYBoolean setOriginal = new XCYBoolean(() -> gamepad1.touchpad && gamepad1.square);
      XCYBoolean activate_cone_save = new XCYBoolean(()-> isConeSaveMode);
      boolean useSensor = true;
//      to_last_eject = new XCYBoolean(() -> gamepad1.left_bumper && holding);

//      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        upper.setAimReleaseCondition(dpad_down, dpad_up, upper_release, button_y);
//        upper.setWebcamConfig();

      Gamepad.LedEffect effect_control = new Gamepad.LedEffect.Builder()
              .addStep(1, 0, 0, 65)
              .addStep(0.5, 0.5, 0, 65)
              .addStep(0, 1, 0, 65)
              .addStep(0, 0.5, 0.5, 65)
              .addStep(0, 0, 1, 65)
              .addStep(0.5, 0, 0.5, 65)
              .setRepeating(true)
              .build();

      Gamepad.LedEffect effect_idle = new Gamepad.LedEffect.Builder()
              .addStep(0.3, 0.2, 0.1, 700)
              .addStep(0.1, 0.2, 0.3, 700)
              .setRepeating(true)
              .build();

      waitForStart();
      logic_period();
      int intakeAimPosIndex = 4;
//      lastEjectPos = current_pos;

      while (opModeIsActive()) {
         logic_period();
         drive_period();

         if (gamepad1.touchpad && gamepad1.share){
            useSensor = false;
         }

         if (activate_cone_save.toTrue()){
            gamepad2.runLedEffect(effect_control);
            gamepad1.runLedEffect(effect_idle);
         } else if (activate_cone_save.toFalse()){
            gamepad1.runLedEffect(effect_control);
            gamepad2.runLedEffect(effect_idle);
         }

         if (gamepad2.touchpad){
            isConeSaveMode = true;
         }

         if (resetIntake.toTrue()) {
            upper.runtimeResetLifter();
         }
//         if (reset_imu.toFalse()) {
//            drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
//            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
//         }

         if (cone_save.toTrue()) {
            upper.coneSaveR();
            upper.armChange(-0.1);
            global_drive_power = 0.55;
            global_drive_turn_ratio = 0.3;
         } else if (cone_save.toFalse()) {
            upper.armChange(0.1);
         }

         if (intake_save.toTrue()) {
            upper.intakeSave();
         }

         if (arm_down.toTrue()) {
            isConeSaveMode = false;
            global_drive_power = 0.7;
            global_drive_turn_ratio = 0.3;
            upper.toAim();
            upper.sleep_with_drive(300);
            holding = false;
            sequence = Sequence.EMPTY_DOWN;
         }

         if (color_detected.toTrue() && sequence == Sequence.EMPTY_DOWN && !isConeSaveMode) {
            upper.grab();
            holding = true;
            sequence = Sequence.HOLDING_AWAIT;
            global_drive_power = 1;
            global_drive_turn_ratio = 1;
            upper.sleep_with_drive(300);
         }

         if (intake_action.toTrue()) {
            if (holding) {
               upper.toGroundJunction();
               global_drive_power = 0.6;
               global_drive_turn_ratio = 0.3;
            } else {
               upper.grab();
               holding = true;
               sequence = Sequence.HOLDING_AWAIT;
               gamepad1.runLedEffect(effect_control);
               global_drive_power = 1;
               global_drive_turn_ratio = 1;
            }
         }

         if (!holding) {
            if (mid_j.toTrue()) {
               intakeAimPosIndex = Range.clip(intakeAimPosIndex + 1, 0, 4);
               upper.toAim(intakeAimPosIndex);
               global_drive_power = 0.7;
               global_drive_turn_ratio = 0.3;
            } else if (low_j.toTrue()) {
               intakeAimPosIndex = Range.clip(intakeAimPosIndex - 1, 0, 4);
               upper.toAim(intakeAimPosIndex);
               global_drive_power = 0.7;
               global_drive_turn_ratio = 0.3;
            }
         } else {
            if (mid_j.toTrue() || low_j.toTrue() || high_j.toTrue()) {
               if (mid_j.toTrue()) {
                  upper.toMidJunction();
                  global_drive_power = 1;
                  global_drive_turn_ratio = 0.4;
               } else if (low_j.toTrue()) {
                  upper.toLowJunction();
                  global_drive_power = 1;
                  global_drive_turn_ratio = 0.4;
               } else if (high_j.toTrue()) {
                  upper.toHighJunction();
                  global_drive_power = 0.9;
                  global_drive_turn_ratio = 0.35;
               }
               upper.sleep_with_drive(50);
               upper.guideOut();
               sequence = Sequence.HOLDING_UP;
               logic_period();
               while (upper_release.get() || high_j.get() || mid_j.get() || low_j.get()) {
                  logic_period();
                  drive_period();
               }
//               while (!(upper_release.toTrue() || high_j.get() || mid_j.get() || low_j.get() || intake_action.get())) {
//                  logic_period();
//                  drive_period();
//                  if (intake_save.toTrue()){
//                     upper.intakeSave();
//                  }
//               }
               long startTime;
//               if (upper_release.get())
               do {
                  while (!(upper_release.toTrue() || high_j.get() || mid_j.get() || low_j.get() || intake_action.get())) {
                     logic_period();
                     if (useSensor) {
                        webcam_drive_period();
                     }
                     else {
                     drive_period();}

                     if (Math.abs(drive.getOffSet())<140){
                        upper.guideOut();
                     } else {
                        upper.guideBack();
                     }
                  }
                  startTime = System.currentTimeMillis();
                  if (upper_release.toTrue()) {
                     upper.armChange(0.1);
                     upper.sleep_with_drive(50);
                     upper.guideBack();
                     while (upper_release.get()) {
                        logic_period();
                        drive_period();
                     }
                  }
                  if (System.currentTimeMillis() - startTime > 450) {
                     upper.armChange(-0.1);
                     upper.guideOut();
                  }
               } while (System.currentTimeMillis() - startTime > 450);
               global_drive_turn_ratio = 1;
               global_drive_power = 1;
            }
            upper.guideBack();
         }

         if (upper_release.toFalse()) {
            //x
            upper.guideBack();
            isConeSaveMode = false;
            if (sequence == Sequence.HOLDING_AWAIT) {
               upper.toGroundJunction();
               sequence = Sequence.HOLDING_UP;
            } else if (sequence == Sequence.HOLDING_UP) {
//               lastEjectPos = current_pos;
               upper.openHand();
               upper.sleep_with_drive(350);
               upper.post_eject();
               upper.sleep_with_drive(100);
               global_drive_turn_ratio = 1;
               global_drive_power = 1;
               holding = false;
               sequence = Sequence.EMPTY_MIDDLE;
               upper.toOrigional();
            } else {
               upper.toOrigional();
               global_drive_turn_ratio = 1;
               global_drive_power = 1;
               holding = false;
            }
         }
         if (setOriginal.toFalse()) {
            drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
         }
      }
   }

   private double last_time_sec;
   private double period_time_sec;

   private void logic_period() {
      XCYBoolean.bulkRead();
      current_pos = drive.getPoseEstimate();
      period_time_sec = time.seconds() - last_time_sec;
      telemetry.addData("elapse time", period_time_sec * 1000);
      last_time_sec = time.seconds();
      telemetry.update();
   }

   private void drive_period() {
      if (isConeSaveMode){
         double x = -gamepad2.left_stick_y * 0.35 + -gamepad2.right_stick_y * 0.65;
         double y = -gamepad2.left_stick_x * 0.35 + -gamepad2.right_stick_x * 0.65;
         double turn_val = (gamepad2.left_trigger - gamepad2.right_trigger);
         Pose2d power = (new Pose2d(x, y, turn_val * global_drive_turn_ratio)).times(global_drive_power);
         drive.setGlobalPower(power, x_static_compensation, y_static_compensation);
      } else {
         double x = -gamepad1.left_stick_y * 0.35 + -gamepad1.right_stick_y * 0.65;
         double y = -gamepad1.left_stick_x * 0.35 + -gamepad1.right_stick_x * 0.65;
         double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
         Vector2d fast_stick = new Vector2d(-gamepad1.right_stick_y, -gamepad1.right_stick_x);
         double corrected_rad = fast_stick.angle() - current_pos.getHeading();
         while (corrected_rad > Math.PI / 2) corrected_rad -= Math.PI;
         while (corrected_rad < -Math.PI / 2) corrected_rad += Math.PI;
         if (Math.abs(corrected_rad) < Math.PI / 5) {
            double div = clamp(
                    Math.toDegrees(corrected_rad) / 20, 1)
                    * max_turn_assist_power * fast_stick.norm();
            turn_val += clamp(div, Math.max(0, Math.abs(div) - Math.abs(turn_val)));
         }
         Pose2d power = (new Pose2d(x, y, turn_val * global_drive_turn_ratio)).times(global_drive_power);
         drive.setGlobalPower(power, x_static_compensation, y_static_compensation);
      }
      drive.update();
   }

   private void webcam_drive_period() {
      double x = -gamepad1.left_stick_y * 0.35 + -gamepad1.right_stick_y * 0.65;
      double y = -gamepad1.left_stick_x * 0.35 + -gamepad1.right_stick_x * 0.65;
      double turn_val = (gamepad1.left_trigger - gamepad1.right_trigger);
      if (Math.abs(turn_val) < 0.001&&Math.abs(drive.getOffSet())>150) {
         turn_val = drive.getOffSet() / 250 * webcamPower;
      }
      Pose2d power = (new Pose2d(x, y, turn_val * global_drive_turn_ratio)).times(global_drive_power);
      drive.setGlobalPower(power, x_static_compensation, y_static_compensation);
      drive.update();
   }
}
