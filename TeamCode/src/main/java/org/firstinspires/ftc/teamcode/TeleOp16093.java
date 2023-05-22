package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Configurations.clamp;

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

   public static String KEY_CODE = "default";

   private static final double max_turn_assist_power = 0.4;
   private NanoClock time;

   private double global_drive_power = 1;
   private double global_drive_turn_ratio = 1;
   private SuperStructure upper;

   public static final double x_static_compensation = 0.06;
   public static final double y_static_compensation = 0.06;

   private BasicMecanumDrive drive;
   private Pose2d current_pos;
   //   private Vector2d cone_pos_vec = new Vector2d(50, -1200);
//   private XCYBoolean to_last_eject;
   private boolean holding;
   private Sequence sequence;

   enum Sequence {
      EMPTY_MIDDLE, EMPTY_DOWN, HOLDING_AWAIT, HOLDING_UP
   }

   @Override
   public void runOpMode() throws InterruptedException {
      time = NanoClock.system();
      sequence = Sequence.EMPTY_MIDDLE;
      upper = new SuperStructure(
              this,
              () -> {
                 logic_period();
                 drive_period();
              });
      drive = new BasicMecanumDrive(hardwareMap);
      drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//      drive.getLocalizer().setPoseEstimate(get_pos_from_csv());
      holding = false;
      XCYBoolean arm_down;
      XCYBoolean intake_action;
      XCYBoolean upper_release;
      XCYBoolean high_j;
      XCYBoolean low_j;
      XCYBoolean mid_j;
      XCYBoolean cone_saveL;
      XCYBoolean cone_saveR;
      XCYBoolean setOriginal;
      cone_saveL = new XCYBoolean(() -> gamepad1.back);
      cone_saveR = new XCYBoolean(() -> gamepad1.start);

      switch (KEY_CODE) {
         case "XCY":
            arm_down = new XCYBoolean(() -> (gamepad1.cross || (gamepad1.right_bumper && sequence == Sequence.EMPTY_MIDDLE)) && !gamepad1.share);
            intake_action = new XCYBoolean(() -> (gamepad1.circle || (gamepad1.right_bumper && sequence == Sequence.EMPTY_DOWN)) && !gamepad1.share);
            upper_release = new XCYBoolean(() -> (gamepad1.square || (gamepad1.right_bumper && sequence == Sequence.HOLDING_UP)) && !gamepad1.share);
            high_j = new XCYBoolean(() -> (gamepad1.triangle || (gamepad1.right_bumper && sequence == Sequence.HOLDING_AWAIT)) && !gamepad1.share);
            low_j = new XCYBoolean(() -> !gamepad1.share && (gamepad2.dpad_down || gamepad1.dpad_down));
            mid_j = new XCYBoolean(() -> !gamepad1.share && (gamepad2.dpad_up || gamepad1.dpad_up));
            setOriginal = new XCYBoolean(() -> gamepad1.touchpad);
            break;
         default:
            arm_down = new XCYBoolean(() -> gamepad1.right_bumper);
            intake_action = new XCYBoolean(() -> gamepad1.left_bumper);
            upper_release = new XCYBoolean(() -> gamepad1.x);
            high_j = new XCYBoolean(() -> gamepad1.y);
            low_j = new XCYBoolean(() -> gamepad1.a);
            mid_j = new XCYBoolean(() -> gamepad1.b);
            setOriginal = new XCYBoolean(() -> gamepad1.back && gamepad1.x);
      }

//      XCYBoolean reset_imu = new XCYBoolean(() -> gamepad1.share && gamepad1.square);

//      to_last_eject = new XCYBoolean(() -> gamepad1.left_bumper && holding);

//      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

//        upper.setAimReleaseCondition(dpad_down, dpad_up, upper_release, button_y);
//        upper.setWebcamConfig();

      Gamepad.LedEffect effect_holding = new Gamepad.LedEffect.Builder()
              .addStep(1, 0, 0, 65)
              .addStep(1, 1, 0, 65)
              .addStep(0, 1, 0, 65)
              .addStep(0, 1, 1, 65)
              .addStep(0, 0, 1, 65)
              .addStep(1, 0, 1, 65)
              .setRepeating(true)
              .build();
      Gamepad.LedEffect effect_idle = new Gamepad.LedEffect.Builder()
              .addStep(1, 0, 0, 100)
              .addStep(0.5, 0.5, 0, 100)
              .addStep(0, 1, 0, 100)
              .addStep(0, 0.5, 0.5, 100)
              .addStep(0, 0, 1, 100)
              .addStep(0.5, 0, 0.5, 100)
              .setRepeating(true)
              .build();
      gamepad1.runLedEffect(effect_idle);

      waitForStart();
      logic_period();
      int intakeAimPosIndex = 0;
//      lastEjectPos = current_pos;

      while (opModeIsActive()) {

         logic_period();
         drive_period();

//         if (reset_imu.toFalse()) {
//            drive.getLocalizer().setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
//            drive.setPoseEstimate(new Pose2d(0, 0, Math.toRadians(0)));
//         }

         if (cone_saveR.toTrue()) {
            upper.coneSaveR();
            upper.armChange(-0.1);
            global_drive_power = 0.7;
            global_drive_turn_ratio = 0.2;
         } else if (cone_saveR.toFalse()) {
            upper.armChange(0.1);
         }

         if (cone_saveL.toTrue()) {
            upper.coneSaveL();
            upper.armChange(-0.1);
            global_drive_power = 0.7;
            global_drive_turn_ratio = 0.2;
         } else if (cone_saveL.toFalse()) {
            upper.armChange(0.1);
         }

         if (arm_down.toTrue()) {
            global_drive_power = 0.7;
            global_drive_turn_ratio = 0.3;
            upper.toAim();
            intakeAimPosIndex = 0;
            upper.sleep_with_drive(300);
            holding = false;
            sequence = Sequence.EMPTY_DOWN;
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
               gamepad1.runLedEffect(effect_holding);
               global_drive_power = 1;
               global_drive_turn_ratio = 1;
            }
         }

         if (!holding) {
            if (mid_j.toTrue()) {
               intakeAimPosIndex = Range.clip(intakeAimPosIndex + 1, 0, 4);
               upper.toAim(intakeAimPosIndex);
            } else if (low_j.toTrue()) {
               intakeAimPosIndex = Range.clip(intakeAimPosIndex - 1, 0, 4);
               upper.toAim(intakeAimPosIndex);
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
               upper.sleep_with_drive(300);
               sequence = Sequence.HOLDING_UP;
               logic_period();
               while (upper_release.get() || high_j.get() || mid_j.get() || low_j.get()) {
                  logic_period();
                  drive_period();
               }
               while (!(upper_release.toTrue() || high_j.get() || mid_j.get() || low_j.get() || intake_action.get())) {
                  logic_period();
                  drive_period();
               }
               long startTime;
               if (upper_release.get())
                  do {
                     while (!(upper_release.toTrue() || high_j.get() || mid_j.get() || low_j.get() || intake_action.get())) {
                        logic_period();
                        drive_period();
                     }
                     startTime = System.currentTimeMillis();
                     if (upper_release.toTrue()) {
                        upper.armChange(0.1);
                        while (upper_release.get()) {
                           logic_period();
                           drive_period();
                        }
                     }
                     if (System.currentTimeMillis() - startTime > 450)
                        upper.armChange(-0.1);
                  } while (System.currentTimeMillis() - startTime > 450);
            }
         }

         if (upper_release.toFalse()) {
            //x
            if (holding) {
//               lastEjectPos = current_pos;
               upper.openHand();
               upper.sleep_with_drive(350);
               upper.post_eject();
               upper.sleep_with_drive(100);
            } else {
               upper.openHand();
            }
            global_drive_turn_ratio = 1;
            global_drive_power = 1;
            holding = false;
            sequence = Sequence.EMPTY_MIDDLE;
            gamepad1.runLedEffect(effect_idle);
            upper.toOrigional();
            upper.runtimeResetLifter();
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
      drive.update();
   }
}
