package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Configurations.*;

@Autonomous
public class BlueLeft extends AutoMaster{

   @Override
   public void runOpMode() throws InterruptedException{
      startSide = LEFT;
      side_color = BLUE;
      firstJunctionPos = Junction.SIDE_HIGH;
      initHardware();
      try {
         longMoveNormal(100);
         eject(0);
         for (cone_index = 4; cone_index >= 0; cone_index--) {
            try {
               intake(cone_index,firstJunctionPos);
            } catch (Configurations.TimeoutException e) {
               intakeSave();
            }
            moveToEject(Junction.SIDE_HIGH);
            eject(0);
         }
         park();
      } catch (GlobalTimeoutException e) {
         park();
      } catch (Exception e) {
         throw new InterruptedException();
      } finally {
         savePosition();
      }
   }
}
