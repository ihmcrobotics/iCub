package it.iit.iCub.hardwareDrivers.yarp;

import it.iit.iCub.messages.it.iit.yarp.RobotFeedback;
import us.ihmc.commons.Conversions;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;

import java.io.IOException;
import java.util.concurrent.TimeUnit;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class IcubUDPRobotFeedbackExample implements Runnable
{
   private final RobotFeedback iCubRobotFeedback = new RobotFeedback();
   private final PeriodicNonRealtimeThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());
   private final IcubUDPRobotFeedbackReceiver feedbackReceiver = new IcubUDPRobotFeedbackReceiver("192.168.96.1",
                                                                                                  IcubUDPRobotFeedbackReceiver.YARP_ROBOT_FEEDBACK_PORT,
                                                                                                  iCubRobotFeedback);

   private void start() throws IOException, InterruptedException
   {
      feedbackReceiver.connect();
      scheduler.schedule(this, Conversions.secondsToNanoseconds(0.005), TimeUnit.NANOSECONDS);

      //      Thread thread = new Thread(this);
      //      thread.start();
      //      thread.join();
   }

   @Override
   public void run()
   {
      //      while(true)
      //      {
      try
      {
         long receive = feedbackReceiver.receive();

         System.out.println(iCubRobotFeedback);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      //      }
   }

   public static void main(String[] args)
   {
      IcubUDPRobotFeedbackExample icubUDPRobotFeedbackExample = new IcubUDPRobotFeedbackExample();

      try
      {
         icubUDPRobotFeedbackExample.start();
      }
      catch (IOException | InterruptedException e)
      {
         e.printStackTrace();
      }
   }
}
