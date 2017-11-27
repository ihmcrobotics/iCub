package it.iit.iCub.hardwareDrivers.yarp;

import it.iit.iCub.IcubRobotModel;
import it.iit.iCub.messages.it.iit.yarp.JointState;
import it.iit.iCub.messages.it.iit.yarp.RobotFeedback;
import it.iit.iCub.parameters.IcubOrderedJointMap;
import us.ihmc.commons.Conversions;
import us.ihmc.idl.IDLSequence;
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

   private final IcubRobotModel robotModel = new IcubRobotModel(false);

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
      long receive = feedbackReceiver.receive();

      if(receive != -1)
      {
         IDLSequence.Object<JointState> jointStates = iCubRobotFeedback.getJointStates();

         System.out.println("received " + jointStates.size() + " joint states");

         for (int i = 0; i < jointStates.size(); i++)
         {
            String jointName = IcubOrderedJointMap.jointNames[i];

//            System.out.println(jointName + "_q: " + jointStates.get(i).getQ());
         }
      }
      else
      {
         System.out.println("Failed to receive robot feedback.");
      }

//      System.out.println(iCubRobotFeedback);
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
