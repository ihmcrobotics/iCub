package it.iit.iCub.hardwareDrivers.yarp;

import gnu.trove.map.TIntObjectMap;
import it.iit.iCub.IcubRobotModel;
import it.iit.iCub.hardwareDrivers.IcubIndexToJointMapTools;
import it.iit.iCub.hardwareDrivers.sensorReader.IcubSensorReader;
import it.iit.iCub.hardwareDrivers.sensorReader.IcubSensorReaderFactory;
import it.iit.iCub.messages.it.iit.yarp.ORSControlMode;
import it.iit.iCub.messages.it.iit.yarp.RobotDesireds;
import it.iit.iCub.messages.it.iit.yarp.RobotFeedback;
import it.iit.iCub.parameters.IcubOrderedJointMap;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ContactSensorHolder;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.sensors.RawJointSensorDataHolderMap;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class IcubUDPRobotFeedbackExample implements Runnable
{
   private final RobotFeedback iCubRobotFeedback = new RobotFeedback();
   //   private final PeriodicNonRealtimeThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler(getClass().getSimpleName());

   private final IcubRobotModel robotModel = new IcubRobotModel(false);

   private final IcubSensorReader sensorReader;
   private final IcubUDPRobotDesiredsSender sender;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final SimulationConstructionSet simulationConstructionSet;
   private final HumanoidFloatingRootJointRobot humanoidFloatingRootJointRobot;
   private final FullHumanoidRobotModel fullRobotModel;

   private final RobotDesireds desireds = new RobotDesireds();
   private final HashMap<OneDoFJoint, YoDouble> desiredsMap = new HashMap<>();
   private final TIntObjectMap<OneDoFJoint> mapping;

   boolean firstTick = true;

   public IcubUDPRobotFeedbackExample()
   {
      IcubSensorReaderFactory sensorReaderFactory = new IcubSensorReaderFactory(robotModel);

      humanoidFloatingRootJointRobot = robotModel.createHumanoidFloatingRootJointRobot(false);
      humanoidFloatingRootJointRobot.setDynamic(false);
      fullRobotModel = robotModel.createFullRobotModel();
      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      ContactSensorHolder contactSensorHolder = new ContactSensorHolder(Arrays.asList(fullRobotModel.getContactSensorDefinitions()));
      RawJointSensorDataHolderMap rawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(fullRobotModel);
      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap, null, registry);

      sensorReader = (IcubSensorReader) sensorReaderFactory.getSensorReader();

      simulationConstructionSet = new SimulationConstructionSet(humanoidFloatingRootJointRobot);
      simulationConstructionSet.setGroundVisible(false);
      simulationConstructionSet.addYoVariableRegistry(registry);

      ArrayList<OneDoFJoint> oneDoFJointsToPack = new ArrayList<>();
      fullRobotModel.getOneDoFJoints(oneDoFJointsToPack);

      mapping = IcubIndexToJointMapTools.createMapping(oneDoFJointsToPack);

      for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
      {
         YoDouble jointDesired = new YoDouble(oneDoFJoint.getName() + "desired", registry);

         desiredsMap.put(oneDoFJoint, jointDesired);
      }

      for (int i = 0; i <= IcubOrderedJointMap.r_ankle_roll; i++)
      {
         desireds.getJointDesireds().add();
      }

      sender = new IcubUDPRobotDesiredsSender(IcubUDPRobotDesiredsSender.DEFAULT_YARP_ROBOT_DESIRED_IP,
                                              IcubUDPRobotDesiredsSender.DEFAULT_YARP_ROBOT_DESIRED_PORT, desireds);
   }

   private void start()
   {
      try
      {
         sensorReader.connect();
         sender.connect();
         simulationConstructionSet.startOnAThread();
      }
      catch (IOException e)
      {
         PrintTools.error("Couldn't connect sensor reader!");
         e.printStackTrace();
         System.exit(-1);
      }

      //      scheduler.schedule(this, Conversions.secondsToNanoseconds(0.005), TimeUnit.NANOSECONDS);
      Thread thread = new Thread(this, getClass().getSimpleName());
      thread.start();
   }

   @Override
   public void run()
   {
      while (true)
      {
         sensorReader.read();

         for (OneDoFJoint oneDoFJoint : fullRobotModel.getOneDoFJoints())
         {
            OneDegreeOfFreedomJoint joint = (OneDegreeOfFreedomJoint) humanoidFloatingRootJointRobot.getJoint(oneDoFJoint.getName());

            joint.setQ(oneDoFJoint.getQ());
            joint.setQd(oneDoFJoint.getQd());

            if(firstTick)
            {
               desiredsMap.get(oneDoFJoint).set(oneDoFJoint.getQ());
            }
         }

         firstTick = false;

         humanoidFloatingRootJointRobot.update();

         for (int i = 0; i <= IcubOrderedJointMap.r_ankle_roll; i++)
         {
            OneDoFJoint oneDoFJoint = mapping.get(i);
            YoDouble yoDouble = desiredsMap.get(oneDoFJoint);

            desireds.getJointDesireds().get(i).setControlMode(ORSControlMode.POSITION_CONTROL);
            desireds.getJointDesireds().get(i).setQDesired(yoDouble.getDoubleValue());
         }

         sender.send();
      }
   }

   public static void main(String[] args)
   {
      IcubUDPRobotFeedbackExample icubUDPRobotFeedbackExample = new IcubUDPRobotFeedbackExample();

      icubUDPRobotFeedbackExample.start();
   }
}
