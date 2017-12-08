package it.iit.iCub.hardwareDrivers.yarp;

import java.io.IOException;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;

import gnu.trove.map.TIntObjectMap;
import it.iit.iCub.IcubRobotModel;
import it.iit.iCub.hardwareDrivers.IcubIndexToJointMapTools;
import it.iit.iCub.hardwareDrivers.sensorReader.IcubSensorReader;
import it.iit.iCub.hardwareDrivers.sensorReader.IcubSensorReaderFactory;
import it.iit.iCub.messages.it.iit.yarp.JointDesired;
import it.iit.iCub.messages.it.iit.yarp.ORSControlMode;
import it.iit.iCub.messages.it.iit.yarp.RobotDesireds;
import it.iit.iCub.parameters.IcubOrderedJointMap;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
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

public class GravityCompensationDemo implements Runnable
{
   private static final IcubRobotModel robotModel = new IcubRobotModel(true);

   private final IcubSensorReader sensorReader;
   private final IcubUDPRobotDesiredsSender sender;
   private final SimulationConstructionSet simulationConstructionSet;
   private final HumanoidFloatingRootJointRobot robot;
   private final FullHumanoidRobotModel fullRobotModel;
   private final RobotDesireds desireds = new RobotDesireds();
   private final TIntObjectMap<OneDoFJoint> mapping;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final InverseDynamicsCalculator inverseDynamicsCalculator;

   private final YoDouble kp = new YoDouble("kp", registry);
   private final HashMap<OneDoFJoint, YoDouble> initialsMap = new HashMap<>();
   private final HashMap<OneDoFJoint, YoDouble> gravityTorques = new HashMap<>();
   private final HashMap<OneDoFJoint, YoDouble> feedbackTorques = new HashMap<>();
   boolean firstTick = true;

   public GravityCompensationDemo()
   {
      robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      robot.setDynamic(false);
      fullRobotModel = robotModel.createFullRobotModel();

      inverseDynamicsCalculator = new InverseDynamicsCalculator(fullRobotModel.getPelvis(), -robot.getGravityZ());

      IcubSensorReaderFactory sensorReaderFactory = new IcubSensorReaderFactory(robotModel);
      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      IMUDefinition[] imuDefinitions = fullRobotModel.getIMUDefinitions();
      ForceSensorDefinition[] forceSensorDefinitions = fullRobotModel.getForceSensorDefinitions();
      ContactSensorHolder contactSensorHolder = new ContactSensorHolder(Arrays.asList(fullRobotModel.getContactSensorDefinitions()));
      RawJointSensorDataHolderMap rawJointSensorDataHolderMap = new RawJointSensorDataHolderMap(fullRobotModel);
      sensorReaderFactory.build(rootJoint, imuDefinitions, forceSensorDefinitions, contactSensorHolder, rawJointSensorDataHolderMap, null, registry);
      sensorReader = (IcubSensorReader) sensorReaderFactory.getSensorReader();

      simulationConstructionSet = new SimulationConstructionSet(robot);
      simulationConstructionSet.setGroundVisible(false);
      simulationConstructionSet.addYoVariableRegistry(registry);

      List<OneDoFJoint> oneDofJoints = Arrays.asList(fullRobotModel.getOneDoFJoints());
      mapping = IcubIndexToJointMapTools.createMapping(oneDofJoints);

      for (OneDoFJoint oneDoFJoint : oneDofJoints)
      {
         YoDouble jointDesired = new YoDouble(oneDoFJoint.getName() + "_initial", registry);
         initialsMap.put(oneDoFJoint, jointDesired);
         YoDouble gravityTorque = new YoDouble(oneDoFJoint.getName() + "_gravityTorque", registry);
         gravityTorques.put(oneDoFJoint, gravityTorque);
         YoDouble feedbackTorque = new YoDouble(oneDoFJoint.getName() + "_feedbackTorque", registry);
         feedbackTorques.put(oneDoFJoint, feedbackTorque);
      }
      kp.set(1.0);

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

      Thread thread = new Thread(this, getClass().getSimpleName());
      thread.start();
   }

   @Override
   public void run()
   {
      while (true)
      {
         sensorReader.read();
         OneDoFJoint[] oneDoFJoints = fullRobotModel.getOneDoFJoints();

         for (OneDoFJoint oneDoFJoint : oneDoFJoints)
         {
            OneDegreeOfFreedomJoint joint = robot.getOneDegreeOfFreedomJoint(oneDoFJoint.getName());
            joint.setQ(oneDoFJoint.getQ());
            joint.setQd(oneDoFJoint.getQd());
            if (firstTick)
            {
               initialsMap.get(oneDoFJoint).set(oneDoFJoint.getQ());
            }
         }

         firstTick = false;
         simulationConstructionSet.tickAndUpdate();
         robot.update();

         inverseDynamicsCalculator.compute();
         for (int i = 0; i <= IcubOrderedJointMap.r_ankle_roll; i++)
         {
            OneDoFJoint oneDoFJoint = mapping.get(i);
            JointDesired jointDesired = desireds.getJointDesireds().get(i);

            double initialQ = initialsMap.get(oneDoFJoint).getDoubleValue();
            double currentQ = oneDoFJoint.getQ();
            double tauGravity = oneDoFJoint.getTau();

            jointDesired.setControlMode(ORSControlMode.TORQUE_CONTROL);
            jointDesired.setTau(tauGravity);
            jointDesired.setQDesired(initialQ);
            jointDesired.setKp(kp.getDoubleValue());

            gravityTorques.get(oneDoFJoint).set(tauGravity);
            feedbackTorques.get(oneDoFJoint).set(kp.getDoubleValue() * (initialQ - currentQ));
         }

         sender.send();
      }
   }

   public static void main(String[] args)
   {
      GravityCompensationDemo gravityCompensationDemo = new GravityCompensationDemo();
      gravityCompensationDemo.start();
   }
}
