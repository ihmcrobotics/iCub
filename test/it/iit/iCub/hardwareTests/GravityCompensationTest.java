package it.iit.iCub.hardwareTests;

import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import it.iit.iCub.IcubRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.InverseDynamicsCalculator;
import us.ihmc.sensorProcessing.simulatedSensors.SDFPerfectSimulatedSensorReader;
import us.ihmc.simulationConstructionSetTools.robotController.SimpleRobotController;
import us.ihmc.simulationToolkit.outputWriters.PerfectSimulatedOutputWriter;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

public class GravityCompensationTest
{
   @Test(timeout = 300000)
   public void testGravityCompensation()
   {
      IcubRobotModel robotModel = new IcubRobotModel(true);

      FullHumanoidRobotModel fullRobotModel = robotModel.createFullRobotModel();
      double totalMass = fullRobotModel.getTotalMass();
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup = robotModel.getDefaultRobotInitialSetup(0.2, 0.0);
      HumanoidFloatingRootJointRobot robot = robotModel.createHumanoidFloatingRootJointRobot(false);
      initialSetup.initializeRobot(robot, robotModel.getJointMap());
      fixPelvis(totalMass, robot);

      robot.setController(new GravityCompensator(fullRobotModel, robot));

      SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      scs.startOnAThread();
      ThreadTools.sleep(100000);
   }

   private class GravityCompensator extends SimpleRobotController
   {
      private final InverseDynamicsCalculator inverseDynamicsCalculator;
      private final PerfectSimulatedOutputWriter outputWriter;
      private final SDFPerfectSimulatedSensorReader sensorReader;

      public GravityCompensator(FullHumanoidRobotModel fullRobotModel, HumanoidFloatingRootJointRobot robot)
      {
         outputWriter = new PerfectSimulatedOutputWriter(robot, fullRobotModel);
         sensorReader = new SDFPerfectSimulatedSensorReader(robot, fullRobotModel, null);
         inverseDynamicsCalculator = new InverseDynamicsCalculator(fullRobotModel.getPelvis(), -robot.getGravityZ());
      }

      @Override
      public void doControl()
      {
         sensorReader.read();
         inverseDynamicsCalculator.compute();
         outputWriter.write();
      }
   }

   private static void fixPelvis(double totalMass, HumanoidFloatingRootJointRobot robot)
   {
      double massCompensation = -robot.getGravityZ() * totalMass;
      List<Vector3D> forcePointOffsets = new ArrayList<>();
      forcePointOffsets.add(new Vector3D(Math.sin(0.0 * Math.PI / 3.0), Math.cos(0.0 * Math.PI / 3.0), 0.0));
      forcePointOffsets.add(new Vector3D(Math.sin(2.0 * Math.PI / 3.0), Math.cos(2.0 * Math.PI / 3.0), 0.0));
      forcePointOffsets.add(new Vector3D(Math.sin(4.0 * Math.PI / 3.0), Math.cos(4.0 * Math.PI / 3.0), 0.0));

      int count = 0;
      for (Vector3D forcePointOffset : forcePointOffsets)
      {
         ExternalForcePoint forcePoint = new ExternalForcePoint("FixPelvis" + count, forcePointOffset, robot);
         robot.getRootJoint().addExternalForcePoint(forcePoint);
         Point3D desiredForcePointPosition = new Point3D();
         robot.getRootJoint().getPosition(desiredForcePointPosition);
         desiredForcePointPosition.add(forcePointOffset);
         double massCompensationPerPoint = massCompensation / forcePointOffsets.size();
         robot.setController(new ForcePointController(count, massCompensationPerPoint, forcePoint, desiredForcePointPosition));
         count++;
      }
   }

   private static class ForcePointController extends SimpleRobotController
   {
      private final int count;
      private final ExternalForcePoint forcePoint;
      private final double massCompensation;

      private final Point3D currentForcePointPosition = new Point3D();
      private final Point3D desiredForcePointPosition = new Point3D();
      private final Vector3D velocity = new Vector3D();
      private final Vector3D force = new Vector3D();

      public ForcePointController(int count, double massCompensation, ExternalForcePoint forcePoint, Point3D desiredPosition)
      {
         this.count = count;
         this.forcePoint = forcePoint;
         this.massCompensation = massCompensation;
         desiredForcePointPosition.set(desiredPosition);
      }

      @Override
      public void doControl()
      {
         forcePoint.getPosition(currentForcePointPosition);
         forcePoint.getVelocity(velocity);

         force.sub(desiredForcePointPosition, currentForcePointPosition);
         force.scale(10000.0);
         velocity.scale(-100.0);
         force.add(velocity);
         force.addZ(massCompensation);

         forcePoint.setForce(force);
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return new YoVariableRegistry("ForcePoint" + count);
      }

   }

}
