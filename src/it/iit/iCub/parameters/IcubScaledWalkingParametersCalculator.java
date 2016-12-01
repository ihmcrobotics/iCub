package it.iit.iCub.parameters;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import boofcv.alg.geo.calibration.CalibrationObservation.Point;
import it.iit.iCub.IcubInitialSetup;
import it.iit.iCub.IcubRobotModel;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;

public class IcubScaledWalkingParametersCalculator
{
   public static double calculateDesiredHeightCalculator(double scale)
   {
      IcubRobotModel orig = new IcubRobotModel(true, true, 1.0);
      IcubRobotModel scaled = new IcubRobotModel(true, true, scale);

      IcubInitialSetup origInitialSetup = new IcubInitialSetup(0, 0);
      IcubInitialSetup scaledInitialSetup = new IcubInitialSetup(0, 0);
      HumanoidFloatingRootJointRobot origModel = orig.createHumanoidFloatingRootJointRobot(false);
      HumanoidFloatingRootJointRobot scaledModel = scaled.createHumanoidFloatingRootJointRobot(false);
      
//      FullHumanoidRobotModel origModel = orig.createFullRobotModel();
//      FullHumanoidRobotModel scaledModel = scaled.createFullRobotModel();
//
//      CenterOfMassCalculator origCalculator = new CenterOfMassCalculator(origModel.getElevator(), origModel.getSoleFrame(RobotSide.LEFT));
//      CenterOfMassCalculator scaledCalculator = new CenterOfMassCalculator(scaledModel.getElevator(), scaledModel.getSoleFrame(RobotSide.LEFT));
//      
//      origModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).setQ(IcubInitialSetup.initialKneeAngle);
//      scaledModel.getLegJoint(RobotSide.LEFT, LegJointName.KNEE_PITCH).setQ(IcubInitialSetup.initialKneeAngle);
//      origModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH).setQ(IcubInitialSetup.initialKneeAngle);
//      scaledModel.getLegJoint(RobotSide.RIGHT, LegJointName.KNEE_PITCH).setQ(IcubInitialSetup.initialKneeAngle);
//
//      origCalculator.compute();
//      scaledCalculator.compute();

      origInitialSetup.initializeRobot(origModel, orig.getJointMap());
      scaledInitialSetup.initializeRobot(scaledModel, scaled.getJointMap());
      
      Point3d origCoM = new Point3d();
      Point3d scaledCoM = new Point3d();
      
      System.out.println(origModel.computeCenterOfMass(origCoM));
      System.out.println(scaledModel.computeCenterOfMass(scaledCoM));
      
      
      return 0.0;

   }

   public static void main(String[] args)
   {
      IcubScaledWalkingParametersCalculator.calculateDesiredHeightCalculator(0.75);
   }
}
