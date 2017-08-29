package it.iit.iCub.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import it.iit.iCub.IcubRobotModel;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class IcubSdfLoadingDemo
{
   private static final boolean SHOW_ELLIPSOIDS = false;
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = true;
   private static final boolean CHECK_SYMMETRY = false;

   public IcubSdfLoadingDemo()
   {
      IcubRobotModel icubRobotModel = new IcubRobotModel();
      FloatingRootJointRobot sdfRobot = icubRobotModel.createHumanoidFloatingRootJointRobot(false);

      sdfRobot.setPositionInWorld(new Vector3D(0.0, 0.0, 1.0));
      PrintTools.info("Robot Mass: " + sdfRobot.computeCenterOfMass(new Point3D()));

      if (SHOW_ELLIPSOIDS)
      {
         addIntertialEllipsoidsToVisualizer(sdfRobot);
      }

      if (SHOW_COORDINATES_AT_JOINT_ORIGIN)
      {
         addJointAxis(sdfRobot);
      }

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);

      Graphics3DObject worldFrameGraphics = new Graphics3DObject();
      worldFrameGraphics.addCoordinateSystem(0.4);
      scs.addStaticLinkGraphics(worldFrameGraphics);

      scs.startOnAThread();

      if (CHECK_SYMMETRY)
      {
         FullHumanoidRobotModel fullRobotModel = icubRobotModel.createFullRobotModel();

         RigidBody leftFoot = fullRobotModel.getFoot(RobotSide.LEFT);
         RigidBody rightFoot = fullRobotModel.getFoot(RobotSide.RIGHT);
         RigidBody pelvis = fullRobotModel.getPelvis();
         compareChain(pelvis, leftFoot, rightFoot);

         RigidBody leftHand = fullRobotModel.getHand(RobotSide.LEFT);
         RigidBody rightHand = fullRobotModel.getHand(RobotSide.RIGHT);
         RigidBody chest = fullRobotModel.getChest();
         compareChain(chest, leftHand, rightHand);
      }
   }

   private static void compareChain(RigidBody commonBody, RigidBody endEffectorA, RigidBody endEffectorB)
   {
      RigidBody bodyA = endEffectorA;
      RigidBody bodyB = endEffectorB;

      while (!commonBody.getName().equals(bodyA.getName()))
      {
         compareBodies(bodyA, bodyB);
         bodyA = bodyA.getParentJoint().getPredecessor();
         bodyB = bodyB.getParentJoint().getPredecessor();
      }
   }

   private static void compareBodies(RigidBody bodyA, RigidBody bodyB)
   {
      PrintTools.info("Comparing body " + bodyA.getName() + " and body " + bodyB.getName() + ".");

      if (bodyA.getInertia().getMass() != bodyB.getInertia().getMass())
      {
         System.out.println("different mass:");
         System.out.println(bodyA.getName() + ": " + bodyA.getInertia().getMass());
         System.out.println(bodyB.getName() + ": " + bodyB.getInertia().getMass());
      }

      if (!bodyA.getInertia().getCenterOfMassOffset().epsilonEquals(bodyA.getInertia().getCenterOfMassOffset(), 1.0E-10))
      {
         System.out.println("different center of mass:");
      }

      if (!bodyA.getInertia().getMassMomentOfInertiaPartCopy().epsilonEquals(bodyA.getInertia().getMassMomentOfInertiaPartCopy(), 1.0E-10))
      {
         System.out.println("different moment of inertia:");
      }

      RigidBodyTransform bodyATransform = new RigidBodyTransform(bodyA.getParentJoint().getOffsetTransform3D());
      RigidBodyTransform bodyBTransform = new RigidBodyTransform(bodyB.getParentJoint().getOffsetTransform3D());

      Vector3D translationVectorA = new Vector3D(bodyATransform.getTranslationVector());
      Vector3D translationVectorB = new Vector3D(bodyBTransform.getTranslationVector());
      translationVectorB.setY(-translationVectorB.getY());

      if (!translationVectorA.epsilonEquals(translationVectorB, 1.0E-3))
      {
         System.out.println("different translation offsets of the parent joints:");
         System.out.println(bodyA.getParentJoint().getName() + ": " + translationVectorA);
         System.out.println(bodyB.getParentJoint().getName() + ": " + translationVectorB + " (y flipped)");
      }
   }

   private void addIntertialEllipsoidsToVisualizer(FloatingRootJointRobot sdfRobot)
   {
      ArrayList<Joint> joints = new ArrayList<>();
      joints.add(sdfRobot.getRootJoint());

      HashSet<Link> links = getAllLinks(joints, new HashSet<Link>());

      for (Link link : links)
      {
         AppearanceDefinition appearance = YoAppearance.Green();
         appearance.setTransparency(0.6);

         if (link.getLinkGraphics() == null)
         {
            Graphics3DObject linkGraphics = new Graphics3DObject();
            link.setLinkGraphics(linkGraphics);
         }

         link.addEllipsoidFromMassProperties(appearance);
         link.addCoordinateSystemToCOM(0.1);
      }
   }

   private HashSet<Link> getAllLinks(ArrayList<Joint> joints, HashSet<Link> links)
   {
      for (Joint joint : joints)
      {
         links.add(joint.getLink());

         if (!joint.getChildrenJoints().isEmpty())
         {
            links.addAll(getAllLinks(joint.getChildrenJoints(), links));
         }
      }

      return links;
   }

   public void addJointAxis(FloatingRootJointRobot sdfRobot)
   {
      ArrayList<OneDegreeOfFreedomJoint> joints = new ArrayList<>(Arrays.asList(sdfRobot.getOneDegreeOfFreedomJoints()));

      for (OneDegreeOfFreedomJoint joint : joints)
      {
         Graphics3DObject linkGraphics = new Graphics3DObject();
         linkGraphics.addCoordinateSystem(0.1);
         Link link = joint.getLink();
         if (link.getLinkGraphics() != null)
         {
            linkGraphics.combine(link.getLinkGraphics());
         }
         else
         {
            link.setLinkGraphics(linkGraphics);
         }
         link.setLinkGraphics(linkGraphics);
      }
   }

   public static void main(String[] args)
   {
      new IcubSdfLoadingDemo();
   }
}
