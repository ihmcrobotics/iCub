package it.iit.iCub.simulation;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;

import it.iit.iCub.IcubRobotModel;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class IcubSdfLoadingDemo
{
   private static final boolean SHOW_ELLIPSOIDS = false;
   private static final boolean SHOW_COORDINATES_AT_JOINT_ORIGIN = true;

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
