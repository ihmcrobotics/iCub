package it.iit.iCub.parameters;

import static it.iit.iCub.parameters.IcubPhysicalProperties.footLength;
import static it.iit.iCub.parameters.IcubPhysicalProperties.footWidth;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotContactPointParameters;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotJointMap;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

public class IcubContactPointParameters extends DRCRobotContactPointParameters
{
   private final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();

   private final List<Pair<String, Vector3d>> jointNameGroundContactPointMap = new ArrayList<Pair<String, Vector3d>>();
   private final SideDependentList<ArrayList<Point2d>> footGroundContactPoints = new SideDependentList<>();

   public IcubContactPointParameters(DRCRobotJointMap jointMap)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         footGroundContactPoints.put(robotSide, new ArrayList<Point2d>());
         RigidBodyTransform ankleToSoleFrame = IcubPhysicalProperties.getAnkleToSoleFrameTransform(robotSide);

         ArrayList<Pair<String, Point2d>> footGCs = new ArrayList<>();
         String jointBeforeFootName = jointMap.getJointBeforeFootName(robotSide);
         footGCs.add(new Pair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0, -footWidth / 2.0)));
         footGCs.add(new Pair<String, Point2d>(jointBeforeFootName, new Point2d(footLength / 2.0, footWidth / 2.0)));
         footGCs.add(new Pair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0, -footWidth / 2.0)));
         footGCs.add(new Pair<String, Point2d>(jointBeforeFootName, new Point2d(-footLength / 2.0, footWidth / 2.0)));

         for (Pair<String, Point2d> footGC : footGCs)
         {
            footGroundContactPoints.get(robotSide).add(footGC.second());

            Point3d gcOffset = new Point3d(footGC.second().getX(), footGC.second().getY(), 0.0);
            ankleToSoleFrame.transform(gcOffset);
            jointNameGroundContactPointMap.add(new Pair<String, Vector3d>(footGC.first(), new Vector3d(gcOffset)));
         }
      }

      setupContactableBodiesFactory(jointMap);
   }

   private void setupContactableBodiesFactory(DRCRobotJointMap jointMap)
   {
      contactableBodiesFactory.addFootContactParameters(getFootContactPoints());
   }

   @Override
   public RigidBodyTransform getPelvisContactPointTransform()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public List<Point2d> getPelvisContactPoints()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public RigidBodyTransform getPelvisBackContactPointTransform()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public List<Point2d> getPelvisBackContactPoints()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public RigidBodyTransform getChestBackContactPointTransform()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public List<Point2d> getChestBackContactPoints()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public SideDependentList<RigidBodyTransform> getThighContactPointTransforms()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public SideDependentList<List<Point2d>> getThighContactPoints()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public List<Pair<String, Vector3d>> getJointNameGroundContactPointMap()
   {
      return jointNameGroundContactPointMap;
   }

   @Override
   public SideDependentList<ArrayList<Point2d>> getFootContactPoints()
   {
      return footGroundContactPoints;
   }

   @Override
   public ContactableBodiesFactory getContactableBodiesFactory()
   {
      return contactableBodiesFactory;
   }

   @Override
   public SideDependentList<RigidBodyTransform> getHandContactPointTransforms()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public SideDependentList<List<Point2d>> getHandContactPoints()
   {
      // TODO Auto-generated method stub
      return null;
   }

   @Override
   public void setupGroundContactModelParameters(LinearGroundContactModel linearGroundContactModel)
   {
      linearGroundContactModel.setZStiffness(2000.0);
      linearGroundContactModel.setZDamping(1500.0);
      linearGroundContactModel.setXYStiffness(50000.0);
      linearGroundContactModel.setXYDamping(2000.0);
   }
}
