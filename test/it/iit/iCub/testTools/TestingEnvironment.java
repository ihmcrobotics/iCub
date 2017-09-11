package it.iit.iCub.testTools;

import java.util.List;

import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.SelectableObjectListener;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;

public class TestingEnvironment implements CommonAvatarEnvironmentInterface
{
   protected final CombinedTerrainObject3D terrain;

   public TestingEnvironment()
   {
      terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
   }

   @Override
   public TerrainObject3D getTerrainObject3D()
   {
      return terrain;
   }

   @Override
   public List<? extends Robot> getEnvironmentRobots()
   {
      return null;
   }

   @Override
   public void createAndSetContactControllerToARobot()
   {
   }

   @Override
   public void addContactPoints(List<? extends ExternalForcePoint> externalForcePoints)
   {
   }

   @Override
   public void addSelectableListenerToSelectables(SelectableObjectListener selectedListener)
   {
   }
}
