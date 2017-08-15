package it.iit.iCub.parameters;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.configurations.ContinuousCMPICPPlannerParameters;
import us.ihmc.euclid.tuple2D.Vector2D;

/** {@inheritDoc} */
public class IcubCapturePointPlannerParameters extends ContinuousCMPICPPlannerParameters
{
   private List<Vector2D> copOffsets;
   private List<Vector2D> copForwardOffsetBounds;

   /** {@inheritDoc} */
   @Override
   public int getNumberOfCoPWayPointsPerFoot()
   {
      return 2;
   }

   /** {@inheritDoc} */
   @Override
   public List<Vector2D> getCoPOffsets()
   {
      if (copOffsets != null)
         return copOffsets;

      Vector2D entryOffset = new Vector2D(0.0, 0.006);
      Vector2D exitOffset = new Vector2D(0.0, 0.006);

      copOffsets = new ArrayList<>();
      copOffsets.add(entryOffset);
      copOffsets.add(exitOffset);

      return copOffsets;
   }

   /** {@inheritDoc} */
   @Override
   public List<Vector2D> getCoPForwardOffsetBounds()
   {
      if (copForwardOffsetBounds != null)
         return copForwardOffsetBounds;

      Vector2D entryBounds = new Vector2D(-0.02, 0.05);
      Vector2D exitBounds = new Vector2D(-0.02, 0.05);

      copForwardOffsetBounds = new ArrayList<>();
      copForwardOffsetBounds.add(entryBounds);
      copForwardOffsetBounds.add(exitBounds);

      return copForwardOffsetBounds;
   }
}
