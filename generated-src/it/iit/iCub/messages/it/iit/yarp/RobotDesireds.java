package it.iit.iCub.messages.it.iit.yarp;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class RobotDesireds extends Packet<RobotDesireds> implements Settable<RobotDesireds>, EpsilonComparable<RobotDesireds>
{
   public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointDesired> jointDesireds_;

   public RobotDesireds()
   {
      jointDesireds_ = new us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointDesired>(100,
                                                                                                         it.iit.iCub.messages.it.iit.yarp.JointDesired.class,
                                                                                                         new it.iit.iCub.messages.it.iit.yarp.JointDesiredPubSubType());
   }

   public RobotDesireds(RobotDesireds other)
   {
      set(other);
   }

   public void set(RobotDesireds other)
   {
      jointDesireds_.set(other.jointDesireds_);
   }

   public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointDesired> getJointDesireds()
   {
      return jointDesireds_;
   }

   @Override
   public boolean epsilonEquals(RobotDesireds other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (this.jointDesireds_.size() == other.jointDesireds_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.jointDesireds_.size(); i++)
         {
            if (!this.jointDesireds_.get(i).epsilonEquals(other.jointDesireds_.get(i), epsilon))
               return false;
         }
      }

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof RobotDesireds))
         return false;

      RobotDesireds otherMyClass = (RobotDesireds) other;

      if (!this.jointDesireds_.equals(otherMyClass.jointDesireds_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotDesireds {");
      builder.append("jointDesireds=");
      builder.append(this.jointDesireds_);

      builder.append("}");
      return builder.toString();
   }
}