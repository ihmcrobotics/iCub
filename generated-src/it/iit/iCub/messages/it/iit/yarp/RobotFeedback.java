package it.iit.iCub.messages.it.iit.yarp;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class RobotFeedback extends Packet<RobotFeedback> implements Settable<RobotFeedback>, EpsilonComparable<RobotFeedback>
{
   public long timestampInNanoseconds_;
   // Optional fields like battery, voltage level etc etc etc
   public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointState> jointStates_;
   // List of states. Joint order agreed beforehand and hard coded in first version
   public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.IMUState> imuStates_;
   // List of states. IMU order agreed beforehand and hard coded in first version
   public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.ForceSensor> forceSensors_;

   public RobotFeedback()
   {

      jointStates_ = new us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointState>(100, it.iit.iCub.messages.it.iit.yarp.JointState.class,
                                                                                                     new it.iit.iCub.messages.it.iit.yarp.JointStatePubSubType());

      imuStates_ = new us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.IMUState>(100, it.iit.iCub.messages.it.iit.yarp.IMUState.class,
                                                                                                 new it.iit.iCub.messages.it.iit.yarp.IMUStatePubSubType());

      forceSensors_ = new us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.ForceSensor>(100, it.iit.iCub.messages.it.iit.yarp.ForceSensor.class,
                                                                                                       new it.iit.iCub.messages.it.iit.yarp.ForceSensorPubSubType());
   }

   public RobotFeedback(RobotFeedback other)
   {
      set(other);
   }

   public void set(RobotFeedback other)
   {
      timestampInNanoseconds_ = other.timestampInNanoseconds_;

      jointStates_.set(other.jointStates_);
      imuStates_.set(other.imuStates_);
      forceSensors_.set(other.forceSensors_);
   }

   public long getTimestampInNanoseconds()
   {
      return timestampInNanoseconds_;
   }

   public void setTimestampInNanoseconds(long timestampInNanoseconds)
   {
      timestampInNanoseconds_ = timestampInNanoseconds;
   }

   // Optional fields like battery, voltage level etc etc etc
   public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.JointState> getJointStates()
   {
      return jointStates_;
   }

   // List of states. Joint order agreed beforehand and hard coded in first version
   public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.IMUState> getImuStates()
   {
      return imuStates_;
   }

   // List of states. IMU order agreed beforehand and hard coded in first version
   public us.ihmc.idl.IDLSequence.Object<it.iit.iCub.messages.it.iit.yarp.ForceSensor> getForceSensors()
   {
      return forceSensors_;
   }

   @Override
   public boolean epsilonEquals(RobotFeedback other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.timestampInNanoseconds_, other.timestampInNanoseconds_, epsilon))
         return false;

      if (this.jointStates_.size() == other.jointStates_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.jointStates_.size(); i++)
         {
            if (!this.jointStates_.get(i).epsilonEquals(other.jointStates_.get(i), epsilon))
               return false;
         }
      }

      if (this.imuStates_.size() == other.imuStates_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.imuStates_.size(); i++)
         {
            if (!this.imuStates_.get(i).epsilonEquals(other.imuStates_.get(i), epsilon))
               return false;
         }
      }

      if (this.forceSensors_.size() == other.forceSensors_.size())
      {
         return false;
      }
      else
      {
         for (int i = 0; i < this.forceSensors_.size(); i++)
         {
            if (!this.forceSensors_.get(i).epsilonEquals(other.forceSensors_.get(i), epsilon))
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
      if (!(other instanceof RobotFeedback))
         return false;

      RobotFeedback otherMyClass = (RobotFeedback) other;

      if (this.timestampInNanoseconds_ != otherMyClass.timestampInNanoseconds_)
         return false;

      if (!this.jointStates_.equals(otherMyClass.jointStates_))
         return false;

      if (!this.imuStates_.equals(otherMyClass.imuStates_))
         return false;

      if (!this.forceSensors_.equals(otherMyClass.forceSensors_))
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("RobotFeedback {");
      builder.append("timestampInNanoseconds=");
      builder.append(this.timestampInNanoseconds_);

      builder.append(", ");
      builder.append("jointStates=");
      builder.append(this.jointStates_);

      builder.append(", ");
      builder.append("imuStates=");
      builder.append(this.imuStates_);

      builder.append(", ");
      builder.append("forceSensors=");
      builder.append(this.forceSensors_);

      builder.append("}");
      return builder.toString();
   }
}