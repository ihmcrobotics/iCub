package it.iit.iCub.messages.it.iit.yarp;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class JointDesired extends Packet<JointDesired> implements Settable<JointDesired>, EpsilonComparable<JointDesired>
{
   public it.iit.iCub.messages.it.iit.yarp.ORSControlMode controlMode_;
   public double tau_;
   //Desired torque
   public double kp_;
   // Desired joint stiffness, can be zero to do only torque control
   public double kd_;
   // Desired joint damping, can be zero to do only torque control
   public double qDesired_;
   // Desired position
   public double qdDesired_;

   public JointDesired()
   {

   }

   public JointDesired(JointDesired other)
   {
      set(other);
   }

   public void set(JointDesired other)
   {
      controlMode_ = other.controlMode_;

      tau_ = other.tau_;

      kp_ = other.kp_;

      kd_ = other.kd_;

      qDesired_ = other.qDesired_;

      qdDesired_ = other.qdDesired_;
   }

   public it.iit.iCub.messages.it.iit.yarp.ORSControlMode getControlMode()
   {
      return controlMode_;
   }

   public void setControlMode(it.iit.iCub.messages.it.iit.yarp.ORSControlMode controlMode)
   {
      controlMode_ = controlMode;
   }

   public double getTau()
   {
      return tau_;
   }

   public void setTau(double tau)
   {
      tau_ = tau;
   }

   //Desired torque
   public double getKp()
   {
      return kp_;
   }

   //Desired torque
   public void setKp(double kp)
   {
      kp_ = kp;
   }

   // Desired joint stiffness, can be zero to do only torque control
   public double getKd()
   {
      return kd_;
   }

   // Desired joint stiffness, can be zero to do only torque control
   public void setKd(double kd)
   {
      kd_ = kd;
   }

   // Desired joint damping, can be zero to do only torque control
   public double getQDesired()
   {
      return qDesired_;
   }

   // Desired joint damping, can be zero to do only torque control
   public void setQDesired(double qDesired)
   {
      qDesired_ = qDesired;
   }

   // Desired position
   public double getQdDesired()
   {
      return qdDesired_;
   }

   // Desired position
   public void setQdDesired(double qdDesired)
   {
      qdDesired_ = qdDesired;
   }

   @Override
   public boolean epsilonEquals(JointDesired other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsEnum(this.controlMode_, other.controlMode_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tau_, other.tau_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kp_, other.kp_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.kd_, other.kd_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.qDesired_, other.qDesired_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.qdDesired_, other.qdDesired_, epsilon))
         return false;

      return true;
   }

   @Override
   public boolean equals(Object other)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;
      if (!(other instanceof JointDesired))
         return false;

      JointDesired otherMyClass = (JointDesired) other;

      if (this.controlMode_ != otherMyClass.controlMode_)
         return false;

      if (this.tau_ != otherMyClass.tau_)
         return false;

      if (this.kp_ != otherMyClass.kp_)
         return false;

      if (this.kd_ != otherMyClass.kd_)
         return false;

      if (this.qDesired_ != otherMyClass.qDesired_)
         return false;

      if (this.qdDesired_ != otherMyClass.qdDesired_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointDesired {");
      builder.append("controlMode=");
      builder.append(this.controlMode_);

      builder.append(", ");
      builder.append("tau=");
      builder.append(this.tau_);

      builder.append(", ");
      builder.append("kp=");
      builder.append(this.kp_);

      builder.append(", ");
      builder.append("kd=");
      builder.append(this.kd_);

      builder.append(", ");
      builder.append("qDesired=");
      builder.append(this.qDesired_);

      builder.append(", ");
      builder.append("qdDesired=");
      builder.append(this.qdDesired_);

      builder.append("}");
      return builder.toString();
   }
}