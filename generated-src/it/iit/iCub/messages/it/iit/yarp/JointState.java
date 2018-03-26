package it.iit.iCub.messages.it.iit.yarp;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class JointState extends Packet<JointState> implements Settable<JointState>, EpsilonComparable<JointState>
{
   public double tau_;
   public double q_;
   public double qd_;

   public JointState()
   {

   }

   public JointState(JointState other)
   {
      set(other);
   }

   public void set(JointState other)
   {
      tau_ = other.tau_;

      q_ = other.q_;

      qd_ = other.qd_;
   }

   public double getTau()
   {
      return tau_;
   }

   public void setTau(double tau)
   {
      tau_ = tau;
   }

   public double getQ()
   {
      return q_;
   }

   public void setQ(double q)
   {
      q_ = q;
   }

   public double getQd()
   {
      return qd_;
   }

   public void setQd(double qd)
   {
      qd_ = qd;
   }

   @Override
   public boolean epsilonEquals(JointState other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tau_, other.tau_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.q_, other.q_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.qd_, other.qd_, epsilon))
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
      if (!(other instanceof JointState))
         return false;

      JointState otherMyClass = (JointState) other;

      if (this.tau_ != otherMyClass.tau_)
         return false;

      if (this.q_ != otherMyClass.q_)
         return false;

      if (this.qd_ != otherMyClass.qd_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("JointState {");
      builder.append("tau=");
      builder.append(this.tau_);

      builder.append(", ");
      builder.append("q=");
      builder.append(this.q_);

      builder.append(", ");
      builder.append("qd=");
      builder.append(this.qd_);

      builder.append("}");
      return builder.toString();
   }
}