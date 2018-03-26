package it.iit.iCub.messages.it.iit.yarp;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class ForceSensor extends Packet<ForceSensor> implements Settable<ForceSensor>, EpsilonComparable<ForceSensor>
{
   public double Fx_;
   public double Fy_;
   public double Fz_;
   public double tauX_;
   public double tauY_;
   public double tauZ_;

   public ForceSensor()
   {

   }

   public ForceSensor(ForceSensor other)
   {
      set(other);
   }

   public void set(ForceSensor other)
   {
      Fx_ = other.Fx_;

      Fy_ = other.Fy_;

      Fz_ = other.Fz_;

      tauX_ = other.tauX_;

      tauY_ = other.tauY_;

      tauZ_ = other.tauZ_;
   }

   public double getFx()
   {
      return Fx_;
   }

   public void setFx(double Fx)
   {
      Fx_ = Fx;
   }

   public double getFy()
   {
      return Fy_;
   }

   public void setFy(double Fy)
   {
      Fy_ = Fy;
   }

   public double getFz()
   {
      return Fz_;
   }

   public void setFz(double Fz)
   {
      Fz_ = Fz;
   }

   public double getTauX()
   {
      return tauX_;
   }

   public void setTauX(double tauX)
   {
      tauX_ = tauX;
   }

   public double getTauY()
   {
      return tauY_;
   }

   public void setTauY(double tauY)
   {
      tauY_ = tauY;
   }

   public double getTauZ()
   {
      return tauZ_;
   }

   public void setTauZ(double tauZ)
   {
      tauZ_ = tauZ;
   }

   @Override
   public boolean epsilonEquals(ForceSensor other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.Fx_, other.Fx_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.Fy_, other.Fy_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.Fz_, other.Fz_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tauX_, other.tauX_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tauY_, other.tauY_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.tauZ_, other.tauZ_, epsilon))
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
      if (!(other instanceof ForceSensor))
         return false;

      ForceSensor otherMyClass = (ForceSensor) other;

      if (this.Fx_ != otherMyClass.Fx_)
         return false;

      if (this.Fy_ != otherMyClass.Fy_)
         return false;

      if (this.Fz_ != otherMyClass.Fz_)
         return false;

      if (this.tauX_ != otherMyClass.tauX_)
         return false;

      if (this.tauY_ != otherMyClass.tauY_)
         return false;

      if (this.tauZ_ != otherMyClass.tauZ_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ForceSensor {");
      builder.append("Fx=");
      builder.append(this.Fx_);

      builder.append(", ");
      builder.append("Fy=");
      builder.append(this.Fy_);

      builder.append(", ");
      builder.append("Fz=");
      builder.append(this.Fz_);

      builder.append(", ");
      builder.append("tauX=");
      builder.append(this.tauX_);

      builder.append(", ");
      builder.append("tauY=");
      builder.append(this.tauY_);

      builder.append(", ");
      builder.append("tauZ=");
      builder.append(this.tauZ_);

      builder.append("}");
      return builder.toString();
   }
}