package it.iit.iCub.messages.it.iit.yarp;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.interfaces.EpsilonComparable;
import us.ihmc.euclid.interfaces.Settable;

public class IMUState extends Packet<IMUState> implements Settable<IMUState>, EpsilonComparable<IMUState>
{
   public double xdd_;
   public double ydd_;
   public double zdd_;
   public double wx_;
   public double wy_;
   public double wz_;
   public double qs_;
   public double qx_;
   public double qy_;
   public double qz_;

   public IMUState()
   {

   }

   public IMUState(IMUState other)
   {
      set(other);
   }

   public void set(IMUState other)
   {
      xdd_ = other.xdd_;

      ydd_ = other.ydd_;

      zdd_ = other.zdd_;

      wx_ = other.wx_;

      wy_ = other.wy_;

      wz_ = other.wz_;

      qs_ = other.qs_;

      qx_ = other.qx_;

      qy_ = other.qy_;

      qz_ = other.qz_;
   }

   public double getXdd()
   {
      return xdd_;
   }

   public void setXdd(double xdd)
   {
      xdd_ = xdd;
   }

   public double getYdd()
   {
      return ydd_;
   }

   public void setYdd(double ydd)
   {
      ydd_ = ydd;
   }

   public double getZdd()
   {
      return zdd_;
   }

   public void setZdd(double zdd)
   {
      zdd_ = zdd;
   }

   public double getWx()
   {
      return wx_;
   }

   public void setWx(double wx)
   {
      wx_ = wx;
   }

   public double getWy()
   {
      return wy_;
   }

   public void setWy(double wy)
   {
      wy_ = wy;
   }

   public double getWz()
   {
      return wz_;
   }

   public void setWz(double wz)
   {
      wz_ = wz;
   }

   public double getQs()
   {
      return qs_;
   }

   public void setQs(double qs)
   {
      qs_ = qs;
   }

   public double getQx()
   {
      return qx_;
   }

   public void setQx(double qx)
   {
      qx_ = qx;
   }

   public double getQy()
   {
      return qy_;
   }

   public void setQy(double qy)
   {
      qy_ = qy;
   }

   public double getQz()
   {
      return qz_;
   }

   public void setQz(double qz)
   {
      qz_ = qz;
   }

   @Override
   public boolean epsilonEquals(IMUState other, double epsilon)
   {
      if (other == null)
         return false;
      if (other == this)
         return true;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.xdd_, other.xdd_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.ydd_, other.ydd_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.zdd_, other.zdd_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wx_, other.wx_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wy_, other.wy_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.wz_, other.wz_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.qs_, other.qs_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.qx_, other.qx_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.qy_, other.qy_, epsilon))
         return false;

      if (!us.ihmc.idl.IDLTools.epsilonEqualsPrimitive(this.qz_, other.qz_, epsilon))
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
      if (!(other instanceof IMUState))
         return false;

      IMUState otherMyClass = (IMUState) other;

      if (this.xdd_ != otherMyClass.xdd_)
         return false;

      if (this.ydd_ != otherMyClass.ydd_)
         return false;

      if (this.zdd_ != otherMyClass.zdd_)
         return false;

      if (this.wx_ != otherMyClass.wx_)
         return false;

      if (this.wy_ != otherMyClass.wy_)
         return false;

      if (this.wz_ != otherMyClass.wz_)
         return false;

      if (this.qs_ != otherMyClass.qs_)
         return false;

      if (this.qx_ != otherMyClass.qx_)
         return false;

      if (this.qy_ != otherMyClass.qy_)
         return false;

      if (this.qz_ != otherMyClass.qz_)
         return false;

      return true;
   }

   @Override
   public java.lang.String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("IMUState {");
      builder.append("xdd=");
      builder.append(this.xdd_);

      builder.append(", ");
      builder.append("ydd=");
      builder.append(this.ydd_);

      builder.append(", ");
      builder.append("zdd=");
      builder.append(this.zdd_);

      builder.append(", ");
      builder.append("wx=");
      builder.append(this.wx_);

      builder.append(", ");
      builder.append("wy=");
      builder.append(this.wy_);

      builder.append(", ");
      builder.append("wz=");
      builder.append(this.wz_);

      builder.append(", ");
      builder.append("qs=");
      builder.append(this.qs_);

      builder.append(", ");
      builder.append("qx=");
      builder.append(this.qx_);

      builder.append(", ");
      builder.append("qy=");
      builder.append(this.qy_);

      builder.append(", ");
      builder.append("qz=");
      builder.append(this.qz_);

      builder.append("}");
      return builder.toString();
   }
}