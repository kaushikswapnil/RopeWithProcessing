interface IPendulumWeight
{
 float GetMass();
 void Display();
 IRopeControlPoint GetAttachPoint();
}

class PendulumWeightImpl implements IPendulumWeight
{
   float m_Mass;
   IRopeControlPoint m_AttachPoint;
   
   PendulumWeightImpl(float mass, IRopeControlPoint attachedPoint)
   {
     m_Mass = mass;
     m_AttachPoint = attachedPoint;
   }  
   
   float GetMass()
   {
     return m_Mass; 
   }
   
   IRopeControlPoint GetAttachPoint()
   {
     return m_AttachPoint;
   }
   
   void Display()
   {
     stroke(255, 0, 0);
     fill(255, 0, 0);
     Vec2 attachPointPos = m_AttachPoint.GetPixelPosition();
     ellipse(attachPointPos.x, attachPointPos.y, 16, 16); //Slightly offsetted    
   }   
}
