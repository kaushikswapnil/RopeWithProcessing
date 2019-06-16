interface IRopeControlPoint
{
   void Display(); 
   Vec2 GetPixelPosition();
   float GetMass();
   void ApplyForce(Vec2 pixelForce);
   Vec2 GetPhysicPosition();
   Vec2 GetLinearVelocity();
   
   void SetPhysicTransform(Vec2 newPos, float newAng);
   
   void SetPixelTransform(Vec2 pixelPos, float pixelAng);
   
   void AttachPendulumWeight(IPendulumWeight pendulumWeight);
   IPendulumWeight GetAttachedPendulumWeight();
   boolean HasAttachedPendulumWeight();
}

class CircleRopeControlPoint implements IRopeControlPoint 
{
  Circle m_PhysicShape;
  IPendulumWeight m_AttachedPendulumWeight;
  
  CircleRopeControlPoint(float centerX, float centerY, float radius, float cpMass, boolean isStatic)
  {
    if (!isStatic)
    {
      m_PhysicShape = new Circle(centerX, centerY, radius, cpMass);
    }
    else
    {
      m_PhysicShape = new StaticCircle(centerX, centerY, radius, cpMass);
    }
  }
  
  void Display()
  {
    m_PhysicShape.Display();
    if (HasAttachedPendulumWeight())
    {
      m_AttachedPendulumWeight.Display();
    }
  }
  
  Vec2 GetPixelPosition()
  {
    return m_PhysicShape.GetPixelPosition(); 
  }
  
  float GetMass()
  {
    float mass = m_PhysicShape.GetMass();
    
    if (HasAttachedPendulumWeight())
    {
       mass +=  m_AttachedPendulumWeight.GetMass();
    }
    
    return mass; 
  }
  
  void ApplyForce(Vec2 force)
  {
    m_PhysicShape.ApplyForce(force);
  }
  
  Vec2 GetPhysicPosition()
  {
   return m_PhysicShape.GetPhysicPosition(); 
  }
  
  Vec2 GetLinearVelocity()
  {
   return m_PhysicShape.GetLinearVelocity(); 
  }
  
  void SetPixelTransform(Vec2 pixelPos, float pixelAng)
  {
   Vec2 pos = GetPhysicWorld().coordPixelsToWorld(pixelPos); 
   m_PhysicShape.SetTransform(pos, -pixelAng); 
  }
  
  void AttachPendulumWeight(IPendulumWeight pendulumWeight)
  {
     m_AttachedPendulumWeight = pendulumWeight; 
  }
  
  IPendulumWeight GetAttachedPendulumWeight()
  {
     return m_AttachedPendulumWeight; 
  }
  
  boolean HasAttachedPendulumWeight()
  {
     return m_AttachedPendulumWeight != null; 
  }
  
  void SetPhysicTransform(Vec2 newPos, float newAng)
  {
     m_PhysicShape.SetTransform(newPos, -newAng); 
  }
}
