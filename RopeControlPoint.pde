interface IRopeControlPoint
{
   void Display(); 
   Vec2 GetPixelPosition();
   float GetMass();
   void ApplyForce(Vec2 pixelForce);
   Vec2 GetPhysicPosition();
   Vec2 GetLinearVelocity();
   
   void SetPixelTransform(Vec2 pixelPos, float pixelAng);
}

class CircleRopeControlPoint implements IRopeControlPoint 
{
  Circle m_PhysicShape;
  
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
  }
  
  Vec2 GetPixelPosition()
  {
    return m_PhysicShape.GetPixelPosition(); 
  }
  
  float GetMass()
  {
    return m_PhysicShape.GetMass(); 
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
}
