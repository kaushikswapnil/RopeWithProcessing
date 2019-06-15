class Circle extends PhysicWorldEntityImpl
{
  Circle(float centerX, float centerY, float radius, float mass)
  {
     super(radius*2, radius*2, centerX, centerY, mass);
  }
  
  FixtureDef GetFixtureDefinition()
  {
    CircleShape circleShape = new CircleShape();
    float radius = ConvertScalarPixelsToPhysicWorldUnit(m_Dimensions.x/2);
    
    circleShape.m_radius = radius;
    
    float area = (PI * radius * radius);
    
    FixtureDef fixDef = new FixtureDef();
    fixDef.shape = circleShape;
    fixDef.density = m_Mass/area;
    fixDef.friction = 0.3;
    fixDef.restitution = 0.5;
     
    return fixDef;
  }
  
  void Display()
  {
    Vec2 pos = GetPhysicWorld().getBodyPixelCoord(m_Body);
    
    pushMatrix();
    
    stroke(0);
    fill(175);
    
    translate(pos.x, pos.y);
    
    ellipse(0, 0, m_Dimensions.x, m_Dimensions.x);
    
    popMatrix();
  }
  
  float GetShapeArea()
  {
    float radius = ConvertScalarPixelsToPhysicWorldUnit(m_Dimensions.x/2);
    
    return (PI * radius * radius);
  }
}

class StaticCircle extends Circle
{
  StaticCircle(float centerX, float centerY, float radius, float mass)
  {
     super(centerX, centerY, radius, mass); 
  }
  
  BodyDef GetBodyDefinition(float posX, float posY)
  {
     BodyDef bodyDef = new BodyDef();
     bodyDef.type = BodyType.STATIC;
     bodyDef.position.set(GetPhysicWorld().coordPixelsToWorld(posX, posY));
     
     return bodyDef;
  }
}
