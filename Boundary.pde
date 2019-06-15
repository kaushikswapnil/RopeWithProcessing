class Boundary extends PhysicWorldEntityImpl
{
  Boundary(float centerX, float centerY, float boundWidth, float boundHeight)
  {
    super(boundWidth, boundHeight, centerX, centerY, 0);
  }
  
  BodyDef GetBodyDefinition(float posX, float posY)
  {
    BodyDef bd = new BodyDef();
    bd.position.set(GetPhysicWorld().coordPixelsToWorld(posX, posY));
    bd.type = BodyType.STATIC;
    return bd;
  }
  
  void Display()
   {
    Vec2 pos = GetPhysicWorld().getBodyPixelCoord(m_Body);
    float angle = m_Body.getAngle();
    
    pushMatrix();
    
    stroke(0);
    fill(0);
    
    translate(pos.x, pos.y);
    rotate(-angle);
    
    rectMode(CENTER);
    rect(0, 0, m_Dimensions.x, m_Dimensions.y);
    
    popMatrix();
   }
}
