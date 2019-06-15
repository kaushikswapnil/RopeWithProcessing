interface IPhysicWorldEntity
{
   void CreateBody(float posX, float posY);
   void KillBody();
   
   BodyDef GetBodyDefinition(float posX, float posY);
   FixtureDef GetFixtureDefinition();
   Body GetPhysicBody();
   Vec2 GetDimensions();
   Vec2 GetPixelPosition();
   Vec2 GetPhysicPosition();
   Vec2 GetLinearVelocity();
   float GetShapeArea(); 
   float GetMass();
   
   void ApplyForce(Vec2 force);
   
   void Display();
   
   void SetTransform(Vec2 pos, float ang);
}

class PhysicWorldEntityImpl implements IPhysicWorldEntity
{
    Body m_Body;
    Vec2 m_Dimensions;
    float m_Mass;
    
    PhysicWorldEntityImpl(float entityDimensionX, float entityDimensionY, float posX, float posY, float mass)
    {
      m_Dimensions = new Vec2(entityDimensionX, entityDimensionY);
      m_Mass = mass;
      
      CreateBody(posX, posY);
    }
    
    void CreateBody(float posX, float posY)
    {
       m_Body = GetPhysicWorld().createBody(GetBodyDefinition(posX, posY));
       
       m_Body.createFixture(GetFixtureDefinition());
    }
    
    void KillBody()
    {
       GetPhysicWorld().destroyBody(m_Body); 
    }
    
    BodyDef GetBodyDefinition(float posX, float posY)
    {
       BodyDef bodyDef = new BodyDef();
       bodyDef.type = BodyType.DYNAMIC;
       bodyDef.position.set(GetPhysicWorld().coordPixelsToWorld(posX, posY));
       
       return bodyDef;
    }
    
    FixtureDef GetFixtureDefinition()
    {
       PolygonShape ps = new PolygonShape();
       
       float shapeWidth = ConvertScalarPixelsToPhysicWorldUnit(m_Dimensions.x/2);
       float shapeHeight = ConvertScalarPixelsToPhysicWorldUnit(m_Dimensions.y/2);
       
       ps.setAsBox(shapeWidth, shapeHeight);
     
       FixtureDef fixDef = new FixtureDef();
       fixDef.shape = ps;
       fixDef.density = m_Mass/GetShapeArea();
       fixDef.friction = 0.0;
       fixDef.restitution = 0.0;
       
       return fixDef;
    }
    
   Body GetPhysicBody()
   {
     return m_Body; 
   }
   
   Vec2 GetDimensions() 
   {
     return m_Dimensions;  
   }
   
   Vec2 GetPixelPosition()
   {
     return GetPhysicWorld().getBodyPixelCoord(m_Body); 
   }
   
   Vec2 GetPhysicPosition()
   {
     return m_Body.getWorldCenter(); 
   }
   
   void Display()
   {
    Vec2 pos = GetPhysicWorld().getBodyPixelCoord(m_Body);
    float angle = m_Body.getAngle();
    
    pushMatrix();
    
    stroke(0);
    fill(175);
    
    translate(pos.x, pos.y);
    rotate(-angle);
    
    rectMode(CENTER);
    rect(0, 0, m_Dimensions.x, m_Dimensions.y);
    
    popMatrix();
   }
   
   float GetShapeArea()
   {
     float shapeWidth = ConvertScalarPixelsToPhysicWorldUnit(m_Dimensions.x/2);
     float shapeHeight = ConvertScalarPixelsToPhysicWorldUnit(m_Dimensions.y/2);
     
     return shapeWidth*shapeHeight;
   }
   
   float GetMass()
   {
     return m_Mass; 
   }
   
   void ApplyForce(Vec2 force)
   {
     m_Body.applyForce(force, m_Body.getWorldCenter());
   }
   
   Vec2 GetLinearVelocity()
   {
      return m_Body.getLinearVelocity(); 
   }
   
   void SetTransform(Vec2 pos, float ang)
   {
      m_Body.setTransform(pos, ang); 
   }
}
