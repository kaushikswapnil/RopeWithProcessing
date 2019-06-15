class Rope
{
   ArrayList<IRopeControlPoint> m_ControlPoints; 
   int m_NumControlPoints;
   float m_RopeLength;
   float m_RopeWidth;
   
   float m_RopeMass;
   float m_SpringConstant;
   float m_SpringFriction;
   
   Rope(float ropeLength, float ropeWidth, Vec2 startPos, float ropeMass, float springConstant, float springFriction, int numControlPoints)
   {
      m_ControlPoints = new ArrayList<IRopeControlPoint>();
      
      m_RopeWidth = ropeWidth;
      m_RopeLength = ropeLength;
      
      m_RopeMass = ropeMass;
      m_SpringConstant = springConstant;
      m_SpringFriction = springFriction;
      
      m_NumControlPoints = numControlPoints;
      
      int numPackedCircles = (int)(m_RopeLength/m_RopeWidth);
      if (numPackedCircles < 1)
      {
         m_RopeWidth = m_RopeLength;
         numPackedCircles = 1;
      }

      m_NumControlPoints = Math.min(numPackedCircles, m_NumControlPoints);
      
      Vec2 ropeDirection = new Vec2(0, 1);
      
      float segmentLength = m_RopeLength/(m_NumControlPoints-1); //This is the pixel segment length
      
      Vec2 curPos = startPos;
      Vec2 posIncrement = ropeDirection.mul(segmentLength);
      float cpMass = m_RopeMass/(m_NumControlPoints-1);
      
      m_ControlPoints.add(new CircleRopeControlPoint(curPos.x, curPos.y, m_RopeWidth/2, cpMass, true));
      for (int iter = 1; iter < m_NumControlPoints; ++iter)
      {
        curPos.addLocal(posIncrement);
        m_ControlPoints.add(new CircleRopeControlPoint(curPos.x, curPos.y, m_RopeWidth/2, cpMass, false)); 
         
      }
   }
   
   void Display()
   {
      Vec2 prevPos = m_ControlPoints.get(0).GetPixelPosition();
      
      for (int iter = 0; iter < m_NumControlPoints; ++iter)
      {
        IRopeControlPoint controlPoint = m_ControlPoints.get(iter);
        Vec2 curPos = controlPoint.GetPixelPosition();
        
        if (iter > 0)
        {
           stroke(175);
           line(prevPos.x, prevPos.y, curPos.x, curPos.y);
        }
        
        controlPoint.Display();
        prevPos = curPos;
      }
   }
   
   void Update(float dt)
   {
       PhysicsUpdate(dt);
       Display();
       
       if (endPointAtMouse)
       {
          Vec2 startPos = m_ControlPoints.get(0).GetPixelPosition();
          Vec2 mouseRelDir = new Vec2(mouseX, mouseY);
          mouseRelDir.subLocal(startPos);
          mouseRelDir.normalize();
          Vec2 desiredPos = mouseRelDir.mul(m_RopeLength);
          desiredPos.addLocal(startPos);
          
          m_ControlPoints.get(m_NumControlPoints-1).SetPixelTransform(desiredPos, 0); 
       }
   }
   
   void PhysicsUpdate(float dt)
   {
      ApplyNaturalForces(dt); 
   }
   
   void ApplyNaturalForces(float dt)
   {
     float segmentLength = GetSegmentLength();
     for (int cpIter = 1; cpIter < m_NumControlPoints; ++cpIter)
     {
        //ApplyGravity 
         {
           ApplyGravity(m_ControlPoints.get(cpIter), dt);
         }
         
         //Apply spring force
         {             
           ApplySpringForces(m_ControlPoints.get(cpIter), m_ControlPoints.get(cpIter - 1), segmentLength);
         }
     }
   }
    
   void ApplyGravity(IRopeControlPoint controlPoint, float dt)
   {
     Vec2 gravity = gravityAcc.mul(controlPoint.GetMass());
     
     controlPoint.ApplyForce(gravity); //<>//
   }
   
   void ApplySpringForces(IRopeControlPoint freePoint, IRopeControlPoint attachedPoint, float segmentDistance)
   {
     //Spring Force
     Vec2 freePointPos = freePoint.GetPhysicPosition();
     Vec2 attachedPointPos = attachedPoint.GetPhysicPosition();
     
     Vec2 distPoints = freePointPos.sub(attachedPointPos);
     
     float distPointMag = distPoints.length();
     float diffMagnitude = distPointMag - segmentDistance;
     distPoints.normalize();
     
     Vec2 springForce = (distPoints).mul(-m_SpringConstant * diffMagnitude);
     

     //Spring friction
     Vec2 freePointVelocity = freePoint.GetLinearVelocity();
     Vec2 attachedPointVelocity = attachedPoint.GetLinearVelocity();
     
     Vec2 freePointVelRelToAttached = freePointVelocity.sub(attachedPointVelocity);
     
     Vec2 springFriction = freePointVelRelToAttached.mul(-m_SpringFriction);
     
     Vec2 forceOnFreePoint = springForce.add(springFriction);
     Vec2 forceOnAttachedPoint = forceOnFreePoint.mul(-1);
     
     freePoint.ApplyForce(forceOnFreePoint);
     attachedPoint.ApplyForce(forceOnAttachedPoint); //<>//
   }
   
   float GetSegmentLength()
   {
     return ConvertScalarPixelsToPhysicWorldUnit(m_RopeLength)/(m_NumControlPoints-1); 
   }
}
