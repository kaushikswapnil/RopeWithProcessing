class Rope
{
   ArrayList<IRopeControlPoint> m_ControlPoints; 
   int m_NumControlPoints;
   float m_RopeLength;
   float m_RopeWidth;
   
   float m_RopeMass;
   float m_SpringConstant;
   float m_SpringFriction;

   int m_DrawMode; //0 = draw bezier and rope 1 = draw rope control points 2 = draw bezier
   
   Rope(float ropeLength, float ropeWidth, Vec2 startPos, float ropeMass, float springConstant, float springFriction, int numControlPoints, Vec2 initialDir, int drawMode)
   {
      m_ControlPoints = new ArrayList<IRopeControlPoint>();
      
      m_RopeWidth = ropeWidth;
      m_RopeLength = ropeLength;
      
      m_RopeMass = ropeMass;
      m_SpringConstant = springConstant;
      m_SpringFriction = springFriction;
      
      m_NumControlPoints = numControlPoints;
      
      m_DrawMode = Limit(drawMode, 0, 2);
      
      int numPackedCircles = (int)(m_RopeLength/m_RopeWidth);
      if (numPackedCircles < 1)
      {
         m_RopeWidth = m_RopeLength;
         numPackedCircles = 1;
      }

      m_NumControlPoints = Math.min(numPackedCircles, m_NumControlPoints);
      
      Vec2 ropeDirection = initialDir;
      ropeDirection.normalize();
      
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
   
   void DisplayRopeControlPoints()
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
   
   void Display()
   {
     switch(m_DrawMode)
     {
        case 0:
        DisplayRopeControlPoints();
        DrawBezier();
        break;
        
        case 1:
        DisplayRopeControlPoints();
        break;
        
        case 2:
        DrawBezier();
        break;
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
     
     controlPoint.ApplyForce(gravity);
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
     //Since we will already apply the springFriction again for the next point,
     //we dont need to calculate relative velocity
     //Vec2 attachedPointVelocity = attachedPoint.GetLinearVelocity();
     //Vec2 freePointVelRelToAttached = freePointVelocity.sub(attachedPointVelocity);
     //Vec2 springFriction = freePointVelRelToAttached.mul(-m_SpringFriction);
     
     Vec2 springFriction = freePointVelocity.mul(-m_SpringFriction); //<>//
     
     Vec2 forceOnFreePoint = springForce.add(springFriction);
     Vec2 forceOnAttachedPoint = forceOnFreePoint.mul(-1);
     
     freePoint.ApplyForce(forceOnFreePoint);
     attachedPoint.ApplyForce(forceOnAttachedPoint);
   }
   
   float GetSegmentLength()
   {
     return ConvertScalarPixelsToPhysicWorldUnit(m_RopeLength)/(m_NumControlPoints-1); 
   }
   
   void DrawBezier()
   {
      int curveOrder = m_NumControlPoints - 1;
      
      for (float t = 0; IsLesserOrEqualWithEpsilon(t, 1.0f); t += 0.001f)
      {
         float pX = 0;
         float pY = 0;
         
         float tInverse = 1-t;
         
         for (int cpIter = 0; cpIter < m_NumControlPoints; ++cpIter)
         {
            Vec2 cpPixelPos = m_ControlPoints.get(cpIter).GetPixelPosition();

            float pointCoeff = (float)(Combination(curveOrder, cpIter) * (Math.pow(tInverse, curveOrder - cpIter)) * (Math.pow(t, cpIter)));
            pX += pointCoeff * cpPixelPos.x;
            pY += pointCoeff * cpPixelPos.y;
         }
         
         stroke(0, 255, 0, 255);
         point(pX, pY);
      }
   }
}
