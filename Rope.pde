class Rope
{
   ArrayList<IRopeControlPoint> m_ControlPoints; 
   int m_NumControlPoints;
   float m_RopeLength;
   float m_RopeWidth;
   
   float m_RopeMass;
   float[] m_SpringConstants; //The first one is for free rope and second for tight rope 
   float m_SpringFrictionCoeff;

   int m_DrawMode; //0 = draw bezier and rope 1 = draw rope control points 2 = draw bezier, 3 = drawDebugMode
   
   int m_CPIndexWithWeight; //-1 if there is no weight, or index value if present

   Rope(float ropeLength, float ropeWidth, Vec2 startPos, float ropeMass, float springConstant, float springFrictionCoeff, int numControlPoints, Vec2 initialDir, int drawMode)
   {
      m_ControlPoints = new ArrayList<IRopeControlPoint>();
      
      m_CPIndexWithWeight = -1;
      
      m_RopeWidth = ropeWidth;
      m_RopeLength = ropeLength;
      
      m_RopeMass = ropeMass;
      m_SpringConstants = new float[2];
      
      m_SpringConstants[0] = springConstant;
      m_SpringConstants[1] = 0;
      m_SpringFrictionCoeff = springFrictionCoeff;
      
      m_DrawMode = drawMode;
      
      m_NumControlPoints = numControlPoints;
      
      int numPackedCircles = (int)(m_RopeLength/m_RopeWidth);
      if (numPackedCircles < 1)
      {
         m_RopeWidth = 2*m_RopeLength;
      }

      m_NumControlPoints = Math.max(2, Math.min(numPackedCircles, m_NumControlPoints));
      
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
      //curPos.addLocal(posIncrement);
      //m_ControlPoints.add(new CircleRopeControlPoint(curPos.x, curPos.y, m_RopeWidth/2, cpMass*, false));   
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
        case 3:
        case 0:
        DisplayRopeControlPoints();
        DrawBezierRope();
        break;
        
        case 1:
        DisplayRopeControlPoints();
        break;
        
        case 2:
        DrawBezierRope();
        break;
     }
     
     if (HasAttachedPendulumWeight())
     {
       DrawAttachedPendulumWeight();
     }
   }
   
   void Update()
   {
       PhysicsUpdate();
       
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
   
   void PhysicsUpdate()
   {
      ApplyNaturalForces(); 
   }
   
   void ApplyNaturalForces()
   {
     float segmentLength = GetSegmentLength();
     
     //Calculate tension and spring force separately for rope under tension     
     if (HasAttachedPendulumWeight())
     {
        ApplyNaturalForcesOnTautSection(segmentLength);     
     }
     
     int startingIndexForFreeRope = Math.max(1, m_CPIndexWithWeight + 1);
     
     for (int cpIter = startingIndexForFreeRope; cpIter < m_NumControlPoints; ++cpIter)
     {
        //ApplyGravity 
         {
           ApplyGravityOnFreePoint(m_ControlPoints.get(cpIter));
         }

         //Apply spring force
         {             
           ApplyFreeSpringForces(m_ControlPoints.get(cpIter), m_ControlPoints.get(cpIter - 1), segmentLength);
         }
     }
   }
   
   void ApplyNaturalForcesOnTautSection(float segmentLength)
   {
      //Apply gravity to rope under tension
      float massOfRopeUnderTension = m_RopeMass*m_CPIndexWithWeight/(m_NumControlPoints-1);
      massOfRopeUnderTension += m_ControlPoints.get(m_CPIndexWithWeight).GetAttachedPendulumWeight().GetMass();
     
      Vec2 gravityForce = gravityAcc.mul(massOfRopeUnderTension);
      
      m_ControlPoints.get(m_CPIndexWithWeight).ApplyForce(gravityForce);
      
      if (m_DrawMode == 3)//Debug mode
      {
         float forceLength = GetPhysicWorld().scalarWorldToPixels(gravityForce.length());
       
         Vec2 textPos = new Vec2(m_ControlPoints.get(m_CPIndexWithWeight).GetPixelPosition().x, m_ControlPoints.get(m_CPIndexWithWeight).GetPixelPosition().y + 40);
         debugDisplay.PrintString(textPos, Float.toString(forceLength), 1, DebugColors.RED);
         
         Vec2 debugForce = GetPhysicWorld().vectorWorldToPixels(gravityForce);
         debugDisplay.DrawArrow(textPos, debugForce, massOfRopeUnderTension*10, DebugColors.RED );
      }
     
      IRopeControlPoint weightedPoint = m_ControlPoints.get(m_CPIndexWithWeight);
      IRopeControlPoint startPoint = m_ControlPoints.get(0);
      
      Vec2 forceOnFreePoint = new Vec2(0, 0);
      
      AddSpringForce(weightedPoint, startPoint, segmentLength*m_CPIndexWithWeight, m_SpringConstants[1], forceOnFreePoint);
      
      AddSpringFriction(weightedPoint, startPoint, forceOnFreePoint);
      
      Vec2 newCPPos = startPoint.GetPhysicPosition().mul(1);
      Vec2 ropeDir = (weightedPoint.GetPhysicPosition()).sub(newCPPos);
      ropeDir.normalize();

      weightedPoint.ApplyForce(forceOnFreePoint);
      
      if (m_DrawMode == 3)//Debug mode
      {
         float forceLength = GetPhysicWorld().scalarWorldToPixels(forceOnFreePoint.length());
       
         Vec2 textPos = new Vec2(weightedPoint.GetPixelPosition().x, weightedPoint.GetPixelPosition().y + 10);
         debugDisplay.PrintString(textPos, Float.toString(forceLength), 1, DebugColors.BLUE);
         
         Vec2 debugForce = GetPhysicWorld().vectorWorldToPixels(forceOnFreePoint);
         debugDisplay.DrawArrow(textPos, debugForce, GetPhysicWorld().scalarWorldToPixels(segmentLength*m_CPIndexWithWeight), DebugColors.BLUE );
      }
      
      Vec2 posIncrement = ropeDir.mul(segmentLength);
      for (int tCPiter = 1; tCPiter < m_CPIndexWithWeight; ++tCPiter)
      {
        newCPPos.addLocal(posIncrement);
        m_ControlPoints.get(tCPiter).SetPhysicTransform(newCPPos, 0.0f);
      }
   }
    
   void ApplyGravityOnFreePoint(IRopeControlPoint controlPoint)
   {
     Vec2 gravity = gravityAcc.mul(controlPoint.GetMass());
     
     if (m_DrawMode == 3)//Debug mode
     {
         float forceLength = GetPhysicWorld().scalarWorldToPixels(gravity.length());
       
         Vec2 textPos = new Vec2(controlPoint.GetPixelPosition().x, controlPoint.GetPixelPosition().y + 40);
         debugDisplay.PrintString(textPos, Float.toString(forceLength), 1, DebugColors.RED);
         
         Vec2 debugForce = GetPhysicWorld().vectorWorldToPixels(gravity);
         debugDisplay.DrawArrow(textPos, debugForce, controlPoint.GetMass()*10, DebugColors.RED );
     }
     
     controlPoint.ApplyForce(gravity);
   }
   
   void ApplyFreeSpringForces(IRopeControlPoint freePoint, IRopeControlPoint attachedPoint, float segmentDistance)
   {
     Vec2 forceOnFreePoint = new Vec2(0, 0);

     //Spring Force
     AddSpringForce(freePoint, attachedPoint, segmentDistance, m_SpringConstants[0],forceOnFreePoint);

     //Spring friction
     AddSpringFriction(freePoint, attachedPoint, forceOnFreePoint);
 //<>//
     Vec2 forceOnAttachedPoint = forceOnFreePoint.mul(-1);
     
     if (m_DrawMode == 3)//Debug mode
     {
       float forceLength = GetPhysicWorld().scalarWorldToPixels(forceOnAttachedPoint.length());
     
       Vec2 textPos = new Vec2(freePoint.GetPixelPosition().x, freePoint.GetPixelPosition().y + 10);
       debugDisplay.PrintString(textPos, Float.toString(forceLength), 1, DebugColors.BLUE);
       
       Vec2 debugForce = GetPhysicWorld().vectorWorldToPixels(forceOnFreePoint);
       debugDisplay.DrawArrow(textPos, debugForce, GetPhysicWorld().scalarWorldToPixels(segmentDistance), DebugColors.BLUE );
     }
     
     freePoint.ApplyForce(forceOnFreePoint);
     attachedPoint.ApplyForce(forceOnAttachedPoint);
   }
   
   void AddSpringForce(IRopeControlPoint freePoint, IRopeControlPoint attachedPoint, float segmentDistance, float springConstant, Vec2 outForce)
   {
     Vec2 freePointPos = freePoint.GetPhysicPosition();
     Vec2 attachedPointPos = attachedPoint.GetPhysicPosition();
     
     Vec2 distPoints = freePointPos.sub(attachedPointPos);
     
     float distPointMag = distPoints.length();
     float diffMagnitude = distPointMag - segmentDistance;
     distPoints.normalize();
     
     Vec2 springForce = (distPoints).mul(-springConstant * diffMagnitude);
     
     outForce.addLocal(springForce);
   }

   void AddSpringFriction(IRopeControlPoint freePoint, IRopeControlPoint attachedPoint, Vec2 outForce)
   {
     Vec2 freePointVelocity = freePoint.GetLinearVelocity();
     
     //With mass it seems to have a heavier effect. I kinda like that
     float springFrictionConstant = m_SpringFrictionCoeff;// * (freePoint.GetMass() + attachedPoint.GetMass());
     
     //Since we will already apply the springFriction again for the next point,
     //we dont need to calculate relative velocity
     //Vec2 attachedPointVelocity = attachedPoint.GetLinearVelocity();
     //Vec2 freePointVelRelToAttached = freePointVelocity.sub(attachedPointVelocity);
     //Vec2 springFriction = freePointVelRelToAttached.mul(-springFrictionConstant);
     
     Vec2 springFriction = freePointVelocity.mul(-springFrictionConstant);
     outForce.addLocal(springFriction);
   }
   
   float GetSegmentLength()
   {
     return ConvertScalarPixelsToPhysicWorldUnit(m_RopeLength)/(m_NumControlPoints-1); 
   }
   
   void DrawBezierRope()
   {
      //Draw two bezier curves. One for under tension and one for free spring
      int freeRopeStartingIndex = 0;
      int freeRopeEndingIndex = m_NumControlPoints - 1;
      
      if (HasAttachedPendulumWeight())
      {
        DrawBezierCurve(0, m_CPIndexWithWeight);
        freeRopeStartingIndex = m_CPIndexWithWeight;
      }
      
      DrawBezierCurve(freeRopeStartingIndex, freeRopeEndingIndex);
   }
   
   void DrawBezierCurve(int startingCPIndex, int endingCPIndex)
   {
     int curveOrder = endingCPIndex - startingCPIndex;
     
     for (float t = 0; IsLesserOrEqualWithEpsilon(t, 1.0f); t += 0.001f)
      {
         float pX = 0;
         float pY = 0;
         
         float tInverse = 1-t;
         
         for (int cpIter = startingCPIndex; cpIter <= endingCPIndex; ++cpIter)
         {
            Vec2 cpPixelPos = m_ControlPoints.get(cpIter).GetPixelPosition();

            int relIter = cpIter - startingCPIndex;
            float pointCoeff = (float)(Combination(curveOrder, relIter) * (Math.pow(tInverse, curveOrder - relIter)) * (Math.pow(t, relIter)));
            pX += pointCoeff * cpPixelPos.x;
            pY += pointCoeff * cpPixelPos.y;
         }
         
         stroke(0, 255, 0, 255);
         point(pX, pY);
      }
   }
   
   void DrawAttachedPendulumWeight()
   {
      IPendulumWeight weight = m_ControlPoints.get(m_CPIndexWithWeight).GetAttachedPendulumWeight();
      weight.Display();
   }
   
   void AttachPendulumWeight(int cpIndex, float mass)
   {
      cpIndex = Limit(cpIndex, 1, m_NumControlPoints - 1); 
      
      IRopeControlPoint cp = m_ControlPoints.get(cpIndex);
      PendulumWeightImpl weight = new PendulumWeightImpl(mass, cp);
      cp.AttachPendulumWeight(weight);
      
      m_CPIndexWithWeight = cpIndex;
      float cpMass = m_RopeMass/(m_NumControlPoints - 1);
      cpMass = cpMass*m_CPIndexWithWeight;
      cpMass += mass;
      
      m_SpringConstants[1] = 100*cpMass;
   }
   
   void DetachPendulumWeight()
   {
      IRopeControlPoint cp = m_ControlPoints.get(m_CPIndexWithWeight);
      cp.DetachPendulumWeight();
      m_CPIndexWithWeight = -1;
   }
   
   boolean HasAttachedPendulumWeight()
   {
     return m_CPIndexWithWeight != -1;
   }
   
   boolean HasTautSubSection()
   {
      if (HasAttachedPendulumWeight())
      {
         float segmentLength = GetSegmentLength();
         Vec2 freePointPos = m_ControlPoints.get(m_CPIndexWithWeight).GetPhysicPosition();
         Vec2 attachedPointPos = m_ControlPoints.get(0).GetPhysicPosition();
         
         return IsGreaterOrEqualWithEpsilon((freePointPos.sub(attachedPointPos)).length(), segmentLength * m_CPIndexWithWeight);
      }
      
      return false;
   }
}
