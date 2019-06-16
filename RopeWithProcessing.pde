import shiffman.box2d.*;
import org.jbox2d.collision.shapes.*;
import org.jbox2d.common.*;
import org.jbox2d.dynamics.*;
import org.jbox2d.dynamics.*;

Box2DProcessing box2d;
ArrayList<Boundary> boundaries;

Rope rope;

Vec2 gravityAcc = new Vec2(0, -10);

boolean endPointAtMouse = false;

void setup()
{
  size(1200, 800);
  
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  box2d.setGravity(0, 0);
  
  float ropeLength = 500; 
  float ropeWidth = 10;
  Vec2 startPos = new Vec2(600, 50);
  
  int numControlPoints = 8;
  float ropeMass = 20;
  float springConstant = 1000*(ropeMass/(numControlPoints-1));
  float springFriction = 0.99;
  Vec2 initialDir = new Vec2(1, 0);
  int drawMode = 0;
  rope = new Rope(ropeLength, ropeWidth, startPos, ropeMass, springConstant, springFriction, numControlPoints, initialDir, drawMode);
  
  int pendulumCPIndex = numControlPoints - 1 - 4;
  float pendulumWeight = 0.1;
  rope.AttachPendulumWeight(pendulumCPIndex, pendulumWeight);  
  
  boundaries = new ArrayList<Boundary>();
  
  float boundWidth = 10;
  float halfBoundWidth = boundWidth/2;
  boundaries.add(new Boundary(halfBoundWidth, height/2, boundWidth, height));
  boundaries.add(new Boundary(width/2, halfBoundWidth, width, boundWidth));
  boundaries.add(new Boundary(width - halfBoundWidth, height/2, boundWidth, height));
  boundaries.add(new Boundary(width/2, height - halfBoundWidth, width, boundWidth));
  
  //frameRate(5);
}

void draw()
{
  background(255);
  
  box2d.step();
  
  rope.Update();
  rope.Display();
  
  for (Boundary boundary : boundaries)
  {
    boundary.Display();
  }
}

void keyPressed()
{
   if (key == ' ')
   {
      int currentDrawMode = rope.m_DrawMode;
      rope.m_DrawMode = (++currentDrawMode)%3;
   }
}

Box2DProcessing GetPhysicWorld()
{
   return box2d; 
}

float ConvertScalarPixelsToPhysicWorldUnit(float value)
{
   return box2d.scalarPixelsToWorld(value); 
}

int Combination(int n, int i) //nCi
{
  return Factorial(n)/(Factorial(i) * Factorial(n-i));
}

int Factorial(int n)
{
   int answer = 1;
   for (int i = 1; i <= n; ++i)
   {
      answer *= i; 
   }
   
   return answer; 
}

boolean IsNullWithEpsilon(float value)
{
  return abs(value - 0.0) <= EPSILON;
}

boolean IsGreaterWithEpsilon(float a, float b)
{
  return (a - b) > EPSILON;
}

boolean IsLesserWithEpsilon(float a, float b)
{
  return (a - b) < EPSILON;
}

boolean IsEqualWithEpsilon(float a, float b)
{
  return IsNullWithEpsilon(a-b); 
}

boolean IsGreaterOrEqualWithEpsilon(float a, float b)
{
   return IsGreaterWithEpsilon(a, b) || IsEqualWithEpsilon(a, b); 
}

boolean IsLesserOrEqualWithEpsilon(float a, float b)
{
   return IsLesserWithEpsilon(a, b) || IsEqualWithEpsilon(a, b); 
}

int Limit(int value, int minValue, int maxValue) {
    return Math.max(minValue, Math.min(value, maxValue));
}
