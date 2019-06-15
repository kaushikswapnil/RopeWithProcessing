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
  size(1200, 1200);
  
  box2d = new Box2DProcessing(this);
  box2d.createWorld();
  box2d.setGravity(0, 0);
  
  float ropeLength = 600; 
  float ropeWidth = 10;
  Vec2 startPos = new Vec2(400, 50);
  
  int numControlPoints = 10;
  float ropeMass = 100;
  float springConstant = 1000*(ropeMass/(numControlPoints-1));
  float springFriction = 0.2;
  rope = new Rope(ropeLength, ropeWidth, startPos, ropeMass, springConstant, springFriction, numControlPoints); 
  
  boundaries = new ArrayList<Boundary>();
  
  float boundWidth = 20;
  float halfBoundWidth = boundWidth/2;
  //boundaries.add(new Boundary(halfBoundWidth, height/2, boundWidth, height));
  //boundaries.add(new Boundary(width/2, halfBoundWidth, width, boundWidth));
  //boundaries.add(new Boundary(width - halfBoundWidth, height/2, boundWidth, height));
  //boundaries.add(new Boundary(width/2, height - halfBoundWidth, width, boundWidth));
}

void draw()
{
  background(255);
  
  box2d.step();
  
  rope.Update(0);
  
  for (Boundary boundary : boundaries)
  {
    boundary.Display();
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
