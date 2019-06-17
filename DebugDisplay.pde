class DebugDisplay
{
  void DrawLine(Vec2 pos, Vec2 dir, float length)
  {
     dir.normalize();
     dir.mulLocal(length);
     
     stroke(0, 0, 255);
     line (pos.x, pos.y, pos.x + dir.x, pos.y + dir.y);
  }
  
  void DrawLine(Vec2 startPos, Vec2 endPos)
  {
     stroke(0, 0, 255);
     line (startPos.x, startPos.y, endPos.x, endPos.y); 
  }
  
  void DrawArrow(Vec2 pos, Vec2 dir, float length)
  {
     dir.normalize();
     Vec2 endPos = pos.add(dir.mul(length)); 
     DrawLine(pos, endPos);
     
     Vec2 revDir = dir.mul(-1);
     float rotAngle = 45;
     float rotAngleRadians = (float)Math.toRadians(rotAngle);
     float cosAngle = (float)Math.cos(rotAngleRadians);
     float sinAngle = (float)Math.cos(rotAngleRadians);
     float cosNegAngle = (float)Math.cos(-rotAngleRadians);
     float sinNegAngle = (float)Math.cos(-rotAngleRadians);
     
     float dirX = revDir.x;
     float dirY = revDir.y;
     
     Vec2 arrowHead1 = new Vec2(((dirX*cosAngle) - (dirY*sinAngle)), ((dirX*sinAngle) + (dirY*cosAngle)));
     Vec2 arrowHead2 = new Vec2(((dirX*cosNegAngle) - (dirY*sinNegAngle)), ((dirX*sinNegAngle) + (dirY*cosNegAngle)));
     
     DrawLine(endPos, arrowHead1, length/3);
     DrawLine(endPos, arrowHead2, length/3);
  }
}
