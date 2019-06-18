enum DebugColors
{
  RED, 
  GREEN,
  BLUE,
  YELLOW,
  WHITE,
  PINK,
  BLACK,
}

class DebugDisplay
{ 
  void DrawLine(float startPosX, float startPosY, float endPosX, float endPosY, DebugColors colorVal)
  {
   pushMatrix();
   SetStrokeColor(colorVal);
   line (startPosX, startPosY, endPosX, endPosY);
   popMatrix();
  }
  
  void DrawLine(Vec2 pos, Vec2 dir, float length, DebugColors colorVal)
  {
     dir.normalize();
     dir.mulLocal(length);
     
     Vec2 endPos = dir.add(pos);
     
     DrawLine(pos.x, pos.y, endPos.x, endPos.y, colorVal);
  }
  
  void DrawLine(Vec2 startPos, Vec2 endPos, DebugColors colorVal)
  {
     DrawLine(startPos.x, startPos.y, endPos.x, endPos.y, colorVal);
  }
  
  void DrawArrow(float posX, float posY, float dirX, float dirY, float length, DebugColors colorVal)
  {
     DrawArrow(new Vec2(posX, posY), new Vec2(dirX, dirY), length, colorVal); 
  }
  
  void DrawArrow(Vec2 pos, Vec2 dir, float length, DebugColors colorVal)
  {
     dir.normalize();
     Vec2 endPos = pos.add(dir.mul(length)); 
     DrawLine(pos, endPos, colorVal);
     
     Vec2 revDir = dir.mul(-1);
     float rotAngle = 45;
     float rotAngleRadians = (float)Math.toRadians(rotAngle);
     float cosAngle = (float)Math.cos(rotAngleRadians);
     float sinAngle = (float)Math.sin(rotAngleRadians);
     float cosNegAngle = (float)Math.cos(-rotAngleRadians);
     float sinNegAngle = (float)Math.sin(-rotAngleRadians);
     
     float dirX = revDir.x;
     float dirY = revDir.y;
     
     Vec2 arrowHead1 = new Vec2(((dirX*cosAngle) - (dirY*sinAngle)), ((dirX*sinAngle) + (dirY*cosAngle)));
     Vec2 arrowHead2 = new Vec2(((dirX*cosNegAngle) - (dirY*sinNegAngle)), ((dirX*sinNegAngle) + (dirY*cosNegAngle)));
     
     DrawLine(endPos, arrowHead1, length/3, colorVal); //<>//
     DrawLine(endPos, arrowHead2, length/3, colorVal);
  }
  
  void PrintString(Vec2 pos, String text, int textScale, DebugColors colorVal)
  {
     PrintString(pos.x, pos.y, text, textScale, colorVal);
  }
  
  void PrintString(float posX, float posY, String text, int textScale, DebugColors colorVal)
  {
     pushMatrix();
     SetFillColor(colorVal);
     textSize(textScale*15);
     text(text, posX, posY); 
     popMatrix();
  }
  
  void SetStrokeColor(DebugColors colorVal)
  {
      float[] rgbColors = GetRGBValues(colorVal);
      stroke(rgbColors[0], rgbColors[1], rgbColors[2]);
  }
  
  void SetFillColor(DebugColors colorVal)
  {
      float[] rgbColors = GetRGBValues(colorVal);
      fill(rgbColors[0], rgbColors[1], rgbColors[2]);
  }
  
  float[] GetRGBValues(DebugColors colorVal)
  {
    float[] rgbColors = new float[3];
    
    switch(colorVal)
    {
       case RED:
       rgbColors[0] = 255;
       rgbColors[1] = 0;
       rgbColors[2] = 0;
       break;
       
       case GREEN:
       rgbColors[0] = 0;
       rgbColors[1] = 255;
       rgbColors[2] = 0;
       break;
       
       case BLUE:
       rgbColors[0] = 0;
       rgbColors[1] = 0;
       rgbColors[2] = 255;
       break;
       
       case YELLOW:
       rgbColors[0] = 255;
       rgbColors[1] = 255;
       rgbColors[2] = 0;
       break;
       
       case PINK:
       rgbColors[0] = 255;
       rgbColors[1] = 51;
       rgbColors[2] = 255;
       break;
       
       case BLACK:
       rgbColors[0] = 0;
       rgbColors[1] = 0;
       rgbColors[2] = 0;
       break;
       
       case WHITE:
       default:
       rgbColors[0] = 255;
       rgbColors[1] = 255;
       rgbColors[2] = 255;
       break;
    }
    
    return rgbColors;
  }
}
