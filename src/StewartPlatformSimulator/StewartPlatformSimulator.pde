import java.io.*; //<>//
import java.net.*;
import java.util.Arrays;

float platRadius = 636.1;
float baseRadius = 546.1;
float bracketAngle = 10.613;
float defaultHeight = 850.9;

PShape Base, Platform;
PVector baseRotation, platRotation;
PVector baseTranslation, platTranslation;
PVector[][] actuatorPoints = new PVector[2][6];
PVector platformHeight = new PVector(0, 0, defaultHeight);

float cameraSpeed;
boolean useOrtho = false;
PVector cameraTranslation, cameraRotation;

double i = 0;
int waypointNum = 0;
DatagramSocket socket;
DatagramPacket packet;
byte[] UDPBuff = new byte[50];

Table pathPlan;

void settings() {
  size(1000, 1000, P3D);
  fullScreen(3);
}

void setup() {
  pathPlan = loadTable("Vibration_Trial.csv", "header");
  println(pathPlan.getRowCount() + " Waypoints in path plan");
  
  hint(ENABLE_DEPTH_SORT);
  setupUDP();
  
  surface.setResizable(true);
  
  cameraSpeed = TWO_PI / width;
  cameraRotation = new PVector(0, 1, 0);
  cameraTranslation = new PVector(width >> 1, height >> 1, 0.30);
  
  baseRotation = new PVector(0, 0, 0);
  baseTranslation = new PVector(0, 0, 0);
  
  platRotation = new PVector(0, 0, -60);
  platTranslation = new PVector(0, 0, defaultHeight + 230);
  
  lights();
  noFill();
}

void draw() {

  noFill();
  background(125);
  drawLegend();
  
  if(useOrtho) {
    ortho(-width >> 1, width >> 1, -height >> 1, height >> 1, 10, width * 2);
  }
  else {
    perspective(PI / 3.0, float(width) / float(height), 10, width * 2);
  }
  
  translate(cameraTranslation.x, cameraTranslation.y);
  scale(cameraTranslation.z);
  rotateX(cameraRotation.y);
  rotateY(cameraRotation.x);
  rotateZ(cameraRotation.z);
  thread("getUDPData");
  
  drawAxis();
  drawGrid(80, 0, -5, 0);
  
  stroke(255, 0, 0);
  drawBoundingArc(0, 0, 0, baseRadius * 2);
  
  stroke(0);
  drawBoundingArc(0, 0, 0, platRadius * 2);
  
  fill(0, 153, 0);
  drawPlatform(Base, 0, baseRadius, baseTranslation, baseRotation);
  
  fill(153, 0, 0);
  drawPlatform(Platform, 1, (int)baseRadius, platTranslation, platRotation);
  drawActuators();
  
  //if(waypointNum < pathPlan.getRowCount()) {
  //  TableRow row = pathPlan.getRow(waypointNum);
    
  //  float x = row.getFloat("X");
  //  float y = row.getFloat("Y");
  //  float z = row.getFloat("Z");
  //  float u = row.getFloat("U");
  //  float v = row.getFloat("V");
  //  float w = row.getFloat("W");
  //  int moveTime = row.getInt("MoveTime(ms)");
    
  //  platTranslation.x = x;
  //  platTranslation.y = y;
  //  platTranslation.z = defaultHeight + (z);
    
  //  platRotation.x = u;
  //  platRotation.y = v;
  //  platRotation.z = -60 + w;
    
  //  println("Moving to waypoint at row " + waypointNum + ": [" + x + ", " + y + ", " + z + ", " + u + ", " + v + ", " + w + "] " + "in " + moveTime + " Ms");
  //  waypointNum++;
  //  delay(moveTime);
  //}
  
  if(i < 360 * 4) {
    i += 1;
    //platTranslation.z = defaultHeight + abs((int)(sin((float)i * 3.14159 / 180.0) * 200.0));
    //platRotation.z = (-60) + (int)(cos((float)i * 3.14159 / 180.0) * 20.0);
    //platTranslation.y = (int)(sin((float)i * 3.14159 / 180.0) * 200.0);
    //platTranslation.x = (int)(cos((float)i * 3.14159 / 180.0) * 200.0);
    platRotation.y = (int)(sin((float)i * 3.14159 / 180.0) * 20.0);
    platRotation.x = (int)(cos((float)i * 3.14159 / 180.0) * 20.0);
  }
  
  //printActuatorLengths();
  PVector check1 = traceLine(defaultHeight, actuatorPoints[0][0], actuatorPoints[1][1]);
  PVector check2 = traceLine(defaultHeight, actuatorPoints[0][1], actuatorPoints[1][2]);
  PVector check3 = traceLine(defaultHeight, actuatorPoints[0][2], actuatorPoints[1][3]);
  PVector check4 = traceLine(defaultHeight, actuatorPoints[0][3], actuatorPoints[1][4]);
  PVector check5 = traceLine(defaultHeight, actuatorPoints[0][4], actuatorPoints[1][5]);
  PVector check6 = traceLine(defaultHeight, actuatorPoints[0][5], actuatorPoints[1][0]);
}

void keyPressed() {
  if(key == 'p') {
    useOrtho = false;
  }
  else {
    if(key == 'o') {
      useOrtho = true;
    }
  }
}

void mouseDragged() {
  if(mouseButton == LEFT) {
    cameraRotation.x += (mouseX - pmouseX) * cameraSpeed;
    cameraRotation.y += (pmouseY - mouseY) * cameraSpeed;
    cameraRotation.y = constrain(cameraRotation.y, -HALF_PI, radians(90));
  }
  else{
    if(mouseButton == RIGHT) {
      cameraRotation.z -= (mouseX - pmouseX) * cameraSpeed;
    }
  }
}

void mouseWheel(MouseEvent e) {
  cameraTranslation.x -= mouseX;
  cameraTranslation.y -= mouseY;
  float delta = e.getCount() > 0 ? 1.05 : e.getCount() < 0 ? 1.0/1.05 : 1.0;
  cameraTranslation.z *= delta;
  cameraTranslation.x *= delta;
  cameraTranslation.y *= delta;
  cameraTranslation.x += mouseX;
  cameraTranslation.y += mouseY;
}

void drawAxis() {
  //X  - red
  stroke(192,0,0);
  line(0,0,0,200,0,0);
  //Y - green
  stroke(0,192,0);
  line(0,0,0,0,200,0);
  //Z - blue
  stroke(0,0,192);
  line(0,0,0,0,0,200);
}

void drawLegend() { 
  fill(0);
  textSize(25);
  textAlign(CENTER);
  text("Stewart Platform Simulator", width >> 1, 30);
  textAlign(LEFT);
  stroke(0);
  strokeWeight(10);
  point(30, 30);
  textSize(15);
  text("Scroll mouse wheel to zoom in and out", 40, 35);
  point(30, 50);
  textSize(15);
  text("Hold right mouse button and drag to rotate about Z", 40, 55);
  point(30, 70);
  textSize(15);
  text("Hold left mouse button and drag to rotate about X and Y", 40, 75);
  point(30, 90);
  textSize(15);
  text("Click 'p' for Perspective and 'o' for orthographic view", 40, 95);
  stroke(192, 0, 0);
  point(30, 115);
  text("X-Axis", 40, 120);
  stroke(0, 192, 0);
  point(30, 135);
  text("Y-Axis", 40, 140);
  stroke(0, 0, 192);
  point(30, 155);
  text("Z-Axis", 40, 160);
  noFill();
}

void drawPlatform(PShape name, int index, float radius, PVector translation, PVector rotation) {
  pushMatrix();
  strokeWeight(3);
  popMatrix();
  
  name = createShape();
  name.beginShape();
  
  name.vertex(translation.x + (cos(radians((-90 + rotation.z) - bracketAngle/2)) * radius), translation.y + (sin(radians((-90 + rotation.z) - bracketAngle/2)) * radius), 0);
  name.vertex(translation.x + (cos(radians((-90 + rotation.z) + bracketAngle/2)) * radius), translation.y + (sin(radians((-90 + rotation.z) + bracketAngle/2)) * radius), 0);
  name.vertex(translation.x + (cos(radians((30 + rotation.z) - bracketAngle/2)) * radius), translation.y + (sin(radians((30 + rotation.z) - bracketAngle/2)) * radius), 0);
  name.vertex(translation.x + (cos(radians((30 + rotation.z) + bracketAngle/2)) * radius), translation.y + (sin(radians((30 + rotation.z) + bracketAngle/2)) * radius), 0);
  name.vertex(translation.x + (cos(radians((150 + rotation.z) - bracketAngle/2)) * radius), translation.y + (sin(radians((150 + rotation.z) - bracketAngle/2)) * radius), 0);
  name.vertex(translation.x + (cos(radians((150 + rotation.z) + bracketAngle/2)) * radius), translation.y + (sin(radians((150 + rotation.z) + bracketAngle/2)) * radius), 0);

  name.endShape( CLOSE );
  
  Roll(name, radians(rotation.y));
  Pitch(name, radians(rotation.x));
  translateZ(name, translation.z);
  
  for(int i = 0; i < name.getVertexCount(); i++) {
    actuatorPoints[index][i] = name.getVertex(i);
  }
  platformHeight.z = translation.z;
  shape(name);
}

void Roll(PShape name, float radians) {
  PVector vertex;
  for(int i = 0; i < name.getVertexCount(); i++) {
    vertex = name.getVertex(i);
    float x = vertex.x;
    float z = vertex.z;
    vertex.x = x * cos(radians) + z * sin(radians);
    vertex.z = z * cos(radians) - x * sin(radians);
    name.setVertex(i, vertex);
  }
}

void Pitch(PShape name, float radians) {
  PVector vertex;
  for(int i = 0; i < name.getVertexCount(); i++) {
    vertex = name.getVertex(i);
    float y = vertex.y;
    float z = vertex.z;
    vertex.y = y * cos(radians) - z * sin(radians);
    vertex.z = z * cos(radians) + y * sin(radians);
    name.setVertex(i, vertex);
  }
}

void translateZ(PShape name, float z) {
  PVector vertex;
  for(int i = 0; i < name.getVertexCount(); i++) {
    vertex = name.getVertex(i);
    vertex.z = vertex.z + z;
    name.setVertex(i, vertex);
  }
}

void drawActuators() {
  stroke(255, 150, 0);
  strokeWeight(30);
  for(int i = 0; i < 2; i++) {
    for(int j = 0; j < 6; j++) {
      PVector point = actuatorPoints[i][j];
      point(point.x, point.y, point.z);
    }
    point(0, 0, 0);
    point(platformHeight.x, platformHeight.y, platformHeight.z);
  }
  stroke(50);
  strokeWeight(15);
  for(int i = 0; i < 6; i++) {
    PVector basePoint = actuatorPoints[0][i];
    PVector platPoint = actuatorPoints[1][(i + 1) % 6];
    line(basePoint.x, basePoint.y, basePoint.z, platPoint.x, platPoint.y, platPoint.z);
  }
}

void drawGrid(int count, int x, int y, int z) {
  pushMatrix();
  stroke(100);
  strokeWeight(3);
  rotateX(radians(90));
  translate(x, y, z);
  float size = (count -1) * 20*2;
  for (int i = 0; i < count; i++) {
    float pos = map(i, 0, count-1, -0.5 * size, 0.5 * size);
    line(pos, 0, -size/2, pos, 0, size/2);
    line(-size/2, 0, pos, size/2, 0, pos);
  }
  popMatrix();
}

void drawBoundingArc(float x, float y, float z, float diameter) {
  pushMatrix();
  translate(x, y, z);
  circle(x, y, diameter);
  popMatrix();
}

void printActuatorLengths() {
  for(int i = 0; i < 6; i++) {
    println("A" + i + ": " + (dist(actuatorPoints[0][i].x, actuatorPoints[0][i].y, actuatorPoints[0][i].z, actuatorPoints[1][i].x, actuatorPoints[1][i].y, actuatorPoints[1][i].z)));
  }
  println("Platform Height: " + dist(0, 0, 0, platformHeight.x, platformHeight.y, platformHeight.z));
  println(" ");
}

PVector traceLine(float plane, PVector basePoint, PVector platPoint) {
  PVector directionVector = PVector.sub(platPoint, basePoint);
  PVector hitMark = new PVector(0, 0, plane);
  float t = (plane - basePoint.z) / (directionVector.z);
  hitMark.x = basePoint.x + t*directionVector.x;
  hitMark.y = basePoint.y + t*directionVector.y;
  
  rectMode(CENTER);
  stroke(0);
  strokeWeight(3);
  fill(0, 0, 128, 40);
  pushMatrix();
  translate(0, 0, plane);
  rect(0, 0, 1500, 1500);
  popMatrix();
  fill(255, 0, 0, 128);
  stroke(255);
  strokeWeight(15);
  line(basePoint.x, basePoint.y, basePoint.z, hitMark.x, hitMark.y, hitMark.z);
  strokeWeight(30);
  stroke(188, 234, 167);
  point(hitMark.x, hitMark.y, plane);
  
  return hitMark;
}

void setupUDP() {
  try {
    socket = new DatagramSocket(7408);
  }
  catch(Exception e) {
    e.printStackTrace();
    println(e.getMessage());
  }
}

void getUDPData() {
  try {
    DatagramPacket packet = new DatagramPacket(UDPBuff, UDPBuff.length);
    socket.receive(packet);
    InetAddress address = packet.getAddress();
    int port = packet.getPort();
    packet = new DatagramPacket(UDPBuff, UDPBuff.length, address, port);

    int positionData[] = new int[6];
    int j = 0;
    for(int i = 20; i+4 <= 44; i+=4) {
      positionData[j] = bytesToInt(UDPBuff[i], UDPBuff[i+1], UDPBuff[i+2], UDPBuff[i+3]);
      println(pulsesToMM(positionData[j]));
      j++;
    }
    println(" ");
    
    //X = pulsesToMM(positionData[0]);
    //Y = pulsesToMM(positionData[1]);
    //Z = (int)defaultHeight + pulsesToMM(positionData[2]);
    //U = pulsesToMM(positionData[3]);
    //V = pulsesToMM(positionData[4]);
    //W = pulsesToMM(positionData[5]);
    //println(Arrays.toString(buf));

    //String received = new String(packet.getData(), 0, packet.getLength());
    //println(received);
  }
  catch(IOException e) {
    e.printStackTrace(); 
    println(e.getMessage());
  }
}

int bytesToInt(byte a, byte b, byte c, byte d) {
  int word1 = (a<<8) + (b);
  int word2 = ((c<<8) + (d)) & 0xFFFF;
  int concat = (word1<<16) + word2;
  return concat;
}

int pulsesToMM(int pulses) {
  int MM = (5 * pulses) / (1 * 7500);
  return MM;
}
