PImage spring;
PImage arrow;

int initial = 0;
float elapsedTime = 0f;
ArrayList<double[]> dataList = new ArrayList<double[]>();
int w = 640;
int h = 640;
PFont f1;
PFont f2;
PFont f3;



void setup() {
  size(640, 640, P3D);
  background(255);
  selectInput("Select the data file to be used: ", "fileSelected");
  spring = loadImage(dataPath("smallSpring.png")); 
  arrow = loadImage(dataPath("force_vert.png"));
  arrow.resize(20, 40);
  spring.resize(30, 60);
  f1 = loadFont("TimesNewRomanPS-ItalicMT-30.vlw");
  textFont(f1);
  
  f2 = loadFont("TimesNewRomanPS-ItalicMT-9.vlw");
  textFont(f2);
  
  f3 = loadFont("TimesNewRomanPS-ItalicMT-15.vlw");
  textFont(f3);
}

void fileSelected(File selection) {
  if (selection == null) {
    println("No file selected");
  }
  else {
    println("User selected: " + selection.getAbsolutePath());
    loadFile(selection.getAbsolutePath());
  }
}


void draw() {
  //float min = (float)(dataList.get(0)[0]);
  //float max = (float)(dataList.get(dataList.size())[0]);
  scale(w/640f);
  float width = 640f;
  float height = 640f;
  while (initial < dataList.size() && dataList.get(initial)[0] < elapsedTime) {
    initial ++;
  }

  if (initial >= dataList.size()) {
    initial = 0;
    elapsedTime = 0;
    return;
  }
  //elapsedTime += 1/frameRate;
  elapsedTime += 1/20f;
  background(255);
  
  stroke(0);
  strokeWeight(2);
  noFill();
  rect(width/2 - 50, height/2 - 30 - (float)(dataList.get(initial)[1]*2000), 100, 60);
  
  strokeWeight(1);
  line(width/2 - 65, height/2 - 30 - 1, width/2 - 55, height/2 - 30 - 1);
  line(width/2 - 65, height/2 - 30 + 18, width/2 - 55, height/2 - 30 + 18);
  //line(width/2 - 65, height/2 - 70 - (float)(dataList.get(initial)[1]*2000), width/2 - 65, height/2 - 70 - (float)(dataList.get(initial)[1]*2000));
  
  textFont(f1, 30);                  
  fill(0);                        
  text("m", width/2 - 8, height/2 - 30 - (float)(dataList.get(initial)[1]*2000) + 35);
  
 
  //draw distance
  strokeWeight(1);
  drawArrow(width/2 - 60, height/2 - 31, width/2 - 60, height/2 - 12, 3, 3, true);

  textFont(f3, 15);                  
  fill(0);                        
  text("x", width/2 - 78, height/2 - 19);
  
  textFont(f2,9);                  
  fill(0);                        
  text("0", width/2 - 71, height/2 - 19);
  
  
  
  noStroke();
  // Draw spring
  beginShape();
  texture(spring);
  vertex(width/2 - 100/6 - 15, height/2 + 30 + 80, 0, 0); // bottom left
  vertex(width/2 - 100/6 + 15, height/2 + 30 + 80, 30, 0); // bottom right
  vertex(width/2 - 100/6 + 15, height/2 + 30 - (float)(dataList.get(initial)[1]*2000), 30, 60); // top right
  vertex(width/2 - 100/6 - 15, height/2 + 30 - (float)(dataList.get(initial)[1]*2000), 0, 60);  // top left
  endShape();
  

  // Draw force
  fill(255);
  if(elapsedTime > 2f){
    image(arrow, width/2 - 10, height/2 - 70 - (float)(dataList.get(initial)[1]*2000));
  }
 
 // beginShape();
 // texture(arrow);
 // vertex(width/2 - 10, height/2 - 30 - (float)(dataList.get(initial)[1]*500), 0, 0);
 // vertex(width/2 + 10, height/2 - 30 - (float)(dataList.get(initial)[1]*500), 20, 0);
 // vertex(width/2 + 10, height/2 - 60 - (float)(dataList.get(initial)[1]*500), 20, 40);
 // vertex(width/2 - 10, height/2 - 60 - (float)(dataList.get(initial)[1]*500), 0, 40);
 // endShape();
  
  stroke(0);
  strokeWeight(2);
  line(width/2 + 100/6, height/2 + 30 + 80, width/2 + 100/6, height/2 + 30 + 80 - 20);
  line(width/2 + 100/6 - 7.5, height/2 + 30 + 80 - 20, width/2 + 100/6 + 7.5, height/2 + 30 + 80 - 20);
  line(width/2 + 100/6 - 7.5, height/2 + 30 + 80 - 20, width/2 + 100/6 - 7.5, height/2 + 30 + 80 - 20 - 35);
  line(width/2 + 100/6 + 7.5, height/2 + 30 + 80 - 20, width/2 + 100/6 + 7.5, height/2 + 30 + 80 - 20 - 35);
  
  line(width/2 + 100/6, height/2 + 30 - (float)(dataList.get(initial)[1]*2000), width/2 + 100/6, height/2 + 30 - (float)(dataList.get(initial)[1]*2000) + 30);
  line(width/2 + 100/6 - 5, height/2 + 30 - (float)(dataList.get(initial)[1]*2000) + 30, width/2 + 100/6 + 5, height/2 + 30 - (float)(dataList.get(initial)[1]*2000) + 30);

  // Draw ground
  stroke(0);
  strokeWeight(2);
  line(0, height/2 + 30 + 80, width, height/2 + 30 + 80);
  
}

void keyPressed() {
  if (key == 'l') {
    selectInput("Selected the data file to be used: ", "fileSelected");
  }

  if (key == 'r') {
    initial = 0;
    elapsedTime = 0f;
  }
  if (key == '+') {
    w += 40;
    h += 40;
    surface.setSize(w, h);
  }
  if (key == '-') {
    w -= 40;
    h -= 40;
    surface.setSize(w, h);
  }
}

void loadFile(String path) {
  dataList.clear();
  initial = 0;
  elapsedTime = 0;
  try {
    BufferedReader br = createReader(path);
    String line;
    while ((line = br.readLine()) != null) {
      double[] lineData;
      String[] lineStr = line.split(",");
      lineData = new double[] {Double.parseDouble(lineStr[0]), Double.parseDouble(lineStr[1]), Double.parseDouble(lineStr[2]), Double.parseDouble(lineStr[3])};
      dataList.add(lineData);
    }
  }
  catch (IOException e) {
    e.printStackTrace();
  }
}



 void drawArrow(float x0, float y0, float x1, float y1, float beginHeadSize, float endHeadSize, boolean filled) {

  PVector d = new PVector(x1 - x0, y1 - y0);
  d.normalize();
  
  float coeff = 1.5;
  
  strokeCap(SQUARE);
  
  line(x0+d.x*beginHeadSize*coeff/(filled?1.0f:1.75f), 
        y0+d.y*beginHeadSize*coeff/(filled?1.0f:1.75f), 
        x1-d.x*endHeadSize*coeff/(filled?1.0f:1.75f), 
        y1-d.y*endHeadSize*coeff/(filled?1.0f:1.75f));
  
  float angle = atan2(d.y, d.x);
  
  if (filled) {
    // begin head
    pushMatrix();
    translate(x0, y0);
    rotate(angle+PI);
    triangle(-beginHeadSize*coeff, -beginHeadSize, 
             -beginHeadSize*coeff, beginHeadSize, 
             0, 0);
    popMatrix();
    // end head
    pushMatrix();
    translate(x1, y1);
    rotate(angle);
    triangle(-endHeadSize*coeff, -endHeadSize, 
             -endHeadSize*coeff, endHeadSize, 
             0, 0);
    popMatrix();
  } 
  else {
    // begin head
    pushMatrix();
    translate(x0, y0);
    rotate(angle+PI);
    strokeCap(ROUND);
    line(-beginHeadSize*coeff, -beginHeadSize, 0, 0);
    line(-beginHeadSize*coeff, beginHeadSize, 0, 0);
    popMatrix();
    // end head
    pushMatrix();
    translate(x1, y1);
    rotate(angle);
    strokeCap(ROUND);
    line(-endHeadSize*coeff, -endHeadSize, 0, 0);
    line(-endHeadSize*coeff, endHeadSize, 0, 0);
    popMatrix();
  }
}
