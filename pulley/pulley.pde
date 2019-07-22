int w = 640;
int h = 640;

int initial = 0;
float elapsedTime = 0f;
PFont f1;
PFont f2;
PImage pully;

ArrayList<double[]> dataList = new ArrayList<double[]>();

void setup() {
  size(640, 640, P3D);
  background(255);
  selectInput("Select the data file to be used: ", "fileSelected");

  pully = loadImage(dataPath("pully.png"));
  pully.resize(60, 60);

  surface.setResizable(true);
  
  f1 = createFont("Arial",16,true); 
  f2 = createFont("Arial",12,true); 
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
  scale(w/640f);
  float width = 640;
  float height = 640;
  while (initial < dataList.size() && dataList.get(initial)[0] < elapsedTime) {
    initial ++;
  }

  if (initial >= dataList.size()) {
    initial = 0;
    elapsedTime = 0;
    return;
  }
  elapsedTime += 1/frameRate;
  background(255);
  smooth(8);
  strokeWeight(2);
  translate(width/2, height/2);
  pushMatrix();
  translate(0, -180);
  strokeWeight(5);
  line(-60, -45, 60, -45);
  strokeWeight(2);
  pushMatrix();
  rotate(-(float)dataList.get(initial)[1]*10/30);
  translate(-30, -30);
  image(pully, 0, 0);
  popMatrix();
  line(0, -45, 0, 45);
  fill(0);
  ellipse(0, 45, 5, 5);
  ellipse(0, 0, 5, 5);
  noFill();
  line(-30, 0, -30, 100 + (float)dataList.get(initial)[1]*10);  // left line
  line(-35, 100, -25, 100); // left horizontal line
  line(-35, 100 + (float)dataList.get(initial)[1]*10, -25, 100 + (float)dataList.get(initial)[1]*10);
  
  // draw the left real-time arrow
  drawArrow(-45, 100, -45, 100 + (float)dataList.get(initial)[1]*10, 2, 2, false);
  // add label
  textFont(f1,16);                  
  fill(0);                        
  text("2x", -65, 100 + (float)dataList.get(initial)[1]*10/2);
  popMatrix();

  pushMatrix();
  translate(15, 60);
  line(-30, 130, 30, 130);  // bottom line
  
  // draw the left real-time arrow
  drawArrow(0, 130, 0, -(float)dataList.get(initial)[2]*10 + 131, 2, 2, false);
    
  // add label
  textFont(f1,17);                  
  fill(0);                        
  text("x", 5, -(float)dataList.get(initial)[2]*10/2 + 132);
 
  translate(0, - (float)dataList.get(initial)[2]*10);
  line(-30, 0, -15, -195 + (float)dataList.get(initial)[2]*10); // middle line
  line(30, 0, 15, -240 + (float)dataList.get(initial)[2]*10);   // right line

  pushMatrix();
  rotate(-(float)dataList.get(initial)[1]*10/30);
  translate(-30, -30);
  image(pully, 0, 0);
  popMatrix();
  fill(0);
  ellipse(0, 0, 5, 5);
  noFill();
  line(0, 0, 0, 90);
  rect(-20, 90, 40, 40);
  for (int i = 1; i < 4; i ++) {
    line(-20, 90 + i*10, 20, 90 + i*10);
  }
  popMatrix(); 
}




void keyPressed() {
  if (key == 'l') {
    selectInput("Selected the data file to be used: ", "fileSelected");
  }

  if (key == '+') {
    w += 40;
    h += 40;
    surface.setSize(w,h);
  }

  if (key == '-') {
    w -= 40;
    h -= 40;
    surface.setSize(w,h);
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
      lineData = new double[] {Double.parseDouble(lineStr[0]), Double.parseDouble(lineStr[1]), Double.parseDouble(lineStr[2])};
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
