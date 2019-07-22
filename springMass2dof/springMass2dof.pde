// initial task:
// 1. make the spring thinner: in line 26, change the second parameter to be smaller(it represents for the height of spring) to make it become a thinner spring
// 2. label the masses: I've added the text line.
// 3. change the length of the baseline: in line 109, change the third parameter to change the length of baseline





int initial = 0;
float elapsedTime = 0f;
ArrayList<double[]> dataList = new ArrayList<double[]>();

PImage spring;
PImage cart;
PImage boarder;
PImage rboarder;
int w = 640;
int h = 640;
PFont f;   
PFont g; 

void setup() {
  size(800, 500, P3D);
  background(255);
  selectInput("Select the data file to be used: ", "fileSelected");
  spring = loadImage("largeSpring.png");
  spring.resize(100, 30); // change the second parameter smaller to make it become a thinner spring
  cart = loadImage(dataPath("cart.png"));
  cart.resize(150, 100);
  boarder = loadImage(dataPath("boarder.png"));
  boarder.resize(30, 150);
  rboarder = loadImage(dataPath("rboarder.png"));
  rboarder.resize(30, 150);
  f = createFont("Arial",16,true); // STEP 2 Create Font
  g = createFont("Arial",12,true); // STEP 2 Create Font
}

void fileSelected(File selection) {
  if (selection == null) {
    println("No file selected");
  }
  else {
    println("User selected : " + selection.getAbsolutePath());
    loadFile(selection.getAbsolutePath());
  }
}

void draw() {
  scale(w / 640f);
  float width = 640f;
  float height = 640f;
  while (initial < dataList.size() && dataList.get(initial)[0] < elapsedTime) {
    initial++;
  }

  if (initial >= dataList.size()) {
    initial = 0;
    elapsedTime = 0;
    return;
  }
  elapsedTime += 1/frameRate;
  background(255);

  fill(255);
  image(cart, width/2 - spring.width/2 - cart.width + (float) dataList.get(initial)[1]*10, height/2 - cart.height/2 + 10);
  textFont(f,20);                  // STEP 3 Specify font to be used
  fill(0);                         // STEP 4 Specify font color 
  text("m", width/2 - spring.width/2 - cart.width + (float) dataList.get(initial)[1]*10 + 62, height/2 - cart.height/2 + 50);
  
  textFont(g,10);                  // STEP 3 Specify font to be used
  fill(0);   
  text("1", width/2 - spring.width/2 - cart.width + (float) dataList.get(initial)[1]*10 + 80, height/2 - cart.height/2 + 50);
  
  textFont(f,20);                  // STEP 3 Specify font to be used
  fill(0);   
  image(cart, width/2 + spring.width/2 + (float) dataList.get(initial)[2]*10, height/2 - cart.height/2 + 10);
  text("m", width/2 + spring.width/2 + (float) dataList.get(initial)[2]*10 + 62, height/2 - cart.height/2 + 50);
  
  textFont(g,10);                  // STEP 3 Specify font to be used
  fill(0);   
  text("2", width/2 + spring.width/2 + (float) dataList.get(initial)[2]*10 + 80, height/2 - cart.height/2 + 50);
  
  image(boarder, width/2 - spring.width/2 - cart.width - spring.width - boarder.width, height/2 - 150 + cart.height/2 + 10);
  image(rboarder, width/2 + spring.width/2 + cart.width + spring.width, height/2 - 150 + cart.height/2 + 10);
  
  noStroke();
  
  //right spring
  beginShape();
  texture(spring);
  vertex(width/2 + spring.width/2 + cart.width + (float) dataList.get(initial)[2]*10, height/2 - spring.height/2, 0, 0);
  vertex(width/2 + spring.width/2 + cart.width + spring.width, height/2 - spring.height/2, 100, 0);
  vertex(width/2 + spring.width/2 + cart.width + spring.width, height/2 + spring.height/2, 100, 40);
  vertex(width/2 + spring.width/2 + cart.width + (float) dataList.get(initial)[2]*10, height/2 + spring.height/2, 0, 40);
  endShape();
  
  // left spring
  beginShape();
  texture(spring);
  vertex(width/2 - spring.width/2 - cart.width - spring.width, height/2 - spring.height/2, 0, 0);
  vertex(width/2 - spring.width/2 - cart.width + (float) dataList.get(initial)[1]*10, height/2 - spring.height/2, 100, 0);
  vertex(width/2 - spring.width/2 - cart.width + (float) dataList.get(initial)[1]*10, height/2 + spring.height/2, 100, 40);
  vertex(width/2 - spring.width/2 - cart.width - spring.width, height/2 + spring.height/2, 0, 40);
  endShape();
  
  //middle spring
  beginShape();
  texture(spring);
  vertex(width/2 - spring.width/2 + (float) dataList.get(initial)[1]*10, height/2 - spring.height/2, 0, 0);
  vertex(width/2 + spring.width/2 + (float) dataList.get(initial)[2]*10, height/2 - spring.height/2, 100, 0);
  vertex(width/2 + spring.width/2 + (float) dataList.get(initial)[2]*10, height/2 + spring.height/2, 100, 40);
  vertex(width/2 - spring.width/2 + (float) dataList.get(initial)[1]*10, height/2 + spring.height/2, 0, 40);
  endShape();
 

  stroke(0);
  strokeWeight(2);
  line(0, height/2 + cart.height/2 + 10, 640, height/2 + cart.height/2 + 10);; 
  // change the third parameter to change the length of baseline
}

void keyPressed() {
  if (key == 'l') {
    selectInput("Select the data file to be used: ", "fileSelected");
  }
  if(key == 'r') {
    initial = 0;
    elapsedTime = 0;
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
      lineData = new double[] {Double.parseDouble(lineStr[0]), Double.parseDouble(lineStr[1]), Double.parseDouble(lineStr[2])};
      dataList.add(lineData);
    }
  }
  catch (IOException e) {
    e.printStackTrace();
  }
}
