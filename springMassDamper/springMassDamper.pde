PImage spring;
PImage cart;
PImage boarder;
PImage arrow;
PFont f;

int initial = 0;
float elapsedTime = 0f;
//ArrayList<double[]> dataList = new ArrayList<double[]>();

int w = 640;
int h = 640;
int x = 0;

void setup() {
  w = 640;
  h = 640;
  size(600, 640, P3D);
  f = createFont("Georgia", 27, true); 
  background(255);
  //selectInput("Select the data file to be used: ", "fileSelected");
  spring = loadImage(dataPath("largeSpring.png"));
  cart = loadImage(dataPath("cart.png"));
  boarder = loadImage(dataPath("boarder.png"));
  arrow = loadImage(dataPath("force.png"));
  surface.setResizable(true);

  arrow.resize(60, 30);
  spring.resize(70, 30);
  cart.resize(150, 100);
  boarder.resize(30, 150);
}


void draw() {
  scale(w/640f);
  float width = 640;
  float height = 640;
  x += 1;
  
  if(x > 67){
   x = 0; 
  }
 
 
  elapsedTime += 1/20f;
  background(255);

  image(cart, x + 40 + spring.width + 25, height/2 - cart.height/2);
  textFont(f, 27);                  
  fill(0);                        
  text("m", x + 40 + spring.width + 15 + cart.width/2, height/2 - cart.height/2 + 45);
  

  noStroke();
  // Draw the spring
  beginShape();
  texture(spring);
  vertex(40, height/2 - cart.height/2 + cart.height/6 + spring.height/2 + 15, 0, 0);
  vertex(spring.width + 65 + x, height/2 - cart.height/2 + cart.height/6 + spring.height/2 + 15, 70, 0);
  vertex(spring.width + 65 + x, height/2 - cart.height/2 + cart.height/6 - spring.height/2 + 15, 70, 40);
  vertex(40, height/2 - cart.height/2 + cart.height/6 - spring.height/2 + 15, 0, 40);
  endShape();
  
 

  // Draw the boarder
  beginShape();
  texture(boarder);
  vertex(10, height/2 + cart.height/2, 0, 0);
  vertex(10 + boarder.width, height/2 + cart.height/2, 30, 0);
  vertex(10 + boarder.width, height/2 + cart.height/2 - boarder.height, 30, 150);
  vertex(10, height/2 + cart.height/2 - boarder.height, 0, 150);
  endShape();
  
  // Draw the force
  if(x >= 2 && x <= 30){
    image(arrow, 20 + boarder.width + 23 + x, height/2 - cart.height/2 - 10);
  }
  
  stroke(0);
  strokeWeight(2);
  // Draw ground
  line(0, height/2 + cart.height/2, width, height/2 + cart.height/2);
  
  // Draw absorber
  line(x + 80, height/2 + cart.height/6, x + 80, height/2 + cart.height/6);
  line(x + 80, height/2 + cart.height/6 + 5, x + 80, height/2 + cart.height/6 - 5);
 
  line(40, height/2 + cart.height/6, 20 + 40 + 15, height/2 + cart.height/6);
 
  line(20 + 40 + 15, height/2 + cart.height/6 + 7.5, 20 + 40 + 15, height/2 + cart.height/6 - 7.5);
  
  line(20 + 40 + 15, height/2 + cart.height/6 + 7.5, 20 + 30 + 90, height/2 + cart.height/6 + 7.5);
  line(20 + 40 + 15, height/2 + cart.height/6 - 7.5, 20 + 30 + 90, height/2 + cart.height/6 - 7.5);

  line(x + 80, height/2 - cart.height/2 + cart.height - 34, x + 135, height/2 - cart.height/2 + cart.height - 34);

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
