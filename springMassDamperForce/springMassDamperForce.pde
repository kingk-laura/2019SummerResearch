
PImage cart;
//PImage boarder;
PImage arrow;
PFont f;

int initial = 0;
float elapsedTime = 0f;

int w = 640;
int h = 640;
float x = 0;

void setup() {
  w = 640;
  h = 640;
  f = createFont("Georgia", 27, true); 
  size(640, 640, P3D);
  background(255);
  
  cart = loadImage(dataPath("cart.png"));
  //boarder = loadImage(dataPath("boarder.png"));
  arrow = loadImage(dataPath("force.png"));
  surface.setResizable(true);
  //boarder.resize(600, 60);
  arrow.resize(55, 26);
  cart.resize(240, 100);

}


void draw() {
  scale(w/640f);
  float width = 640;
  float height = 640;
  x += 2;
  elapsedTime += 1/20f;
  background(255);
  
  if(x >= 700){
   x = 0; 
  }

  noStroke();
  beginShape();
  texture(cart);
  vertex(width/7 + x - 38, height/2 , 0, 0);
  vertex(width/7 + 150 + x - 38, height/2 , cart.width, 0);
  vertex(width/7 + 150 + x - 38, height/2 + 100, cart.width, cart.height);
  vertex(width/7 + x - 38, height/2 + 100, 0, cart.height);
  endShape();
  
  textFont(f, 27);                  
  fill(0);                        
  text("m", width/7 + 28 + x, height/2 + 45);
  
  // Draw ground
  stroke(0);
  strokeWeight(2);
  line(0, height/2 + 30 + 60 + 10, width, height/2 + 30 + 60 + 10);
  
  //image(boarder, 0, height/2 + 100);

  if(x >= 2 && x <= 50){
    image(arrow, x - 5 + 3, height/2 + 17);
  }




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
