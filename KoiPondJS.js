/*
 * @name Flocking
 * @description Demonstration of <a href="http://www.red3d.com/cwr/">Craig Reynolds' "Flocking" behavior</a>.<br>
 * (Rules: Cohesion, Separation, Alignment.)<br>
 * From <a href="http://natureofcode.com">natureofcode.com</a>.
 */
let boids = [];
var scribble = new Scribble();    
scribble.roughness = 1.5;


function setup() 
{
  createCanvas(windowWidth, windowHeight/4);
 
  // Add an initial set of boids into the system
  for (let i = 0; i < 10; i++) {
    boids[i] = new Boid(random(width), random(height));
  }
}

function draw() 
{
  background(0);
  // Run all the boids
  for (let i = 0; i < boids.length; i++) {
    boids[i].run(boids);
  }
  //text("A Small Creatures Pond", width/2, height/6, width/1.5, height/4);
}

class Node {
  constructor() {
    this.x = 0;
    this.y = 0;
  }
}

class flagellum 
{
  constructor() {
    this.node = [];
     this.theta = 180;
    this.fsize  = (height/200) * random(3, 5);
    this.numNodes = random(1,3) * this.fsize;

    this.count = 0;
    
    this.rot = 0; 
    this.rotspeed  = random(0.03,1.3);
    
    this.sw=random(1,1); 
    
    this.muscleFreq = random(0.06, 0.17); 
    this.muscleRange = 16;
    this.node[0] = new Node();
    for (let n = 1; n < this.numNodes; n++) {
      this.node[n]= new Node();
    }
  }

  move()
  {
    // head node
    this.node[0].x = cos(radians(this.theta));
    this.node[0].y = sin(radians(this.theta));

    // mucular node (neck)
    this.count += this.muscleFreq;
    let thetaMuscle = this.muscleRange * sin(this.count);
    this.node[1].x = (-3*this.fsize) * cos(radians(this.theta + thetaMuscle)) + this.node[0].x;
    this.node[1].y = (-3*this.fsize) * sin(radians(this.theta + thetaMuscle)) + this.node[0].y;

    // apply kinetic force trough body nodes (spine)
    for (let n = 2; n < this.numNodes; n++) {
      let dx = this.node[n].x - this.node[n - 2].x;
      let dy = this.node[n].y - this.node[n - 2].y;
      let d = sqrt(dx * dx + dy * dy);
      this.node[n].x = this.node[n - 1].x + (dx * (3*this.fsize)) / d;
      this.node[n].y = this.node[n - 1].y + (dy * (3*this.fsize)) / d;
    }
  }

  display() {
    stroke(250,250,250,100);
//    fill(0);
    this.rot += this.rotspeed;
    strokeWeight(this.sw/5.0);
    rectMode(CENTER);
   // beginShape(QUAD_STRIP);

    let dx = this.node[1].x - this.node[0].x;
    let dy = this.node[1].y - this.node[0].y;
    let angle = -atan2(dy, dx);
      
    let x1 = this.node[0].x + sin(angle) * -(1/this.fsize);
    let y1 = this.node[0].y + cos(angle) * -(1/this.fsize);
    rotate(-angle);
    scribble.scribbleEllipse (x1, y1, 1.83 * this.fsize*(this.numNodes), 1.42 * this.fsize * (this.numNodes));
    ellipse (x1, y1, 1.8 * this.fsize * (this.numNodes), 1.4 * this.fsize * (this.numNodes));
    rotate(angle);
    
    for (let n = 1; n < this.numNodes; n++) {
        dx = this.node[n].x - this.node[n - 1].x;
        dy = this.node[n].y - this.node[n - 1].y;
        angle = -atan2(dy, dx);
      
        x1 = this.node[n].x + sin(angle) * -(1/this.fsize);
        y1 = this.node[n].y + cos(angle) * -(1/this.fsize);

        //rotate(this.rot);
        scribble.scribbleRect  (x1, y1, 0.5*this.fsize*(this.numNodes-n), 0.5*this.fsize*(this.numNodes-n));
        rect  (x1, y1, 0.5*this.fsize*(this.numNodes-n), 0.5*this.fsize*(this.numNodes-n));
        //rotate(-this.rot);
    }
   // endShape();
    }
}





// Boid class
// Methods for Separation, Cohesion, Alignment added
class Boid extends flagellum {
  constructor(x, y) {
    super();
    this.acceleration = createVector(0, 0);
    this.velocity = p5.Vector.random2D();
    this.position = createVector(x, y);
    this.r = 2*this.numNodes*this.fsize;
    this.Col = color(random(255),random(255), random(255), 20);

    this.maxspeed = random(2,3);    // Maximum speed
    this.maxforce = 0.3; // Maximum steering force
  }

  run(boids) {
    this.flock(boids);
    this.update();
    this.borders();
    //this.render();
    this.display();
  }

  // Forces go into acceleration
  applyForce(force) {
    this.acceleration.add(force);
  }

  // We accumulate a new acceleration each time based on three rules
  flock(boids) {
    let sep = this.separate(boids); // Separation
    let ali = this.align(boids);    // Alignment
    let coh = this.cohesion(boids); // Cohesion
    // Arbitrarily weight these forces
    sep.mult(0.1);
    ali.mult(1);
    coh.mult(0.01);
 
    // Add the force vectors to acceleration
    this.applyForce(sep);
    this.applyForce(ali);
    this.applyForce(coh);
  }

  // Method to update location
  update() {
    // Update velocity
    this.velocity.add(this.acceleration);
    // Limit speed
    this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
    // Reset acceleration to 0 each cycle
    this.acceleration.mult(0);
    super.muscleFreq = norm(this.velocity.mag(), 0, 1) * 0.06;
    super.move();

  }

  // A method that calculates and applies a steering force towards a target
  // STEER = DESIRED MINUS VELOCITY
  seek(target) {
    let desired = p5.Vector.sub(target, this.position); // A vector pointing from the location to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(this.maxspeed);
    // Steering = Desired minus Velocity
    let steer = p5.Vector.sub(desired, this.velocity);
    steer.limit(this.maxforce); // Limit to maximum steering force
    return steer;
  }

  // Draw boid as a circle
  render() {
    fill(127, 127);
    stroke(200);
    ellipse(this.position.x, this.position.y, 16, 16);
  }
  
  display()
  {
    let theta = this.velocity.heading() + radians(180);
    fill(this.Col);
    stroke(200);
 
    push();
    translate(this.position.x, this.position.y);
    rotate(theta);
    super.display();
    //rect(0,0,16,16);
    pop();
    //noTint();

    // update flagellum body rotation
    //super.theta = degrees(theta);
    //super.theta += 180;
  }
  
  // Wraparound
  borders() {
    if (this.position.x < -this.r) {this.position.x = width + this.r;}
    if (this.position.y < -this.r) {this.position.y = height + this.r;}
    if (this.position.x > width + this.r) {this.position.x = -this.r;}
    if (this.position.y > height + this.r) {this.position.y = -this.r;}
  }

  // Separation
  // Method checks for nearby boids and steers away
  separate(boids) {
    let desiredseparation = 70.0;
    let steer = createVector(0, 0);
    let count = 0;
    // For every boid in the system, check if it's too close
    for (let i = 0; i < boids.length; i++) {
      let d = p5.Vector.dist(this.position, boids[i].position);
      // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
      if ((d > 0) && (d < desiredseparation)) {
        // Calculate vector pointing away from neighbor
        let diff = p5.Vector.sub(this.position, boids[i].position);
        diff.normalize();
        diff.div(d); // Weight by distance
        steer.add(diff);
        count++; // Keep track of how many
      }
    }
    // Average -- divide by how many
    if (count > 0) {
      steer.div(count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
      // Implement Reynolds: Steering = Desired - Velocity
      steer.normalize();
      steer.mult(this.maxspeed);
      steer.sub(this.velocity);
      steer.limit(this.maxforce);
    }
    return steer;
  }

  // Alignment
  // For every nearby boid in the system, calculate the average velocity
  align(boids) {
    let neighbordist = 40;
    let sum = createVector(0, 0);
    let count = 0;
    for (let i = 0; i < boids.length; i++) {
      let d = p5.Vector.dist(this.position, boids[i].position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(boids[i].velocity);
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      sum.normalize();
      sum.mult(this.maxspeed);
      let steer = p5.Vector.sub(sum, this.velocity);
      steer.limit(this.maxforce);
      return steer;
    } else {
      return createVector(0, 0);
    }
  }

  // Cohesion
  // For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
  cohesion(boids) {
    let neighbordist = 40;
    let sum = createVector(0, 0); // Start with empty vector to accumulate all locations
    let count = 0;
    for (let i = 0; i < boids.length; i++) {
      let d = p5.Vector.dist(this.position, boids[i].position);
      if ((d > 0) && (d < neighbordist)) {
        sum.add(boids[i].position); // Add location
        count++;
      }
    }
    if (count > 0) {
      sum.div(count);
      return this.seek(sum); // Steer towards the location
    } else {
      return createVector(0, 0);
    }
  }
}

function windowResized() {
  resizeCanvas(windowWidth,windowHeight/4);
}
