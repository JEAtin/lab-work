#define _BOID
#include <vector>
#include <limits>
#include <algorithm>
#include "ofMain.h"

class Boid
{
// all the methods and variables after the
// private keyword can only be used inside
// the class
private:    
    ofVec3f position;
    ofVec3f velocity;
    
    float separationWeight;
    float cohesionWeight;
    float alignmentWeight;
    
    float separationThreshold;
    float neighbourhoodSize;
    
    ofVec3f separation(std::vector<Boid *> &otherBoids);
    ofVec3f cohesion(std::vector<Boid *> &otherBoids);
    ofVec3f alignment(std::vector<Boid *> &otherBoids);
    
// all the methods and variables after the
// public keyword can only be used by anyone
public:    
    Boid();
    Boid(ofVec3f &pos, ofVec3f &vel);
    
    ~Boid();
    
    ofVec3f getPosition();
    ofVec3f getVelocity();
    
    
    float getSeparationWeight();
    float getCohesionWeight();
    float getAlignmentWeight();
    
    float getSeparationThreshold();
    float getNeighbourhoodSize();
    
    void setSeparationWeight(float f);
    void setCohesionWeight(float f);
    void setAlignmentWeight(float f);
    
    void setSeparationThreshold(float f);
    void setNeighbourhoodSize(float f);
    
    void update(std::vector<Boid *> &otherBoids, ofVec3f &min, ofVec3f &max);
    
    void walls(ofVec3f &min, ofVec3f &max);
    
    void draw();
};

#endif
*  boid.cpp
*  boids
*
*  Created by Marco Gillies on 05/10/2010.
*  Copyright 2010 Goldsmiths, University of London. All rights reserved.
*
*/

#include "boid.h"
#include "ofMain.h"

Boid::Boid()
{
   separationWeight = 1.0f;
   cohesionWeight = 0.2f;
   alignmentWeight = 0.1f;
   
   separationThreshold = 15;
   neighbourhoodSize = 100;
   
   position = ofVec3f(ofRandom(0, 200), ofRandom(0, 200));
   velocity = ofVec3f(ofRandom(-2, 2), ofRandom(-2, 2));
}

Boid::Boid(ofVec3f &pos, ofVec3f &vel)
{
   separationWeight = 1.0f;
   cohesionWeight = 0.2f;
   alignmentWeight = 0.1f;
   
   separationThreshold = 15;
   neighbourhoodSize = 100;
   
   position = pos;
   velocity = vel;
}

Boid::~Boid()
{
   
}

float Boid::getSeparationWeight()
{
   return separationWeight;
}
float Boid::getCohesionWeight()
{
   return cohesionWeight;
}

float Boid::getAlignmentWeight()
{
   return alignmentWeight;
}


float Boid::getSeparationThreshold()
{
   return separationThreshold;
}

float Boid::getNeighbourhoodSize()
{
   return neighbourhoodSize;
}


void Boid::setSeparationWeight(float f)
{
   separationWeight = f;
}
void Boid::setCohesionWeight(float f)
{
   cohesionWeight = f;
}

void Boid::setAlignmentWeight(float f)
{
   alignmentWeight = f;
}


void Boid::setSeparationThreshold(float f)
{
   separationThreshold = f;
}

void Boid::setNeighbourhoodSize(float f)
{
   neighbourhoodSize = f;
}


ofVec3f Boid::getPosition()
{
   return position;
}

ofVec3f Boid::getVelocity()
{
   return velocity;
}

ofVec3f Boid::separation(std::vector<Boid *> &otherBoids)
{
   float nearest_distance = std::numeric_limits<float>::max();
   ofVec3f result(0, 0, 0);
   for (int i = 0; i < otherBoids.size(); i++)
   {   
       float distance = position.distance(otherBoids[i]->getPosition());
       ofVec3f v = position -  otherBoids[i]->getPosition();
       v.normalize();
       if (distance < nearest_distance) { 
           nearest_distance = distance;
           result = v;
       }
       if(distance < separationThreshold) {
           if (position.distance(position + separationThreshold * v - otherBoids[i]->getPosition()) >= separationThreshold) position += separationThreshold * v;
           else position -= separationThreshold * v;
       }
   }
   return result;
}

ofVec3f Boid::cohesion(std::vector<Boid *> &otherBoids)
{
   ofVec3f average(0,0,0);
   int count = 0;
   for (int i = 0; i < otherBoids.size(); i++)
   {
       if (position.distance(otherBoids[i]->getPosition()) < neighbourhoodSize)
       {
           average += otherBoids[i]->getPosition();
           count += 1;
       }
   }
   average /= count;
   ofVec3f v =  average - position;
   v.normalize();
   return v;
}

ofVec3f Boid::alignment(std::vector<Boid *> &otherBoids)
{
   ofVec3f average(0,0,0);
   int count = 0;
   for (int i = 0; i < otherBoids.size(); i++)
   {
       if (position.distance(otherBoids[i]->getPosition()) < neighbourhoodSize)
       {
           average += otherBoids[i]->getVelocity();
           count += 1;
       }
   }
   average /= count;
   ofVec3f v =  average - velocity;
   v.normalize();
   return v;
}

void Boid::update(std::vector<Boid *> &otherBoids, ofVec3f &min, ofVec3f &max)
{
   velocity += separationWeight*separation(otherBoids);
   velocity += cohesionWeight*cohesion(otherBoids);
   velocity += alignmentWeight*alignment(otherBoids);
   
   walls(min, max);
   position += velocity;
}

void Boid::walls(ofVec3f &min, ofVec3f &max)
{
   if (position.x < min.x){
       position.x = min.x;
       velocity.x *= -1;
   } else if (position.x > max.x){
       position.x = max.x;
       velocity.x *= -1;
   }
   
   if (position.y < min.y){
       position.y = min.y;
       velocity.y *= -1;
   } else if (position.y > max.y){
       position.y = max.y;
       velocity.y *= -1;
   }
   
   
}

void Boid::draw()
{
   ofSetColor(0, 255, 255);
   ofCircle(position.x, position.y, 5);
}