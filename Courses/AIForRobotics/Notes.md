# AI for Robotics

[Localization](#localization)
- 
[Kalman Filters](#kalman-filters)

[Particle Filters](#particle-filters)

[Search](#search)

[PID Control](#pid-control)

[Simultaneous Localization and Mapping](#slam)

[References](#references)

## Introduction
### Uncertainity in Robotics
Robots have to deal with uncertainity in the world. Unless they are able to handle this uncertainity, robot applications will remain limited in scope. 

There are many factors that contribute to uncertainity. 
1. First, is the **robot environment**. Different environments have different degrees of uncertainity. Assembly lines are most structured, whereas highways and roads are much more uncertain. Robots working around people have to deal with most uncertainity. 
2. Second, is the **sensor**. Sensors are not capable of infinte accuracy, neither they can measure everything. They are often limited by their range and resolution. They are also subject to noise which perturb sensor measurements in unpredictible ways. Finally, sensors can break, and detecting faulty sensor can be extremely difficult
3. Third is **actuation**. Robot motors are imperfect, this could be due to control noise, wear-and-tear and mechanical failure. 
4. Fourth is **internal models**. The robot operates under certain assumption about the real world, embodied in internal models used by the software. These models can be faulty. Model errors are a source of uncertainity that has often been ignored in robotics
5. Fifth, is **algorithmic approximation**. Robots are real-time systems, this limits the amount of computation that can be carried out. Computational techniques employed to improve real-time performance often introduce approximation errors. 

### Probabilistic Robotics
It is a relatively new field in robotics, that pays tribute to the uncertainity in robot perception and action. The key idea in probabilistic robotics is to represent the uncertainity explicitly using calculus of probability. What this means? Instead of a single value, probabilitic algorithms represent information as probability distribution over a whole set of possible values. 

### Probabilisitc Robotics vs Traditional Robotics
Some advantages of probabilisitic robotics over traditional robotics (model based)
1. Prob. Robotics scale well for real-world environment because they explicitly model uncertainity
2. Prob. Robotics do not require accurate sensors as they can handle sensor noise
3. Prob. Robotics have weaker requirements on accuracy robot's models

Two most frequently cited limitations of probabilistic robotics
1. computational complexity
2. need to approximate: computing exact posterior for continuous space is computationally intractable. Sometimes,
one is fortunate in that the uncertainty can be approximated tightly with a compact parametric model (e.g., Gaussians). In other cases, such approximations are too crude to be of use, and more complicated representations must be employed.


## Localization

### The Localization Problem
How can we know where we are with accuracy of +/- 10cm. This is much better than GPS. 

### Localization Application
Used in Google's self driving car. Takes images of the road surface, and then uses techniques to localize itself with an accuracy of a few centimeters. 

### Intuition About Localization Math
We model a robot's estimation of its position in space with a probability distribution function (pdf). Initially, the robot has no clue of where it is, so it could be anywhere. This is modeled as uniformly distributed. 

To localize, the world has to have some distinctive features or landmarks. When the robot's sensors detect landmarks, the robot can modify its belief (encoded in the localization pdf). For example, in the image below, landmarks are 3 identical doors, when robot detects a door, belieft is modified, and it now assigns equal porbability of being at one of three doors, and very low probability of being at other locations. 
![localization_doors](images/localization_doors.png)

The new belief is referred to as the posterior belief because it is after the measurement. Now sa the robot moves, the belief moves with it. Let's understand this part... The belief is a representation of robot's position in space, if the robot were to move to right by 10m, it would now say that it is is one of the three positions 10m to the right of positions identified previously. This is depicted in the modified belief. 
![belief_movement](images/belief_movement.png)
Notice that, the belief peak are spread out, this is due to error in sensing movement. This is referred to as the convoluton, where the movement pdf convolves with belief pdf. Now, with a second measurement from sensors, we are able to generate a new belief which depends on the prior (convolved pdf) and current measurement. It assigns most weight to the where the second door is. 
![belief modification](images/belief_modification.png)

We achive this by multiplying the prior to the belief generate by current measurements (similar to our first belief). 

### Representing probabability Distribution
We can represent probabilities as a vector. In python, this would be a list
```
p = []                                 # empty list
p = [0.2, 0.2, 0.2, 0.2, 0.2]          # list with 5 elements, represents uniform distribution over 5 grid cells

p = []
n = 5
for x in range(n):                     # generalized uniform distribution with arbitrary size (n)
    p.append(1/n)
```


### Probability After Sense
![prob_sense](images/prob_sense.png)
Note that even after sensing red, the probability associated with green block is non-zero. This is because the sensor measurements can be inaccurate, so it is accounted for by non-zero probability of green block. We also need to normalize the posterior belief. 

### Defining the Sense Function
The sense function takes in the current belief and sensor measurement and other globals (map of world), and generates the unnormalized posterior belief.Here is an example implmentation
```
#Modify your code so that it normalizes the output for 
#the function sense. This means that the entries in q 
#should sum to one.


p=[0.2, 0.2, 0.2, 0.2, 0.2]                       # current belief
world=['green', 'red', 'red', 'green', 'green']   # map of world
Z = 'red'                                         # current measurement
pHit = 0.6                                        
pMiss = 0.2

def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    return q
print sense(p,Z)
```
The final (normalized) version would look like so
```
def sense(p, Z):
    q=[]
    for i in range(len(p)):
        hit = (Z == world[i])
        q.append(p[i] * (hit * pHit + (1-hit) * pMiss))
    s = sum(q)
    for x in range(len(q)):
        q = q[x]/s
    return q
```


## Kalman Filters

## Particle Filters

## Search

## PID Control

## SLAM


## References
- [Udacity AI for Robotics](https://classroom.udacity.com/courses/cs373)
- Seminal Papers in Probabilistic Robotics
  - [Kalman Filter Techniques for High-Dimensional Perception, Smith and Cheesman, 1986](https://www.frc.ri.cmu.edu/~hpm/project.archive/reference.file/Smith&Cheeseman.pdf)
  - Invention of occupancy grid maps
    - [Elfes, 1987](https://www.cs.cmu.edu/~motionplanning/papers/sbp_papers/integrated4/elfes_occup_grids.pdf)
    - [Moravec, 1988](http://www.aaai.org/ojs/index.php/aimagazine/article/viewFile/676/594)
  - [Partially Observable Planning Techniques, Kaelbling, 1998](https://www.seas.upenn.edu/~mkearns/papers/barbados/klc-pomdp.pdf)
  - [Particle Filters, Dellaert, 1999](https://www.ri.cmu.edu/pub_files/pub1/dellaert_frank_1999_2/dellaert_frank_1999_2.pdf)
  - Bayesian Information Processing
    - [Thrun, 2000b](http://www.cs.cmu.edu/~thrun/movies/papers/thrun.map3d.pdf)
    - [Lebeltel, 2004](http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.104.7379&rep=rep1&type=pdf)
    - [Park, 2005](http://reports-archive.adm.cs.cmu.edu/anon/anon/usr/ftp/usr0/ftp/2004/CMU-CS-04-173.pdf)
