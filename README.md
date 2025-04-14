# Software for MVG-25sp

This is a spring undergrad research to understand
multiple view geometry by writing code, doing calculations,
and implementing it on our six-legged (hexapod) robot
for identifying and capturing growing crop plants.

## Main Project Areas

We will work on these areas roughly in order,
earlier areas are pre-requisites to later areas.

* 1. Calibration Cube
* 2. Hexapod Robot
* 3. MATLAB to Octave Code
* 4. Infrared Positioning
* 5. Image Segmentation
* 6. 3D Reconstruction
* 7. Plant Classification

## 1. Calibration Cube

Reading the text and working with general mathematical
camera matrices can sometimes be abstract.

We will build a physical calibration cube out of wood,
cardboard, and paper, with OCR text to label points
for calibration in our projective geometry.

For learning 8 camera parameters in a single view as
described in Hartley Zisserman and the Stanford class notes,
we only need 8 points to solve for 8 unknowns, and to use
Singular-Value-Decomposition.

## 2. Hexapod Robot

This is a six-legged robot with a camera that can be used to
automate image capture and data collection over a large area
(ideally, all of Evergreen's organic farm or forests), aiding
and collaborating with human data gathering, and also providing
more precise geolocating.

At the beginning, the robot with a camera could be replaced by
a 3D printer with the hot end / extrusion replaced with a camera.

## 3. MATLAB to Octave Code

The code from the Hartley and Zisserman textbook comes in MATLAB,
but this is an expensive package, not available on Evergreen
computers, does not appear to make use of modern GPU advances,
and is in general harder-to-learn and inaccessible.

We propose to convert and test as much of this openly-available
code on the open source version GNU Octave, as well as 
freely available Python libraries such as `numpy` and `scipy`.

## 4. Infrared Positioning 

We will build an IR (infrared) emitter and receive circuit
using discrete circuit parts following this design.

## 5. Image Segmentation (Optional)

One our robot is able to capture images of a plant,
most of the image will consist of a background (soil,
other plants, buildings, etc.). We want to partition
each image it captures into smaller segments making it easier to  
run analysis algorithms.

In computer vision, this task is known as *image segmentation*.

This part is optional, as the trend in machine learning work is
to ingest as much data as possible and train One Model to
Rule Them All. Meaning if we were to train a classifier,
we wouldn't want to shield it from negative training examples of
what is *not* a plant, because image segmentation of
plant vs. not plant is still a human / hand-labeled feature
that could be automated.

## 6. 3D Reconstruction  (Optional)

A 3D geometry of a plant, at any stage of its growth,
can be used to generate new images or a "digital twin"
for visualization purposes.

This is optional as machine learning tasks operate on
raw data, and don't care whether it is interpretable as
2D or 3D data. This is something humans make a distinction for.

However, as humans and for aesthetic reasons, we may still
wish to stop and rest at this "intermediate" stage.

## 7. Plant Classification

Finally, our end goal is to produce an automated classifier
and model
so that when a robot encounters a new plant crop growing in the
world, or a new crop field, it runs its model,
or sends the model back by wifi
to a server, and it can classify the species.

This is useful pedagogically because we've created a new
training set that can be cleaned up and published for
future computer science students.
