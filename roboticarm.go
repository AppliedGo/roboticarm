/*
<!--
Copyright (c) 2016 Christoph Berger. Some rights reserved.
Use of this text is governed by a Creative Commons Attribution Non-Commercial
Share-Alike License that can be found in the LICENSE.txt file.

The source code contained in this file may import third-party source code
whose licenses are provided in the respective license files.
-->

+++
title = "Inverse Kinematics: how to move a robot arm (and why this is harder than it seems)"
description = "This article first glances over Inverse Kinematics. Then a small sample code implements a SCARA robot's arm movement."
author = "Christoph Berger"
email = "chris@appliedgo.net"
date = "2016-06-16"
publishdate = "2016-06-16"
domains = ["Robotics"]
tags = ["Inverse Kinematics", "SCARA", "Trigonometry"]
categories = ["Tutorial"]
+++

So you have built a robot arm? Great, let's make it serve your five o'clock tea. Sounds simple enough. Or is it?

<!--more-->

HYPE[SCARA robot arm writing hello](SCARA_left.html)
*Image By Pasimi (Own work) [<a href="http://creativecommons.org/licenses/by-sa/3.0">CC BY-SA 3.0</a>], <a href="https://commons.wikimedia.org/wiki/File%3ASCARA_left.gif">via Wikimedia Commons</a>*

## Forward kinematics

Calculating the current coordinates of a robot's hand is easy.

We just need to look at each segment of a robot's arm--the coordinates of the segment's base, the direction of the joint's axis, the angle between this segment and the next one, and the length of the segment--in order to calculate where the end of this segment is. Repeat this with each segment, until we arrive at the robot's hand. Et voilà: we determined the hand's position.

This is called forward kinematics.

## Inverse kinematics

Now the robot's arm must adjust each joint's angle in order to move its hand over the cup. This is quite the opposite of the previous calculation - here, we start with a given position and want to know how to rotate each segment of the arm.

It turns out that this is [much harder](https://en.wikipedia.org/wiki/Arm_solution) than the forward case. And whenever something is hard to solve, there are usually several different approaches available for solving that problem. For inverse kinematics, there are three:

1. The algebraic approach: This basically works by solving rather complex matrix equations.
2. The geometric approach: The idea is to combine knowledge about the robot arm's geometry with suitable trigonometric formulas.
3. The numeric approach: Take a guess and look how far we are off. Move one or more segments to locally minimize the error. Repeat.

Which one to pick? After all, each of them has its raison d'être.

For the sake of brevity, let's drop the first one. It involves a lot of matrix calculations, and frankly, I haven't done any since the last millennium or so.


## The SCARA robot arm

To make matters more simple, our robot has a very simple design.

* The arm has only two segments of fixed length.
* The segments can only rotate around their base joint; there is no sliding movement.
* The axes of both joints have the same direction.
* There is no hand attached to the end of the arm.

A robot of this kind is called a [SCARA robot](https://en.wikipedia.org/wiki/SCARA).

Here is a schematic diagram of our robot:

![The SCARA robot and a cup of tea](positions.png)

## Applying the geometric approach to the SCARA robot

Let me just tweak the diagram a little by replacing some of the labels and adding one line and two angles:

![The SCARA robot arm as a triangle](robotarm.png)

Now we can see that the segments have the length *len1* and *len2*, respectively. The root joint describes an angle *A1* measured from the x axis. The second joint describes an angle *A2* measured from the first segment (counterclockwise in both cases). At the tip of segment 2 there is a point *(x,y)*, and we want to calculate back from that point to the yet unknown values of *A1* and *A2*.

In the diagram you also see a dotted line named `dist`. It points from *(0,0)* to *(x,y)*, and as you can easily see, the three lines `dist`, *len1*, and *len2* define a triangle. Now is a good moment to dig out some old trig formulas from school.

It is fascinating [how many formulas there are](https://en.wikipedia.org/wiki/List_of_trigonometric_identities) just for reasoning about a simple triangle. Luckily, for our purposes, we only need one of them: The Law of Cosines.

![The Law of Cosines](lawofcosines.png)

The law of cosines (see the first formula in the picture above) is a generalization of the Pythargorean theorem for right triangles (c^2 = a^2 + b^2) to arbitrary triangles. We do not need the basic form, but rather the transformed version that you can see below the original formula. With this version, we can calculate angle *C* from the triangle's sides *a*, *b*, and *c*. This comes handy in two places.

But first, let's see what we need.

* From the robot arm picture above, we can directly derive the first formula:

      A1 = D1 + D2

* *D1* is fairly easy to calculate. This is good ol' Pythagoras:

  ![D1 is calculated from Pythagoras](calcd1.png)

* *D2* requires the law of cosines.
  Basically, we just map our "robot triangle" to the "law of cosines" triangle by using *dist* as *a*,
  *len1* as *b*, and *len2* as *c*. The resulting angle *C* is our *D2*.

  ![D2 and the law of cosines](calcd2.png)

  * Now only *A2* is left. Luckily, we can reuse the law of cosines for this. We only need to map our triangle to the one from the law of cosines with different parameter mappings than for *D2*: *len1* as *a*, *len2* as *b*, and *dist* as *c*.

  ![Calculating A2](calca2.png)

And that's it. Let's pour this into code now.

## The code
*/

//
package main

// Only the plain `math` package is needed for the formulas.
import (
	"fmt"
	"math"
)

// The lengths of the two segments of the robot's arm.
// Using the same length for both segments allows the robot
// to reach the *(0,0)* coordinate.
const (
	len1 = 10.0
	len2 = 10.0
)

// The law of cosines, transfomred so that *C* is the unknown.
// The names of the sides and angles correspond to the standard
// names in mathematical writing. Later, we have to map the
// sides and angles from our scenario to a, b, c, and C, respectively.
func lawOfCosines(a, b, c float64) (C float64) {
	return math.Acos((a*a + b*b - c*c) / (2 * a * b))
}

// The distance from *(0,0)* to *(x,y)*.
// HT to Pythagoras.
func distance(x, y float64) float64 {
	return math.Sqrt(x*x + y*y)
}

// Calculating the two joint angles for given x and y.
func angles(x, y float64) (A1, A2 float64) {
	// First, get the length of line *dist*.
	dist := distance(x, y)

	// Calculating angle D1 is trivial.
	// `Atan2` is a modified *arctan()* function that [returns unambiguous results.](https://golang.org/pkg/math/#Atan2)
	D1 := math.Atan2(y, x)

	// D2 can be calculated using the law of cosines where
	// a = dist, b = len1, and c = len2.
	D2 := lawOfCosines(dist, len1, len2)

	// Then A1 is simply the sum of D1 and D2.
	A1 = D1 + D2

	// A2 can also be calculated with the law of cosine, but this time with
	// a = len1, b = len2, and c = dist.
	A2 = lawOfCosines(len1, len2, dist)

	return A1, A2
}

// Convert radians into degrees.
func deg(rad float64) float64 {
	return rad * 180 / math.Pi
}

func main() {

	// Lets do some tests, first for (5,5):
	x, y := 5.0, 5.0
	a1, a2 := angles(x, y)
	fmt.Printf("x=%v, y=%v: A1=%v (%v°), A2=%v (%v°)\n", x, y, a1, deg(a1), a2, deg(a2))

	// If y is 0 and x = Sqrt(10^2 + 10^2), then alpha should become 45 degrees
	// and beta should become 90 degrees.
	x, y = math.Sqrt(200), 0
	a1, a2 = angles(x, y)
	fmt.Printf("x=%v, y=%v: A1=%v (%v°), A2=%v (%v°)\n", x, y, a1, deg(a1), a2, deg(a2))

	// Now let's try (1, 19).
	x, y = 1, 19
	a1, a2 = angles(x, y)
	fmt.Printf("x=%v, y=%v: A1=%v (%v°), A2=%v (%v°)\n", x, y, a1, deg(a1), a2, deg(a2))

	// An extreme case: (20,0).
	x, y = 20, 0
	a1, a2 = angles(x, y)
	fmt.Printf("x=%v, y=%v: A1=%v (%v°), A2=%v (%v°)\n", x, y, a1, deg(a1), a2, deg(a2))

	// And (0,20).
	x, y = 0, 20
	a1, a2 = angles(x, y)
	fmt.Printf("x=%v, y=%v: A1=%v (%v°), A2=%v (%v°)\n", x, y, a1, deg(a1), a2, deg(a2))

	// (0,0) technically works if the arm segments have the same length.
	// Still we get some weird result here!?
	x, y = 0, 0
	a1, a2 = angles(x, y)
	fmt.Printf("x=%v, y=%v: A1=%v (%v°), A2=%v (%v°)\n", x, y, a1, deg(a1), a2, deg(a2))

	// What happens if the target point is outside the reach? Like (20,20).
	x, y = 20, 20
	a1, a2 = angles(x, y)
	fmt.Printf("x=%v, y=%v: A1=%v (%v°), A2=%v (%v°)\n", x, y, a1, deg(a1), a2, deg(a2))
}

/* ## Outlook

The next article approaches the same problem from the numerical viewpoint. That is, we let the robot iteratively move its arm in small steps until it reaches the target.

Until then, have fun!
*/
