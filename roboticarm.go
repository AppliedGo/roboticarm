/*
<!--
Copyright (c) 2016 Christoph Berger. Some rights reserved.
Use of this text is governed by a Creative Commons Attribution Non-Commercial
Share-Alike License that can be found in the LICENSE.txt file.

The source code contained in this file may import third-party source code
whose licenses are provided in the respective license files.
-->

+++
title = ""
description = ""
author = "Christoph Berger"
email = "chris@appliedgo.com"
date = "2016-00-00"
publishdate = "2016-00-00"
domains = [""]
tags = ["", "", ""]
categories = ["Tutorial"]
+++

### Summary goes here

<!--more-->

## Intro goes here

## The code
*/

// ## Imports and globals
package main

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

// The law of cosine, transfomred so that *C* is the unknown.
// The names of the sides and angles correspond to the standard
// names in mathematical writing. Later, we have to map the
// sides and angles from our scenario to a, b, c, and C, respectively.
func lawOfCosine(a, b, c float64) (C float64) {
	return math.Acos((a*a + b*b - c*c) / (2 * a * b))
}

// The distance from *(0,0)* to *(x,y)*, squared.
func squareDist(x, y float64) (dist float64) {
	return x*x + y*y
}

// Calculating angle D1 is trivial.
// Atan2 is a modified atan() function that returns
// unambiguous results.
func D1(x, y float64) float64 {
	return math.Atan2(y, x)
}

// Calculating the two joint angles for given x and y.
func angles(x, y float64) (A1, A2 float64) {
	// First, get the length of line *dist*.
	dist := math.Sqrt(squareDist(x, y))

	// D2 can be calculated using the law of cosines where
	// a = dist, b = len1, c = len2
	D2 := lawOfCosine(dist, len1, len2)

	// Then A1 is simply the sum of D1 and D2.
	A1 = D1(x, y) + D2

	// A2 can also be calculated with the law of cosine, but this time with
	// a = len1, b = len2, c = dist
	A2 = lawOfCosine(len1, len2, dist)

	return A1, A2
}

/*
### An unnecessary calculation

Did you notice the superfluous calculation hidden in the functions? `angles()` passes
the square root of `squareDist()` to lawOfCosine(), where the value is again squared. To square the
square root of a value is a no-op, so let's see if we can avoid this, as we do already have the square
value readily available.

We could change `lawOfCosine()` to accept the squared value of *dist* rather than *dist* itself. However,
`angles()` calls `lawOfCosine()` two times, and the second time with a shifted parameter list. Because of
this, *dist* is passed to `lawOfCosine()` in the place of *c*. So `lawOfCosine` would treat *a* as the
squared value, which is not correct.

If we want to eliminate the no-op, we have no choice but to define two separate functions.
*/

func deg(rad float64) float64 {
	return rad * 180 / math.Pi
}

func main() {

	// Lets do some tests, first for (5,5):
	a1, a2 := angles(5, 5)
	fmt.Printf("angles  - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))
	//a1, a2 = angles2(5, 5)
	//fmt.Printf("angles2 - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))

	// If y is 0 and x = Sqrt(10^2 + 10^2), then alpha should become 45 degrees
	// and beta should become 90 degrees.
	a1, a2 = angles(math.Sqrt(200), 0)
	fmt.Printf("angles  - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))
	//1, a2 = angles2(math.Sqrt(200), 0)
	//mt.Printf("angles2 - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))

	// Now let's try (1, 19)
	a1, a2 = angles(1, 19)
	fmt.Printf("angles  - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))
	//1, a2 = angles2(1, 19)
	//mt.Printf("angles2 - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))

	// (20,0)
	a1, a2 = angles(20, 0)
	fmt.Printf("angles  - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))
	//1, a2 = angles2(20, 0)
	//mt.Printf("angles2 - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))

	// (0,20)
	a1, a2 = angles(0, 20)
	fmt.Printf("angles  - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))
	//1, a2 = angles2(0, 20)
	//mt.Printf("angles2 - x=%v (%v°), y=%v (%v°)\n", a1, deg(a1), a2, deg(a2))
}
