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
const (
	l1 = 10.0
	l2 = 10.0
)

// The distance d from *(0,0)* to *(x,y)*, squared.
func squareDist(x, y float64) float64 {
	return x*x + y*y
}

func beta1(x, y float64) float64 {
	return math.Atan2(y, x)
}

func beta2(dsq, l1, l2 float64) float64 {
	return math.Acos((dsq + l2*l2 - l1*l1) / (2 * l2 * math.Sqrt(dsq)))
}

func alpha2(dsq, l1, l2 float64) float64 {
	return math.Acos((dsq + l1*l1 - l2*l2) / (2 * l1 * math.Sqrt(dsq)))
}

func move1(x, y float64) (a1, a2 float64) {
	d := squareDist(x, y)
	return beta1(x, y) + beta2(d, l1, l2), alpha2(d, l1, l2)
}

func lawOfCosine(a, b, c float64) (alpha float64) {
	return math.Acos((b*b - a*a + c*c) / (2 * b * c))
}

func move2(x, y float64) (a1, a2 float64) {
	d := math.Sqrt(squareDist(x, y))
	return math.Atan2(x, y) + lawOfCosine(l1, l2, d), lawOfCosine(l2, d, l1)
}

func main() {
	a1, a2 := move1(5, 5)
	fmt.Println("x=5, y=5:", a1, ",", a2)
	a1, a2 = move2(5, 5)
	fmt.Println("x=5, y=5:", a1, ",", a2)
	a1, a2 = move1(math.Sqrt(200), 0)
	fmt.Println("x=math.Sqrt(200), y=0:", a1, ",", a2)
	a1, a2 = move2(math.Sqrt(200), 0)
	fmt.Println("x=math.Sqrt(200), y=0:", a1, ",", a2)
}
