Alles Im Fluss - Modelling Tools
================================
© 2008-2014 by Christoph Kubisch

This repository contains portions of the source-code for the [Alles Im Fluss](http://allesimfluss.biz/) polygon modelling toolkit. Active development of the toolkit has ceased.

![features 1](http://allesimfluss.biz/imagesHTML/keyfeatures2.png)

![features 2](http://allesimfluss.biz/imagesHTML/keyfeaturesv111.png)

## Important Notice
The code is meant for porting to other open-source modelling applications. As a consequence several key components of the original plugin code are missing. Contact the author if you are interested in licensing under a non-GPL license, or obtaining the full source-code.

## Key Components

### Brushes

There is several brushes that handle selection painting, but also typical sculpt deformations.

![Brushes](http://allesimfluss.biz/imagesHTML/brushconfig.png)

### Workplane

Drawing is performend in a work-plane, which is automatically derived from reference coordinate system and viewing angles, or local geometry.

![Workplane](http://allesimfluss.biz/imagesHTML/workplane.png)

### StrokeProcessor

The core engine to painting polygon strips and extrusions.

![StrokeEngine](http://allesimfluss.biz/imagesHTML/drawtypes.png)

### GridFill

A quad-dominant cap which implements a subset of [Filling N-sided holes for Subdivision Surfaces](http://staff.aub.edu.lb/~anasri/Research.html) by A. Nasri et al.

![GridCap](http://allesimfluss.biz/imagesHTML/capgrid.png)

### Mesh Classes

Several auxiliary classes to iterate loops and connected components. 


