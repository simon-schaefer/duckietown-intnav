[![CircleCI](https://circleci.com/gh/duckietown/duckietown-intnav.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-intnav)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-intnav/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-intnav?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown_intnav.svg)](https://pypi.python.org/pypi/duckietown_intnav/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown_intnav.svg)](https://pypi.python.org/pypi/duckietown_intnav/)


# Project Unicorn

Reliable and Efficient monocular-based Intersection navigation

## iMap

Map representation library for duckietown intersections 

<img src="documentation/imap/imap_example.png" alt="" style="width: 200px;"/>

### Conventions

The iMap library implements four kinds of intersection, the 4-way intersection (which is symmetric to rotation) and the three possible 3-way intersections (from "duckiebot's perspective": left-straight (3SL), left-right (3LR), straight-right (3SR)). All coordinate systems are implemented such that the duckiebot starts in the upper half of the image, therefore the coordinate system is introduced in the middle of the upper lane (see figure). 

### Visualization

iMap contains visualization functionalities for a planned trajectory, the duckiebot's pose and further points to visualize (e.g. keypoints for localization) and publishes the resulting graphic as sensor_msgs::Img (viewable e.g in rqt_image_view). 

### Demo

```
roslaunch imap test_visualization.launch
```


## Installation from source

This is the way to install within a virtual environment created by 
using `pipenv`:

    $ pipenv install
    $ pipenv shell
    $ cd lib-unicorn
    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps
    
   
## Unit tests

Run this:

    $ make -C lib-unicorn tests-clean tests
    
The output is generated in the folder in `lib-unicorn/out-comptests/`.
