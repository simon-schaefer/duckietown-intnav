[![CircleCI](https://circleci.com/gh/duckietown/duckietown-intnav.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-intnav)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-intnav/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-intnav?branch=master18)

# Project intnav

Reliable and Efficient monocular-based Intersection navigation

## Installation from source

We recommend to run our package natively: 

    $ cd lib-intnav
    $ pip install -r requirements.txt --user
    $ python setup.py develop --no-deps --user

To run the package using a virtual environment created by `pipenv` the 
following steps are necessary:

    $ pipenv install
    $ pipenv shell
    $ cd lib-intnav
    $ pip install -r requirements.txt
    $ python setup.py develop --no-deps

## Unit tests

Run this:

    $ make -C lib-intnav tests-clean tests
    
The output is generated in the folder in `lib-intnav/out-comptests/`.
