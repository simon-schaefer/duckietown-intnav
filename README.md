[![CircleCI](https://circleci.com/gh/duckietown/duckietown-intnav.svg?style=shield)](https://circleci.com/gh/duckietown/duckietown-intnav)

[![Coverage Status](https://coveralls.io/repos/github/duckietown/duckietown-intnav/badge.svg?branch=master18)](https://coveralls.io/github/duckietown/duckietown-intnav?branch=master18)

[![PyPI status](https://img.shields.io/pypi/status/duckietown_intnav.svg)](https://pypi.python.org/pypi/duckietown_intnav/)


[![PyPI pyversions](https://img.shields.io/pypi/pyversions/duckietown_intnav.svg)](https://pypi.python.org/pypi/duckietown_intnav/)


# unicorn

Reliable and Efficient monocular-based Intersection navigation


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
