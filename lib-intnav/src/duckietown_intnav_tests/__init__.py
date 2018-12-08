# coding=utf-8

# add an import for each test file in this directory
from .timing import *
from .config import *
from .vcompass import *


def jobs_comptests(context):
    """ Hook for comptests. No need to modify."""
    from comptests.registrar import jobs_registrar_simple
    jobs_registrar_simple(context)
