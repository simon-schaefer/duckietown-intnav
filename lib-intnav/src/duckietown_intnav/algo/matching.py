#!/usr/bin/env python
###############################################################################
# Duckietown - Project intnav ETH
# Author: Simon Schaefer
# Select and match keypoints from image and database using heuristics (e.g. 
# distance to last keypoints pixel coordinate), sum of squared distances 
# (SSD) quantitative measure and brute force matching (small # features). 
###############################################################################
__all__ = [
    'Matching',
]

import cv2

class Matching: 
    
    def __init__(self):
        ''' Initialize internal keypoint matching memory as well 
        as matching algorithm.  '''
        # Load program parameters. 
        # params = {}
        # try: 
        #     config_file = os.path.dirname(os.path.realpath(__file__))
        #     config_file = os.path.join(config_file, "../data/parameter.yaml")
        #     with open(config_file, 'r') as stream:
        #         params = yaml.load(stream)
        # except (IOError, yaml.YAMLError): 
        #     raise IOError("Unknown or invalid parameters file !") 
        # # Check input arguments validity. 
        # if not detector in Features.detectors_options: 
        #     raise ValueError("Invalid detector %s !" % detector)
        # if not descriptor in Features.descriptors_options: 
        #     raise ValueError("Invalid descriptor %s !" %descriptor)
        # Initialize internal memory dictionary of matches, storing the 
        # image (not database) pixel position of last match.
        self._memory = []
        # Initialize matching algorithm. 
        self._matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    def process(self, kp, kp_database, descs, descs_database): 
        ''' Match descriptors in image with descriptors in database using
        the sum of squared distance metric. To avoid non-unique matches and 
        save computational ressources apply heuristic in order to select 
        possible matching candidates before matching. '''
        matches = self._matcher.match(descs, descs_database)
        return matches

    @staticmethod
    def draw(img, kp, img_db, kp_db, matches): 
        ''' Draw best 5 matches between image and database image. '''
        matching_image = cv2.drawMatches(img, kp, img_db, kp_db, matches[:5], img)
        cv2.imshow("matches", matching_image)
        cv2.waitKey(0)