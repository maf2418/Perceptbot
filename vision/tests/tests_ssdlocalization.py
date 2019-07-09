import os
import sys
sys.path.insert(0, os.getcwd() + "/src")

import unittest
from ssdlocalization import *
from geometry import CameraObj, make_transform, make_point3
from math import cos, sin
import numpy as np

class SSDTest(unittest.TestCase):
    def setUp(self):
        self.camera1 = CameraObj(make_transform(), 0)
        self.camera2 = CameraObj(make_transform(make_point3(0.25, 0.25, 0), -1.57), 0.5)
        self.camera3 = CameraObj(make_transform(make_point3(0.25, 0.25, 0), 1.57), 0.5)

        t1= np.eye(4)
        t1[0,0] = -.957
        t1[1,0] = 0.287
        t1[2, 0] = 0.02
        t1[0,1] = -0.288
        t1[1,1] = -0.958
        t1[0, 2] = 0.02
        t1[1, 2] = -0.006
        t1[0,3] = 1.044
        t1[1,3] = -0.705
        bearing1 = (-0.9866, 0.1547, 0.052)
        extant1 = (0.05889, 0.14821)
        screen1 = (-0.1366, 0.0448)
        t2 = np.eye(4)
        t2[0, 0] = -0.352
        t2[1, 0] = -0.935
        t2[2, 0] = 0.02
        t2[0, 1] = 0.936
        t2[1, 1] = -0.353
        t2[0, 2] = 0.007
        t2[1, 2] = 0.0187
        t2[0, 3] = 0.673
        t2[1, 3] = -0.302
        bearing2 = (-0.142, -0.984, 0.108)
        extant2 = (0.06725, 0.20175)
        screen2 = (-0.2203, 0.1259)

        self.camera4 = CameraObj(t1, 0.5)
        self.camera5 = CameraObj(t2, 0.5)
        self.camera1.set_camera_fov(90, 72)
        self.camera2.set_camera_fov(90, 72)
        self.camera3.set_camera_fov(90, 72)
        self.camera4.set_camera_fov(90, 72)
        self.camera5.set_camera_fov(90, 72)

        self.ob1 = Observation(self.camera1, "test", 0.5, 0.5, 0.05, 0.2)
        self.ob2 = Observation(self.camera2, "test", 0.5, 0.5, 0.05, 0.2)
        self.ob3 = Observation(self.camera3, "test", 0.5, 0.5, 0.05, 0.2 * self.camera3.scale_y / self.camera1.scale_y)
        self.ob4 = Observation(self.camera4, "test", 0.5 + 0.5 * screen1[0], 0.5 + 0.5 * screen1[1], extant1[0], extant1[1])
        self.ob5 = Observation(self.camera5, "test", 00.5 + 0.5 * screen2[0], 0.5 + 0.5 * screen2[1], extant2[0], extant2[1])

        self.proposal = []
        self.db = Database()

    def tearDown(self):
        pass


    def testPairingObservations(self):
        self.assertTrue(propose_position(self.ob1, self.ob2, self.proposal))
        self.assertTrue(self.ob1.is_matching(self.proposal))
        self.assertTrue(self.ob2.is_matching(self.proposal))
        print(self.proposal)

    def testPairRealObservations(self):
        self.ob4.print_me()
        self.ob5.print_me()
        self.assertTrue(propose_position(self.ob4, self.ob5, self.proposal))
        self.assertTrue(self.ob4.is_matching(self.proposal))
        self.assertTrue(self.ob5.is_matching(self.proposal))


    def testPairingObservationsDiffWidth(self):
        ob2b = Observation(self.camera2, "test", 0.5, 0.5, 0.4, 0.2)
        self.assertTrue(propose_position(self.ob1, ob2b, self.proposal))
        self.assertTrue(self.ob1.is_matching(self.proposal))
        self.assertTrue(self.ob2.is_matching(self.proposal))

    def testNonAlignedObservations(self):
        ob2b = Observation(self.camera2, "test", 0.25, 0.5, 0.1, 0.2)
        self.assertTrue(propose_position(self.ob1, ob2b, self.proposal))
        self.assertFalse(self.ob1.is_matching(self.proposal))

    def testWrongSizedObservations(self):
        ob2b = Observation(self.camera2, "test", 0.5, 0.5, 0.1, 0.4)
        self.assertTrue(propose_position(self.ob1, ob2b, self.proposal))
        self.assertFalse(self.ob1.is_matching(self.proposal))

    def testCameraFacingAway(self):
        ob3 = Observation(self.camera3, "test", 0.5, 0.5, 0.1, 0.2)
        self.assertFalse(propose_position(self.ob1, ob3, self.proposal))

    def testMatchToObservation(self):
        propose_position(self.ob1, self.ob2, self.proposal)
        match = Match.from_observations(self.ob1, self.ob2, self.proposal)
        self.assertTrue(is_with_match(self.ob1, match))
        self.assertTrue(is_with_match(self.ob2, match))

    def testNoMatchToDiffObservation(self):
        ob2b = Observation(self.camera2, "test", 0.25, 0.5, 0.1, 0.2)
        propose_position(self.ob1, self.ob2, self.proposal)
        match = Match.from_observations(self.ob1, self.ob2, self.proposal)
        self.assertFalse(is_with_match(ob2b, match))

    def testNoMatchToDiffHeight(self):
        ob2b = Observation(self.camera2, "test", 0.5, 0.5, 0.1, 0.4)
        propose_position(self.ob1, self.ob2, self.proposal)
        match = Match.from_observations(self.ob1, self.ob2, self.proposal)
        self.assertFalse(is_with_match(ob2b, match))

    def testDatabaseMatch(self):
        self.db.match_observations([self.ob1])
        self.db.match_observations([self.ob2])
        self.assertEqual(self.db.num_of_matches("test"), 1)

    def testDatabaseMatchOnlyOnce(self):
        self.db.match_observations([self.ob1])
        self.db.match_observations([self.ob2])
        self.db.match_observations([self.ob1])
        self.assertEqual(self.db.num_of_matches("test"), 1)

    def testNoMatchOfDiffIds(self):
        ob2b = Observation(self.camera2, "test diff", 0.5, 0.5, 0.1, 0.2)
        self.db.match_observations([self.ob1])
        self.db.match_observations([ob2b])
        self.assertEqual(self.db.num_of_matches("test"), 0)
        self.assertEqual(self.db.num_of_matches("test diff"), 0)
    

if __name__ == '__main__':
    unittest.main()
