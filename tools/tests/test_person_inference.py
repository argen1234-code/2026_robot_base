"""Regression checks for partially visible people and shared backend output layouts."""
import unittest

import numpy as np

from person_inference import ConfidenceGate, decode_persons


class PersonBoxTests(unittest.TestCase):
    def outputs(self):
        prediction = np.zeros((4,84),dtype=np.float32)
        prediction[0,:4] = [5,140,30,100]
        prediction[0,4] = .9
        prediction[1,:4] = [6,140,30,100]
        prediction[1,4] = .8
        prediction[2,:4] = [200,140,60,120]
        prediction[2,4] = .4
        prediction[2,4+56] = .95  # Chair: person score alone must not admit it.
        prediction[3,:4] = [-100,140,20,100]
        prediction[3,4] = .99  # A box entirely in letterbox/outside image is invalid.
        return prediction

    def test_edge_clipping_nms_and_class_filter(self):
        persons = decode_persons(self.outputs(),640,480,.5,0,40)
        self.assertEqual(len(persons),1)
        self.assertEqual(persons[0]['bbox'],[0.,100.,40.,200.])
        self.assertEqual(persons[0]['center'],[20.,200.])

    def test_ncnn_and_onnx_layouts_agree(self):
        prediction=self.outputs()
        expected=decode_persons(prediction,640,480,.5,0,40)
        self.assertEqual(decode_persons(prediction.T,640,480,.5,0,40),expected)
        self.assertEqual(decode_persons(prediction.T[None],640,480,.5,0,40),expected)

    def test_rectangular_letterbox_mapping(self):
        prediction=np.zeros((1,84),dtype=np.float32)
        prediction[0,:4]=[208,160,104,130]
        prediction[0,4]=.9
        person=decode_persons(prediction,640,480,.65,0,4)[0]
        self.assertEqual(person['bbox'],[240.,140.,160.,200.])
        self.assertEqual(person['center'],[320.,240.])

    def test_weak_current_box_supported_but_no_stale_pose(self):
        gate=ConfidenceGate()
        strong={'conf':.8,'bbox':[10,10,40,80]}
        weak={'conf':.25,'bbox':[12,10,40,80]}
        self.assertEqual(gate.filter([weak],now=10),[])
        self.assertEqual(gate.filter([strong],now=10.1),[strong])
        supported=gate.filter([weak],now=10.3)
        self.assertEqual(len(supported),1)
        self.assertTrue(supported[0]['confidence_supported'])
        self.assertEqual(gate.filter([],now=10.4),[])
        self.assertEqual(gate.filter([weak],now=10.8),[])

    def test_unrelated_weak_person_not_promoted(self):
        gate=ConfidenceGate()
        gate.filter([{'conf':.8,'bbox':[10,10,40,80]}],now=1)
        self.assertEqual(gate.filter([{'conf':.25,'bbox':[100,100,40,80]}],now=1.1),[])


if __name__=='__main__':
    unittest.main()
