from transformers import Owlv2Processor, Owlv2ForObjectDetection

import torch
import cv2
import numpy as np

class VisionController:
    """
    Vision controller for processing inputs and finding the can
    """
    def __init__(self):
        self.cap = cv2.VideoCapture(1)
        self.processor = Owlv2Processor.from_pretrained("google/owlv2-base-patch16-ensemble")
        self.model = Owlv2ForObjectDetection.from_pretrained("google/owlv2-base-patch16-ensemble")
        # list of tuples (confidence_score of the can, octet number)
        self.cans_observed = []


    def get_can_position(self, can_description, octet_number):
        """
        NOTE: This has not been tested
        
        can_description: str - The can that we are searching for ("a green can of sprite") 
        octet_number: int    - The octet number that describes the section of the field that the camera is looking at

        We are assuming that the camera is looking at 1/8th of the current field.
        This function will store the confidence scores of all the cans in the field in the cans_observed function.
        """
        for _ in range(100):
            _, frame = self.cap.read()

        texts = [[can_description]]
        inputs = self.processor(text=texts, images=frame, return_tensors="pt")

        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_object_detection(outputs=outputs, target_sizes=torch.Tensor([(640, 480)]), threshold=0.2)
        for s in results[0]["scores"]:
            self.cans_observed.append((s, octet_number))
        
        return True



    def center_the_frame(self, can_description):
        """
        can_description: str - The can that we are searching for ("a green can of sprite") 

        Will return the percentage to tilt the camera either right or left such that the can is centered in the frame.
        This is assuming that the camera is horizontal to the table.
        """

        # Take a hundred shots and keep the last one (camera seems to need a bit of a warm up)
        for _ in range(100):
            _, frame = self.cap.read()

        texts = [[can_description]]
        inputs = self.processor(text=texts, images=frame, return_tensors="pt")

        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_object_detection(outputs=outputs, target_sizes=torch.Tensor([(640, 480)]), threshold=0.2)
        boxes, scores, _ = results[0]["boxes"], results[0]["scores"], results[0]["labels"]

        # Get the best score
        index = np.argmax(scores).numpy()
        box = [round(i, 2) for i in boxes[index].tolist()]

        # Actual positions for it
        x1, _, x2, _ = tuple(box)
        center_x = abs(x1 - x2)
        distance = center_x - 320
        percentage = round((distance / 640) * 100, 2)
        if percentage < 0:
            print(f"Move camera by {abs(percentage) * 100}% to the left")
        else:
            print(f"Move camera by {abs(percentage) * 100}% to the right")

        return percentage


        
