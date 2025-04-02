
class VisionController:
    """
    Vision controller for processing inputs and finding the can
    """
    def __init__(self, vision):
        self.vision = vision

    def get_can_position(self):
        """
        Scan around using movement controller, then identity can and return the estimated position. 
        """
        pass