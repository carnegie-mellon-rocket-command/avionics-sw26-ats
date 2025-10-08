class ATS:
    """Represents actual control ATS/servo"""
    def __init__(self, max_extension=1.0):
        self.extension = 0.0
        self.max_extension = max_extension

    def set_extension(self, extension):
        self.extension = max(0.0, min(1.0, extension))

    def get_drag_coefficient(self) -> float:
        # fill in function approximation/lookup table
        # use self.extension to lookup/approximate
        pass