
class JevoisModule(object):

    def __init__(self, name, fps, should_start_guvcview):
        self.name = name
        self.fps = fps
        self.should_start_guvcview = should_start_guvcview

    # To override
    # Params to load on Jevois camera
    # Return an array of commands to send via serial
    def get_params(self):
        raise NotImplementedError()
