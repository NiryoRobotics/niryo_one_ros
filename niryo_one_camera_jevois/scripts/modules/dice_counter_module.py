from modules.jevois_module import JevoisModule


class DiceCounterModule(JevoisModule):

    def __init__(self, name):
        super(DiceCounterModule, self).__init__(
            name=name, fps=7.5, should_start_guvcview=True)

    def get_params(self):
        return ['streamoff',
                'setmapping2 YUYV 640 480 7.5 SampleVendor DiceCounter',
                'setpar serout USB',
                'streamon']
