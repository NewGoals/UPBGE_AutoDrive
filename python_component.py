import bge
import MathLib
#import VehicleWheel
from collections import OrderedDict

class MyComponent(bge.types.KX_PythonComponent):
    # Put your arguments here of the format ("key", default_value).
    # These values are exposed to the UI.
    args = OrderedDict([
        ("my Var", 12345)
    ])

    def start(self, args):
        # Put your initialization code here, args stores the values from the UI.
        # self.object is the owner object of this component.
        # self.object.scene is the main scene.
        pass

    def update(self):
        # Put your code executed every logic step here.
        # self.object is the owner object of this component.
        # self.object.scene is the main scene.
        pass
