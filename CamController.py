
import bge

# Main program
def Main():
    scene = bge.logic.getCurrentScene()
    print(dir(bge.logic))
    cam1 = scene.objects["Camera.002"]
    cam2 = scene.objects["Camera"]

    cam1.useViewport = True
    cam2.useViewport = True
    cam2.setOnTop()

    width = bge.render.getWindowWidth()
    height = bge.render.getWindowHeight()
    print(width,height,'height----------------------------')



    bottom = 0
    top = height

    # here are the boundaries for the right viewport
    left_1 = 0
    right_1 = width

    # here are the boundaries for the left viewport
    left_2 = 0
    right_2 = int(width/3)

    # set player viewports with player1 at the right, player 2 at the left
    cam1.setViewport( int(width/2), 0, int(width), int(height/2))
    cam2.setViewport( 0, 0, right_2, int(height/3))

Main()


######  CREDITS  ######
######################################################
#    Viewports.py  Blender 2.50
#    Tutorial can be found at 
#    www.tutorialsforblender3d.com
#    Released under the Creative Commons Attribution 3.0 Unported License.	
#    If you use this code, please include this information header.
######################################################

### The original code from tutorialsforblender3d
### was modified by Tim Hickey (http://www.cs.brandeis.edu/~tim)
### on 10/15/2011 by stripping it down to a simpler program with less
### procedural abstraction. This code continues with the CCA3.0 license