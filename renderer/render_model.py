from OpenGL.GL import *
from PIL import Image
import sys
import math
import numpy as np
import time
import imgui
import glfw

import magic
# We import the 'lab_utils' module as 'lu' to save a bit of typing while still clearly marking where the code came from.
import lab_utils as lu
from ObjModel import ObjModel
import numpy as np

# %matplotlib inline
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

g_cameraDistance = 45
g_yFovDeg = 56.9
g_cameraYawDeg = 0.0
g_cameraPitchDeg = 0.0
g_lookPos = [0.0, 0.0, 0.0]
g_camRotation = [0.0, 0.0, 0.0]
g_model = None

def fovCalc(focalLength):
    return 180/math.pi * 2 * math.atan(12/focalLength)

def quarternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.degrees(math.atan2(t0, t1))

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.degrees(math.asin(t2))

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.degrees(math.atan2(t3, t4))

    return X, Y, Z

def averageColor(colorArray):
    rSum, gSum, bSum, aSum = 0, 0, 0, 0
    pixelCount = len(colorArray)
    if pixelCount == 0: return (0, 0, 0, 0)
    for i in range(pixelCount):
        rSum += colorArray[i][0]
        gSum += colorArray[i][1]
        bSum += colorArray[i][2]
        aSum += colorArray[i][3]
    return (rSum/pixelCount, gSum/pixelCount, bSum/pixelCount, aSum/pixelCount)

def renderFrame(width, height, camPosition, camRotation):
    global g_cameraDistance
    global g_cameraYawDeg
    global g_cameraPitchDeg
    global g_yFovDeg
    global g_lookPos
    global g_camRotation
    global g_frameBufferID
    global g_screenTextureID

    # This configures the fixed-function transformation from Normalized Device Coordinates (NDC)
    # to the screen (pixels - called 'window coordinates' in OpenGL documentation).
    #   See: https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glViewport.xhtml
    glViewport(0, 0, width, height)
    # Set the colour we want the frame buffer cleared to, 
    glClearColor(0.0, 0.0, 0.0, 1.0)
    # Tell OpenGL to clear the render target to the clear values for both depth and colour buffers (depth uses the default)
    glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT)

    viewToClipTransform = magic.make_perspective(g_yFovDeg, width/height, 1.0, 1000.0)

    worldToViewTransform = lu.make_rotation_x(math.radians(camRotation[0])) \
        * lu.make_rotation_y(math.radians(camRotation[1])) \
        * lu.make_rotation_z(math.radians(camRotation[2])) \
        * lu.make_translation(camPosition[0], camPosition[1], camPosition[2])

    output = None

    fbo = glGenFramebuffers(1)
    render_buf = glGenRenderbuffers(1)
    glBindRenderbuffer(GL_RENDERBUFFER, render_buf)
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, width, height)
    glBindFramebuffer(GL_FRAMEBUFFER, fbo)
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, render_buf)

    glBindFramebuffer(GL_FRAMEBUFFER, fbo)
    magic.drawObjModel(viewToClipTransform, worldToViewTransform, lu.make_translation(0, 0, 0), g_model)

    glReadBuffer(GL_COLOR_ATTACHMENT0) 
    buffer = glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, output)

    return Image.frombytes(mode="RGBA", size=(width, height), data=buffer)
    # image.save("out.png")

    # magic.drawCoordinateSystem(viewToClipTransform, worldToViewTransform)

def initResources():
    global g_model
    global g_frameBufferID
    global g_screenTextureID

    g_model = ObjModel(sys.argv[2])
    glEnable(GL_CULL_FACE)

### Program Start

print("The arguments are:")
print(str(sys.argv))
# load cli arguments
if (len(sys.argv) != 6):
    print('Incorrect number of arguments given. I will now exit.')
    sys.exit(1)

atlasPath = sys.argv[1] #empy texture map to be filled
objPath = sys.argv[2] #untextured model
txtPath = sys.argv[3] #images.txt
cameraImagesFolder = sys.argv[4]
outputPath = sys.argv[5] #texture map filled

# create imgui context
print("Creating imgui context...")
imgui.create_context()
if not glfw.init():
    sys.exit(1)

# open textureAtlas
print("Opening textureAtlas...")
images_count = 60
textureImage = Image.open(atlasPath)
textureWidth = textureImage.size[0]
textureHeight = textureImage.size[1]
textureAtlas = np.zeros((textureWidth,textureHeight,3))
textureAtlas_list = np.zeros((images_count+1, textureWidth, textureHeight ,3))

# init params
g_yFovDeg = fovCalc(32)
window, impl = None, None

# look through images.txt
print("Scanning through images.txt...")
with open(txtPath, 'r', encoding="utf8") as fp:
    # for each line
    ln = fp.readline()
    count = 1
    # while ln:
    while count<images_count:
        ln = ln.split()
        if ln[0].isnumeric(): # hack to check that line starts with an integer
            count += 1
            # open camera image
            cameraImage = Image.open(cameraImagesFolder + ln[9])
            cameraPixels = cameraImage.load()
            if window == None:
                # init glfw
                window, impl = magic.initGlFwAndResources("Atlas generator", cameraImage.size[0], cameraImage.size[1], initResources)
            # convert quarternion -> (rx, ry, rz) and get (x, y, z)
            rx, ry, rz = quarternion_to_euler(
                float(ln[1]),
                float(ln[2]),
                float(ln[3]),
                float(ln[4])
            )

            x, y, z= float(ln[5]), float(ln[6]), float(ln[7])
            # render model from camera pose
            uvMap = renderFrame(cameraImage.size[0], cameraImage.size[1], (x, y, z), (rx, ry, rz))
            # uvMap.show()
            # imgui.render() # do I need this?

            uvPixels = uvMap.load()
            # for pixel in image
            for i in range(uvMap.size[0]):
                for j in range(uvMap.size[1]):
                    if (uvPixels[i, j][3] == 0): continue
                    # put pixel into texture atlas
                    u = round(textureWidth * uvPixels[i, j][0]/255)-1
                    v = round(textureHeight * uvPixels[i, j][1]/255)-1
                    textureAtlas[u][v] = cameraPixels[i, j]
            textureAtlas_list[count] = textureAtlas

        ln = fp.readline()
        print(float(count/images_count))
fp.close()

textureAtlas = np.average(textureAtlas_list, axis=0)
print(textureAtlas.min())
print(textureAtlas.max())
print(textureAtlas.mean())
plt.imshow(textureAtlas)
plt.show()

# export texture atlas
plt.savefig(outputPath)

# textureAtlas.save(outputPath)
print("Successfully generated texture atlas")
