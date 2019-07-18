from OpenGL.GL import *
from PIL import Image
import sys
import math
import numpy as np
import time
import imgui

import magic
# We import the 'lab_utils' module as 'lu' to save a bit of typing while still clearly marking where the code came from.
import lab_utils as lu
from ObjModel import ObjModel

g_cameraDistance = 45
g_yFovDeg = 56.9
g_cameraYawDeg = 0.0
g_cameraPitchDeg = 0.0
g_lookPos = [0.0, 0.0, 0.0]
g_camRotation = [0.0, 0.0, 0.0]
g_model = None

def renderFrame(width, height):
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

    worldToViewTransform = lu.make_rotation_x(math.radians(g_camRotation[0])) \
        * lu.make_rotation_y(math.radians(g_camRotation[1])) \
        * lu.make_rotation_z(math.radians(g_camRotation[2])) \
        * lu.make_translation(g_lookPos[0], g_lookPos[1], g_lookPos[2])

    output = None
    # fbo = glGenFramebuffers(1)
    # colorRenderBuffer, depthRenderBuffer = magic.setupFbo(fbo, width, height, 1)

    fbo = glGenFramebuffers(1)
    render_buf = glGenRenderbuffers(1)
    glBindRenderbuffer(GL_RENDERBUFFER, render_buf)
    glRenderbufferStorage(GL_RENDERBUFFER, GL_RGBA, width, height)
    glBindFramebuffer(GL_FRAMEBUFFER, fbo)
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, render_buf)

     # glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo)
    # glBindFrameBufferEXT(GL_FRAMEBUFFER_EXT, framebufferID)
    glBindFramebuffer(GL_FRAMEBUFFER, fbo)
    magic.drawObjModel(viewToClipTransform, worldToViewTransform, lu.make_translation(0, 0, 0), g_model)
    # glBindTexture(GL_FRAMEBUFFER, fbo)
    # glGetTexImage(GL_FRAMEBUFFER, 0, GL_RGBA, GL_UNSIGNED_BYTE, output)
    #glBindFramebuffer(GL_FRAMEBUFFER, fbo)

    glReadBuffer(GL_COLOR_ATTACHMENT0) 
    buffer = glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, output)
    # print(buffer)
    image = Image.frombytes(mode="RGBA", size=(width, height), data=buffer)
    image.save("out.png")
    sys.exit()

    magic.drawCoordinateSystem(viewToClipTransform, worldToViewTransform)

def initResources():
    global g_model
    global g_frameBufferID
    global g_screenTextureID

    g_model = ObjModel(sys.argv[1])
    glEnable(GL_CULL_FACE)

def fovCalc(focalLength):
    return 180/math.pi * 2 * math.atan(12/focalLength)

def quaternion_to_euler(x, y, z, w):

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

print("The arguments are:")
print(str(sys.argv))

if len(sys.argv) >= 8:
    g_lookPos[0] = float(sys.argv[2])
    g_lookPos[1] = float(sys.argv[3])
    g_lookPos[2] = float(sys.argv[4])

    g_camRotation[0] = float(sys.argv[5])
    g_camRotation[1] = float(sys.argv[6])
    g_camRotation[2] = float(sys.argv[7])

if len(sys.argv) >= 9:
    g_yFovDeg = fovCalc(float(sys.argv[8]))
else:
    g_yFovDeg = fovCalc(32)

imgui.create_context()

magic.runProgram("Model renderer", 1280, 720, renderFrame, initResources)

 # create a blank texture atlas (PIL) (what is it's size?)
 textureWidth = 10
 textureHeight = 10
 textureAtlas = Image.new("RGBA", (textureWidth, textureHeight))
 texturePixels = textureAtlas.load()
 # open .ply file
 initResources()
 # open images.txt file
 txtLocation = sys.argv[1]
 with open(txtLocation, 'r') as fp:
    # for each line
    ln = fp.readLine()
    count = 1
    while ln:
        ln = ln.split()
        if ln[0] == str(count):
            count += 1
            #    quarternion -> x, y, z, rx, ry, rz
            rx, ry, rz = quarternion_to_euler(
                float(ln[1]),
                float(ln[2]),
                float(ln[3]),
                float(ln[4])
            )

            x, y, z = float(ln[5]), float(ln[6]), float(ln[7])
            # render model from camera pose
            uvMap = renderFrame()
            uvPixels = uvMap.load()
            # open camera image
            cameraImage = Image.open(ln[9]).load()
            # for pixel in image
            for i in range(uvPixels.size[0]):
                for j in range(uvPixels.size[1]):
                    if (uvPixels[i, j][3] == 0): continue
                    # average pixel into texture atlas
                    texturePixels[pixel[0], pixel[1]] = cameraPixels[i, j] # append values and average later!!!


            ...
        ln = fp.readLine()
 fp.close()
 # export texture atlas

