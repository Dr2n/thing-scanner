from OpenGL.GL import *
import glfw

import numpy as np
from ctypes import sizeof, c_float, c_void_p, c_uint, string_at
import math
import sys
import os

import imgui

# we use 'warnings' to remove this warning that ImGui[glfw] gives
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)
from imgui.integrations.glfw import GlfwRenderer as ImGuiGlfwRenderer

from lab_utils import Mat3, Mat4, make_translation, normalize

from ObjModel import ObjModel
import lab_utils as lu

g_mousePos = [0.0, 0.0]

VAL_Position = 0
g_vertexDataBuffer = 0
g_vertexArrayObject = 0
g_simpleShader = 0

g_screenCaptureTexture = None

g_coordinateSystemModel = None

def getShaderInfoLog(obj):
    logLength = glGetShaderiv(obj, GL_INFO_LOG_LENGTH)

    if logLength > 0:
        return glGetShaderInfoLog(obj).decode()

    return ""



#
# This function performs the steps needed to compile the source code for a
# shader stage (e.g., vertex / fragment) and attach it to a shader program object.
#
def compileAndAttachShader(shaderProgram, shaderType, source):
    # Create the opengl shader object
    shader = glCreateShader(shaderType)
    # upload the source code for the shader
    # Note the function takes an array of source strings and lengths.
    glShaderSource(shader, [source])
    glCompileShader(shader)

    # If there is a syntax or other compiler error during shader compilation,
    # we'd like to know
    compileOk = glGetShaderiv(shader, GL_COMPILE_STATUS)

    if not compileOk:
        err = getShaderInfoLog(shader)
        print("SHADER COMPILE ERROR: '%s'" % err);
        return False

    glAttachShader(shaderProgram, shader)
    glDeleteShader(shader)
    return True


# creates a more general shader that binds a map of attribute streams 
# to the shader and the also any number of output shader variables
# The fragDataLocs can be left out for programs that don't use multiple render targets as 
# the default for any output variable is zero.
def buildShader(vertexShaderSource, fragmentShaderSource, attribLocs, fragDataLocs = {}):
    shader = glCreateProgram()

    if compileAndAttachShader(shader, GL_VERTEX_SHADER, vertexShaderSource) and compileAndAttachShader(shader, GL_FRAGMENT_SHADER, fragmentShaderSource):
	    # Link the attribute names we used in the vertex shader to the integer index
        for name, loc in attribLocs.items():
            glBindAttribLocation(shader, loc, name)

	    # If we have multiple images bound as render targets, we need to specify which
	    # 'out' variable in the fragment shader goes where in this case it is totally redundant 
        # as we only have one (the default render target, or frame buffer) and the default binding is always zero.
        for name, loc in fragDataLocs.items():
            glBindFragDataLocation(shader, loc, name)

        # once the bindings are done we can link the program stages to get a complete shader pipeline.
        # this can yield errors, for example if the vertex and fragment shaders don't have compatible out and in 
        # variables (e.g., the fragment shader expects some data that the vertex shader is not outputting).
        glLinkProgram(shader)
        linkStatus = glGetProgramiv(shader, GL_LINK_STATUS)
        if not linkStatus:
            err = glGetProgramInfoLog(shader)
            print("SHADER LINKER ERROR: '%s'" % err)
            sys.exit(1)
    return shader


def debugMessageCallback(source, type, id, severity, length, message, userParam):
    print(message)


# creates a basic shader that binds the 0th attribute stream to the shader attribute "positionIn" and the output shader variable 'fragmentColor' to the 0th render target (the default)
def buildBasicShader(vertexShaderSource, fragmentShaderSource):
    shader = glCreateProgram()

    if compileAndAttachShader(shader, GL_VERTEX_SHADER, vertexShaderSource) and compileAndAttachShader(shader, GL_FRAGMENT_SHADER, fragmentShaderSource):
	    # Link the name we used in the vertex shader 'positionIn' to the integer index 0
	    # This ensures that when the shader executes, data fed into 'positionIn' will
	    # be sourced from the 0'th generic attribute stream
	    # This seemingly backwards way of telling the shader where to look allows
	    # OpenGL programs to swap vertex buffers without needing to do any string lookups at run time.
        glBindAttribLocation(shader, 0, "positionIn")

	    # If we have multiple images bound as render targets, we need to specify which
	    # 'out' variable in the fragment shader goes where in this case it is totally redundant 
        # as we only have one (the default render target, or frame buffer) and the default binding is always zero.
        glBindFragDataLocation(shader, 0, "fragmentColor")

        # once the bindings are done we can link the program stages to get a complete shader pipeline.
        # this can yield errors, for example if the vertex and fragment shaders don't have compatible out and in 
        # variables (e.g., the fragment shader expects some data that the vertex shader is not outputting).
        glLinkProgram(shader)
        linkStatus = glGetProgramiv(shader, GL_LINK_STATUS)
        if not linkStatus:
            err = glGetProgramInfoLog(shader)
            print("SHADER LINKER ERROR: '%s'" % err)
            sys.exit(1)
    return shader



# Turns a multidimensional array (up to 3d?) into a 1D array
def flatten(*lll):
	return [u for ll in lll for l in ll for u in l]



def uploadFloatData(bufferObject, floatData):
    flatData = flatten(floatData)
    data_buffer = (c_float * len(flatData))(*flatData)
    # Upload data to the currently bound GL_ARRAY_BUFFER, note that this is
    # completely anonymous binary data, no type information is retained (we'll
    # supply that later in glVertexAttribPointer)
    glBindBuffer(GL_ARRAY_BUFFER, bufferObject)
    glBufferData(GL_ARRAY_BUFFER, data_buffer, GL_STATIC_DRAW)



def createVertexArrayObject(vertexPositions):
    #GLuint &positionBuffer, GLuint &vertexArrayObject
    # glGen*(<count>, <array of GLuint>) is the typical pattern for creating
    # objects in OpenGL.  Do pay attention to this idiosyncrasy as the first
    # parameter indicates the
    # number of objects we want created.  Usually this is just one, but if you
    # were to change the below code to '2' OpenGL would happily overwrite
    # whatever is after
    # 'positionBuffer' on the stack (this leads to nasty bugs that are
    # sometimes very hard to detect - i.e., this was a poor design choice!)
    positionBuffer = glGenBuffers(1)
    glBindBuffer(GL_ARRAY_BUFFER, positionBuffer)

    # re-package python data into something we can send to OpenGL
    flatData = flatten(vertexPositions)
    data_buffer = (c_float * len(flatData))(*flatData)
    # Upload data to the currently bound GL_ARRAY_BUFFER, note that this is
    # completely anonymous binary data, no type information is retained (we'll
    # supply that later in glVertexAttribPointer)
    glBufferData(GL_ARRAY_BUFFER, data_buffer, GL_STATIC_DRAW)

    vertexArrayObject = glGenVertexArrays(1)
    glBindVertexArray(vertexArrayObject)

    # The positionBuffer is already bound to the GL_ARRAY_BUFFER location.
    # This is typycal OpenGL style - you bind the buffer to GL_ARRAY_BUFFER,
    # and the vertex array object using 'glBindVertexArray', and then
    # glVertexAttribPointer implicitly uses both of these.  You often need to
    # read the manual
    # or find example code.
    # 'VAL_Position' is an integer, which tells it which array we want to
    # attach this data to, this must be the same that we set up our shader
    # using glBindAttribLocation.  Next provide we type information about the
    # data in the buffer: there are three components (x,y,z) per element
    # (position)
    # and they are of type 'float'.  The last arguments can be used to describe
    # the layout in more detail (stride & offset).
    # Note: The last adrgument is 'pointer' and has type 'const void *',
    # however, in modern OpenGL, the data ALWAYS comes from the current
    # GL_ARRAY_BUFFER object,
    # and 'pointer' is interpreted as an offset (which is somewhat clumsy).
    glVertexAttribPointer(VAL_Position, 3, GL_FLOAT, GL_FALSE, 0, None)
    # For the currently bound vertex array object, enable the VAL_Position'th
    # vertex array (otherwise the data is not fed to the shader)
    glEnableVertexAttribArray(VAL_Position)

    # Unbind the buffers again to avoid unintentianal GL state corruption (this
    # is something that can be rather inconventient to debug)
    glBindBuffer(GL_ARRAY_BUFFER, 0)
    glBindVertexArray(0)
    return (positionBuffer, vertexArrayObject)


def drawVertexDataAsTrianglesColour(triangleVerts, transform, colour):
    def setColour(shader):
        glUniform4fv(glGetUniformLocation(shader, "colour"), 1, colour);

    drawVertexDataAsTrianglesWithShaders(triangleVerts, transform, """
    #version 330
    in vec3 positionIn;
    uniform mat4 transformationMatrix;
    void main() 
    {
	    gl_Position = transformationMatrix * vec4(positionIn, 1.0);
    }
    """,
    """
    #version 330
    uniform vec4 colour;
    out vec4 fragmentColor;
    void main() 
    {
	    fragmentColor = vec4(colour);
    }
    """, setColour);

def drawVertexDataAsTrianglesDepth(triangleVerts, transform, colour):
    def setColour(shader):
        glUniform4fv(glGetUniformLocation(shader, "colour"), 1, colour);

    drawVertexDataAsTrianglesWithShaders(triangleVerts, transform, """
    #version 330
    in vec3 positionIn;
    uniform mat4 transformationMatrix;
    void main() 
    {
	    gl_Position = transformationMatrix * vec4(positionIn, 1.0);
    }
    """,
    """
    #version 330
    uniform vec4 colour;
    out vec4 fragmentColor;
    void main() 
    {
	    //fragmentColor = vec4(vec3(gl_FragCoord.z), colour.w);
	    fragmentColor = vec4(mix(vec3(1.0), vec3(colour.xyz), 0.05) * gl_FragCoord.z, colour.w);
    }
    """, setColour);

def drawVertexDataAsTriangles(triangleVerts, transform):
    drawVertexDataAsTrianglesWithShaders(triangleVerts, transform, """
    #version 330
    in vec3 positionIn;
    uniform mat4 transformationMatrix;
    out vec3 v2f_worldSpacePosition;
    void main() 
    {
        v2f_worldSpacePosition = positionIn;
	    gl_Position = transformationMatrix * vec4(positionIn, 1.0);
    }
    """,
    """
    #version 330
    in  vec3 v2f_worldSpacePosition;
    out vec4 fragmentColor;
    void main() 
    {
	    fragmentColor = vec4(v2f_worldSpacePosition, 1.0);
    }
    """);

def drawVertexDataAsTriangles_part3(triangleVerts, transform, patternThreshold):
    def setUniforms(shader):
        glUniform1f(glGetUniformLocation(shader, "patternThreshold"), patternThreshold);
    drawVertexDataAsTrianglesWithShaders(triangleVerts, transform, """
    #version 330
    in vec3 positionIn;
    uniform mat4 transformationMatrix;
    out vec3 v2f_worldSpacePosition;
    void main() 
    {
        v2f_worldSpacePosition = positionIn;
	    gl_Position = transformationMatrix * vec4(positionIn, 1.0);
    }
    """,
    """
    #version 330
    in  vec3 v2f_worldSpacePosition;
    uniform float patternThreshold;
    out vec4 fragmentColor;
    void main() 
    {
        if (patternThreshold >= 1.0)
        {
            fragmentColor = vec4(1.0);
            return;
        }
        vec2 uv = abs(2.0 * mod(v2f_worldSpacePosition.xy * 10.0, 1.0) - 1.0);
        if (uv.x > patternThreshold || uv.y > patternThreshold)
    	    fragmentColor = vec4(0.0, 1.0, 0.0, 1.0);
        else
    	    fragmentColor = vec4(1.0, 0.0, 0.0, 1.0);
    }
    """, setUniforms);

g_userShader = 0
g_vertexShaderSourceCode = ""
g_fragmentShaderSourceCode = ""

def drawVertexDataAsTrianglesWithShaders(triangleVerts, tfm, vertexShaderSourceCode, fragmentShaderSourceCode, setUniforms = None):
    global g_userShader
    global g_vertexArrayObject
    global g_vertexDataBuffer
    global g_vertexShaderSourceCode
    global g_fragmentShaderSourceCode 

    # We re-compile and create the shader program if the source code has changed, this is useful for debugging,
    # for example to re-load shaders from files without re-starting, but care must be taken that all uniforms etc are set!
    if g_vertexShaderSourceCode != vertexShaderSourceCode or g_fragmentShaderSourceCode != fragmentShaderSourceCode or not g_userShader:
        if g_userShader != 0:
            glDeleteProgram(g_userShader)
        g_vertexShaderSourceCode = vertexShaderSourceCode
        g_userShader = buildBasicShader(vertexShaderSourceCode,fragmentShaderSourceCode)

    # Fetch the location of the uniform from the shader program object - usually this is done once and 
    # then the integer index can be used to set the value of the uniform in the rendering loop, here we
    # re-do this every iteration for illustration.
    # https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glGetUniformLocation.xhtml
    transformUniformIndex = glGetUniformLocation(g_userShader, "transformationMatrix")

    # Bind ('use') current shader program
    # NOTE: glGetUniformLocation takes the shader program as an argument, so it does not require the shader to be bound.
    #       However, glUniformMatrix4fv DOES NOT take the shader program as an argument and therefore it works on the one
    #       that is currenly bound using glUseProgram. This is easy to mess up, so be careful!
    glUseProgram(g_userShader)
    
    if setUniforms:
        setUniforms(g_userShader)

    # Now we set the uniform to the value of the transform matrix, there are functions for different types.
    # NOTE: the third argument tells OpenGL that it should transpose the matrix, this is because numpy
    # stores the data in row-major order and OpenGL expects it in column major order. This is one of those 
    # pesky details that is easy to get wrong and somewhat hard to debug.
    # The 'ascontiguousarray' is also needed as the OpenGL wrapper does not handle the numpy.matrix data type
    # https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glUniform.xhtml
    glUniformMatrix4fv(transformUniformIndex, 1, GL_TRUE, tfm.getData())

    # as before pipe up the vertex data.
    uploadFloatData(g_vertexDataBuffer, triangleVerts)

    # Bind gl object storing the shphere mesh data (it is set up in main)
    # https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glBindVertexArray.xhtml
    glBindVertexArray(g_vertexArrayObject)

    # Tell OpenGL to draw triangles using data from the currently bound vertex
    # array object by grabbing three at a time vertices from the array up to 
    # len(triangleVerts) vertices, something like (but in hardware)
    # for(i in range(0, len(triangleVerts), 3): ...  draw triangle ...
    # https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glDrawArrays.xhtml
    glDrawArrays(GL_TRIANGLES, 0, len(triangleVerts))

    # Unbind the shader program & vertex array object to ensure it does not
    # affect anything else (in this simple program, no great risk, but
    # otherwise it pays to be careful)
    glBindVertexArray(0)
    glUseProgram(0)


def beginImGuiHud():
    global g_mousePos
    #imgui.set_next_window_position([1.0,1.0], imgui.ALWAYS, [float(imgui.get_io().display_size.x) - 5.0, 5.0])
    #imgui.set_next_window_position([1.0,1.0], imgui.ALWAYS, [float(imgui.get_io().display_size.x) - 5.0, 5.0])
    #imgui.set_next_window_position(1.0,1.0)
    imgui.set_next_window_position(5.0, 5.0)
    #imgui.setpi
    #imgui.set_next_window_size(float(imgui.get_io().display_size.x) - 5.0, 5.0)
#, imgui.ALWAYS, [])

    #imgui.SetNextWindowBgAlpha(0.5f); // Transparent background
    if imgui.begin("UI", 0, imgui.WINDOW_NO_RESIZE | imgui.WINDOW_ALWAYS_AUTO_RESIZE  | imgui.WINDOW_NO_SAVED_SETTINGS | imgui.WINDOW_NO_FOCUS_ON_APPEARING):
        pass#imgui.text("Mouse Position: {%0.3f, %0.3f}" %(g_mousePos[0], g_mousePos[1]))


def endImGuiHud():
        imgui.end()

def drawMagnifiedRegion(lowerLeft, size, magFactor):
    global g_screenCaptureTexture
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    pixel_data = glReadPixels(lowerLeft[0], lowerLeft[1], size[0], size[1], GL_BGR, GL_UNSIGNED_BYTE);

    if not g_screenCaptureTexture:
        g_screenCaptureTexture = glGenTextures(1)
        glBindTexture(GL_TEXTURE_2D, g_screenCaptureTexture)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST)
    glBindTexture(GL_TEXTURE_2D, g_screenCaptureTexture)
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, size[0], size[1], 0, GL_BGR, GL_UNSIGNED_BYTE, pixel_data);
    glBindTexture(GL_TEXTURE_2D, 0)

    imgui.image(g_screenCaptureTexture, size[0]*magFactor, size[1]*magFactor, (0,1), (1,0), (1,1,1,1), (1,1,1,1) )

#
def drawObjModel(viewToClipTfm, worldToViewTfm, modelToWorldTfm, model):
    # Lighting/Shading is very often done in view space, which is why a transformation that lands positions in this space is needed
    modelToViewTransform = worldToViewTfm * modelToWorldTfm
    
    # this is a special transform that ensures that normal vectors remain orthogonal to the 
    # surface they are supposed to be even in the prescence of non-uniform scaling.
    # It is a 3x3 matrix as vectors don't need translation anyway and this transform is only for vectors,
    # not points. If there is no non-uniform scaling this is just the same as Mat3(modelToViewTransform)
    modelToViewNormalTransform = lu.inverse(lu.transpose(lu.Mat3(modelToViewTransform)));

    # Bind the shader program such that we can set the uniforms (model.render sets it again)
    glUseProgram(model.defaultShader)

    # transform (rotate) light direction into view space (as this is what the ObjModel shader wants)
    viewSpaceLightDirection = [0.0, 0.0, -1.0]
    glUniform3fv(glGetUniformLocation(model.defaultShader, "viewSpaceLightDirection"), 1, viewSpaceLightDirection);

    # This dictionary contains a few transforms that are needed to render the ObjModel using the default shader.
    # it would be possible to just set the modelToWorld transform, as this is the only thing that changes between
    # the objects, and compute the other matrices in the vertex shader.
    # However, this would push a lot of redundant computation to the vertex shader and makes the code less self contained,
    # in this way we set all the required parameters explicitly.
    transforms = {
        "modelToClipTransform" : viewToClipTfm * worldToViewTfm * modelToWorldTfm,
        "modelToViewTransform" : modelToViewTransform,
        "modelToViewNormalTransform" : modelToViewNormalTransform,
    }
    
    model.render(None, None, transforms)


def drawCoordinateSystem(viewToClipTfm, worldToViewTfm):
    global g_coordinateSystemModel

    glUseProgram(g_coordinateSystemModel.defaultShader)
    viewSpaceLightDirection = [0.0, 0.0, -1.0]
    glUniform3fv(glGetUniformLocation(g_coordinateSystemModel.defaultShader, "viewSpaceLightDirection"), 1, viewSpaceLightDirection);

    transforms = {
        "modelToClipTransform" : viewToClipTfm * worldToViewTfm,
        "modelToViewTransform" : worldToViewTfm,
        "modelToViewNormalTransform" : Mat3(worldToViewTfm),
    }
    
    g_coordinateSystemModel.render(None, None, transforms)

g_numMsaaSamples = 8
g_currentMsaaSamples = 1

def initGlFwAndResources(title, startWidth, startHeight, initResources):
    global g_simpleShader
    global g_vertexArrayObject
    global g_vertexDataBuffer
    global g_mousePos
    global g_coordinateSystemModel
    global g_numMsaaSamples 
    global g_currentMsaaSamples

    #glfw.window_hint(glfw.OPENGL_DEBUG_CONTEXT, 1)
    glfw.window_hint(glfw.CONTEXT_VERSION_MAJOR, 3)
    glfw.window_hint(glfw.CONTEXT_VERSION_MINOR, 3)
    glfw.window_hint(glfw.OPENGL_PROFILE, glfw.OPENGL_CORE_PROFILE)
    glfw.window_hint(glfw.OPENGL_FORWARD_COMPAT, GL_TRUE)

    #glfw.window_hint(glfw.SAMPLES, g_currentMsaaSamples)


    window = glfw.create_window(startWidth, startHeight, title, None, None)
    if not window:
        glfw.terminate()
        sys.exit(1)

    glfw.make_context_current(window)

    print("--------------------------------------\nOpenGL\n  Vendor: %s\n  Renderer: %s\n  Version: %s\n--------------------------------------\n" % (glGetString(GL_VENDOR).decode("utf8"), glGetString(GL_RENDERER).decode("utf8"), glGetString(GL_VERSION).decode("utf8")), flush=True)

    impl = ImGuiGlfwRenderer(window)

    #glDebugMessageCallback(GLDEBUGPROC(debugMessageCallback), None)

    # (although this glEnable(GL_DEBUG_OUTPUT) should not have been needed when
    # using the GLUT_DEBUG flag above...)
    #glEnable(GL_DEBUG_OUTPUT)
    # This ensures that the callback is done in the context of the calling
    # function, which means it will be on the stack in the debugger, which makes it
    # a lot easier to figure out why it happened.
    #glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS)

    g_simpleShader = buildBasicShader(
    """
    #version 330
    in vec3 positionIn;

    void main() 
    {
	    gl_Position = vec4(positionIn, 1.0);
    }
    """,
    """
    #version 330
    out vec4 fragmentColor;

    void main() 
    {
	    fragmentColor = vec4(1.0);
    }
    """);

    # Create Vertex array object and buffer with dummy data for now, we'll fill it later when rendering the frame
    (g_vertexDataBuffer, g_vertexArrayObject) = createVertexArrayObject([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])

    glDisable(GL_CULL_FACE)
    glEnable(GL_DEPTH_TEST)
    glDepthFunc(GL_LEQUAL)
    #glEnable(GL_DEPTH_CLAMP)

    if initResources:
        initResources()
    
    script_dir = os.path.dirname(__file__)
    modelPath = "./data/coordinate_system.obj"
    g_coordinateSystemModel =  ObjModel(os.path.join(script_dir, modelPath))

    return window,impl

# add anything here that must be deleted when creating a new context
def cleaupGlResources():
    global g_userShader
    global g_screenCaptureTexture

    if g_userShader:
        glDeleteProgram(g_userShader)
        g_userShader = 0
    if g_screenCaptureTexture:
        glDeleteTextures(g_screenCaptureTexture)
        g_screenCaptureTexture = None

def runProgram(title, startWidth, startHeight, renderFrame, initResources = None, drawUi = None, update = None):
    global g_simpleShader
    global g_vertexArrayObject
    global g_vertexDataBuffer
    global g_mousePos
    global g_coordinateSystemModel
    global g_numMsaaSamples 
    global g_currentMsaaSamples

    if not glfw.init():
        sys.exit(1)

    window, impl = initGlFwAndResources(title, startWidth, startHeight, initResources)

    currentTime = glfw.get_time()
    prevMouseX,prevMouseY = glfw.get_cursor_pos(window)

    while not glfw.window_should_close(window):
        prevTime = currentTime
        currentTime = glfw.get_time()
        dt = currentTime - prevTime

        keyStateMap = {}
        for name,id in g_glfwKeymap.items():
            keyStateMap[name] = glfw.get_key(window, id) == glfw.PRESS

        for name,id in g_glfwMouseMap.items():
            keyStateMap[name] = glfw.get_mouse_button(window, id) == glfw.PRESS

        mouseX,mouseY = glfw.get_cursor_pos(window)
        g_mousePos = [mouseX,mouseY]

        # Udpate 'game logic'
        if update:
            imIo = imgui.get_io()
            mouseDelta = [mouseX - prevMouseX,mouseY - prevMouseY]
            if imIo.want_capture_mouse:
                mouseDelta = [0,0]
            update(dt, keyStateMap, mouseDelta)
        prevMouseX,prevMouseY = mouseX,mouseY

        width, height = glfw.get_framebuffer_size(window)

        imgui.new_frame()

        beginImGuiHud()

        if drawUi:
            drawUi(width, height)

        renderFrame(width, height)
    
        #drawCoordinateSystem()

        #mgui.show_test_window()

        endImGuiHud()
        imgui.render()
        # Swap front and back buffers
        glfw.swap_buffers(window)

        # Poll for and process events
        glfw.poll_events()
        impl.process_inputs()


    glfw.terminate()


# This function creates render buffers for a Frame Buffer Object (FBO) for rendering to. An FBO can be thought of as an 
# off-screen fulll-screen window. To store the pixels, and depth buffer samples we create and attach 'render buffers', 
# using glRenderbufferStorageMultisample we can set a number of samples per pixel for multisampling.
# glRenderbufferStorageMultisample: https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glRenderbufferStorageMultisample.xhtml
# It is also possible to use the contents of the buffers as textures, but then you must attach a texture to the framebuffer using
# glFramebufferTexture: https://www.khronos.org/registry/OpenGL-Refpages/gl4/html/glFramebufferTexture.xhtml
# In this lab, we create the off-screen FBO to enable multi-sampling AA (and more, to make it easy to play with the number of sampels at run-time)
# this is easiest using the render-buffer APIs. The call to 'glBlitFramebuffer' 'resolves' i.e, averages the multi-sampled image to
# the non-MSAA default frame buffer (Note that creating a MSAA frame buffer and blitting to it may cause problems).
def setupFbo(msaaFbo, fboWidth, fboHeight, numSamples, colorRenderBuffer = 0, depthRenderBuffer = 0):
    if not colorRenderBuffer:
        colorRenderBuffer, depthRenderBuffer = glGenRenderbuffers(2)
    glBindRenderbuffer(GL_RENDERBUFFER, colorRenderBuffer)
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, numSamples, GL_RGB8, fboWidth, fboHeight)
    glBindRenderbuffer(GL_RENDERBUFFER, depthRenderBuffer)
    glRenderbufferStorageMultisample(GL_RENDERBUFFER, numSamples, GL_DEPTH_COMPONENT32, fboWidth, fboHeight)
     # glBindRenderbuffer(GL_RENDERBUFFER, 0)

    glBindFramebuffer(GL_FRAMEBUFFER, msaaFbo)
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, colorRenderBuffer)
    glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, depthRenderBuffer)
    # glBindFramebuffer(GL_FRAMEBUFFER, 0)

    return colorRenderBuffer, depthRenderBuffer

def runProgramMsaa(title, startWidth, startHeight, numMsaaSamples, renderFrame, initResources = None, drawUi = None, update = None):
    global g_simpleShader
    global g_vertexArrayObject
    global g_vertexDataBuffer
    global g_mousePos
    global g_coordinateSystemModel
    global g_numMsaaSamples 
    global g_currentMsaaSamples

    if not glfw.init():
        sys.exit(1)

    window, impl = initGlFwAndResources(title, startWidth, startHeight, initResources)

    currentTime = glfw.get_time()
    prevMouseX,prevMouseY = glfw.get_cursor_pos(window)

    msaaFbo = glGenFramebuffers(1)
    # Lazy create  
    fboWidth = startWidth
    fboHeight = startHeight
    g_numMsaaSamples = numMsaaSamples
    g_currentMsaaSamples = numMsaaSamples

    colorRenderBuffer, depthRenderBuffer = setupFbo(msaaFbo, fboWidth, fboHeight, g_currentMsaaSamples)

    while not glfw.window_should_close(window):
        # Udpate 'game logic'
        prevTime = currentTime
        currentTime = glfw.get_time()
        dt = currentTime - prevTime

        keyStateMap = {}
        for name,id in g_glfwKeymap.items():
            keyStateMap[name] = glfw.get_key(window, id) == glfw.PRESS

        for name,id in g_glfwMouseMap.items():
            keyStateMap[name] = glfw.get_mouse_button(window, id) == glfw.PRESS

        mouseX,mouseY = glfw.get_cursor_pos(window)
        g_mousePos = [mouseX,mouseY]

        # Udpate 'game logic'
        if update:
            imIo = imgui.get_io()
            mouseDelta = [mouseX - prevMouseX,mouseY - prevMouseY]
            if imIo.want_capture_mouse:
                mouseDelta = [0,0]
            update(dt, keyStateMap, mouseDelta)
        prevMouseX,prevMouseY = mouseX,mouseY

        # Render here, e.g.  using pyOpenGL
        width, height = glfw.get_framebuffer_size(window)
    
        # in this case, re-create the FBO texture
        if fboWidth != width or  fboHeight != height or g_numMsaaSamples != g_currentMsaaSamples:
            fboWidth  = max(width, fboWidth)
            fboHeight = max(height, fboHeight)
            g_currentMsaaSamples = g_numMsaaSamples
            colorRenderBuffer, depthRenderBuffer = setupFbo(msaaFbo, fboWidth, fboHeight, g_currentMsaaSamples, colorRenderBuffer, depthRenderBuffer)

        imgui.new_frame()

        beginImGuiHud()

        # Note: here we bind the off-screen frame buffer before calling the program to draw
        # since we take care to make sure it is the same size as the window, the rendering code does not need to know.
        glBindFramebuffer(GL_FRAMEBUFFER, msaaFbo)
    
        renderFrame(width, height)
    
        #drawCoordinateSystem()

        #mgui.show_test_window()

        # reset the frame buffer binding to the default (i.e., window-visible one)
        glBindFramebuffer(GL_FRAMEBUFFER, 0)
        glBindFramebuffer(GL_READ_FRAMEBUFFER, msaaFbo)
        # Copy the data from the render buffers attached to the MSAA FBO (bound to 'GL_READ_FRAMEBUFFER') into the default FBO (bound to 'GL_FRAMEBUFFER')
        glBlitFramebuffer(0,0,width,height,0,0,width,height, GL_COLOR_BUFFER_BIT, GL_LINEAR)
        # Reset the 'GL_READ_FRAMEBUFFER' binding point, just in case...
        glBindFramebuffer(GL_READ_FRAMEBUFFER, 0)

        # we draw the UI after binding the default frame buffer and resolving so we can easily copy a portion to display the zoomed in
        # view (otherwise it might have made more sense to draw the UI with AA as well).
        if drawUi:
            drawUi(width, height)

        endImGuiHud()
        imgui.render()
        # Swap front and back buffers
        glfw.swap_buffers(window)

        # Poll for and process events
        glfw.poll_events()
        impl.process_inputs()

    glfw.terminate()


# The reason we need a 'look from', and don't just use lookAt(pos, pos+dir, up) is because if pos is large (i.e., far from the origin) and 'dir' is a unit vector (common case)
# then the precision loss in the addition followed by subtraction in lookAt to get the direction back is _significant_, and leads to jerky camera movements.
def make_lookFrom(eye, direction, up):
    f = normalize(direction)
    U = np.array(up[:3])
    s = normalize(np.cross(f, U))
    u = np.cross(s, f)
    M = np.matrix(np.identity(4))
    M[:3,:3] = np.vstack([s,u,-f])
    T = make_translation(-eye[0], -eye[1], -eye[2])
    return Mat4(M) * T


# make_lookAt defines a view transform, i.e., from world to view space, using intuitive parameters. location of camera, point to aim, and rough up direction.
# this is basically the same as what we saw in Lexcture #2 for placing the car in the world, except the inverse! (and also view-space 'forwards' is the negative z-axis)
def make_lookAt(eye, target, up):
    return make_lookFrom(eye, np.array(target[:3]) - np.array(eye[:3]), up)



def make_perspective(yFovDeg, aspect, n, f):
    radFovY = math.radians(yFovDeg)
    tanHalfFovY = math.tan(radFovY / 2.0)
    sx = 1.0 / (tanHalfFovY * aspect)
    sy = 1.0 / tanHalfFovY
    zz = -(f + n) / (f - n)
    zw = -(2.0 * f * n) / (f - n)

    return Mat4([[sx,0,0,0],
                 [0,sy,0,0],
                 [0,0,zz,zw],
                 [0,0,-1,0]])


def getUniformLocationDebug(shaderProgram, name):
    loc = glGetUniformLocation(shaderProgram, name)
    # Useful point for debugging, replace with silencable logging 
    #if loc == -1:
    #    print("Uniforn '%s' was not found"%name)
    return loc


g_glfwMouseMap = {
    "MOUSE_BUTTON_LEFT" : glfw.MOUSE_BUTTON_LEFT,
    "MOUSE_BUTTON_RIGHT" : glfw.MOUSE_BUTTON_RIGHT,
    "MOUSE_BUTTON_MIDDLE" : glfw.MOUSE_BUTTON_MIDDLE,
}


g_glfwKeymap = {
    "SPACE" : glfw.KEY_SPACE,
    "APOSTROPHE" : glfw.KEY_APOSTROPHE,
    "COMMA" : glfw.KEY_COMMA,
    "MINUS" : glfw.KEY_MINUS,
    "PERIOD" : glfw.KEY_PERIOD,
    "SLASH" : glfw.KEY_SLASH,
    "0" : glfw.KEY_0,
    "1" : glfw.KEY_1,
    "2" : glfw.KEY_2,
    "3" : glfw.KEY_3,
    "4" : glfw.KEY_4,
    "5" : glfw.KEY_5,
    "6" : glfw.KEY_6,
    "7" : glfw.KEY_7,
    "8" : glfw.KEY_8,
    "9" : glfw.KEY_9,
    "SEMICOLON" : glfw.KEY_SEMICOLON,
    "EQUAL" : glfw.KEY_EQUAL,
    "A" : glfw.KEY_A,
    "B" : glfw.KEY_B,
    "C" : glfw.KEY_C,
    "D" : glfw.KEY_D,
    "E" : glfw.KEY_E,
    "F" : glfw.KEY_F,
    "G" : glfw.KEY_G,
    "H" : glfw.KEY_H,
    "I" : glfw.KEY_I,
    "J" : glfw.KEY_J,
    "K" : glfw.KEY_K,
    "L" : glfw.KEY_L,
    "M" : glfw.KEY_M,
    "N" : glfw.KEY_N,
    "O" : glfw.KEY_O,
    "P" : glfw.KEY_P,
    "Q" : glfw.KEY_Q,
    "R" : glfw.KEY_R,
    "S" : glfw.KEY_S,
    "T" : glfw.KEY_T,
    "U" : glfw.KEY_U,
    "V" : glfw.KEY_V,
    "W" : glfw.KEY_W,
    "X" : glfw.KEY_X,
    "Y" : glfw.KEY_Y,
    "Z" : glfw.KEY_Z,
    "LEFT_BRACKET" : glfw.KEY_LEFT_BRACKET,
    "BACKSLASH" : glfw.KEY_BACKSLASH,
    "RIGHT_BRACKET" : glfw.KEY_RIGHT_BRACKET,
    "GRAVE_ACCENT" : glfw.KEY_GRAVE_ACCENT,
    "WORLD_1" : glfw.KEY_WORLD_1,
    "WORLD_2" : glfw.KEY_WORLD_2,
    "ESCAPE" : glfw.KEY_ESCAPE,
    "ENTER" : glfw.KEY_ENTER,
    "TAB" : glfw.KEY_TAB,
    "BACKSPACE" : glfw.KEY_BACKSPACE,
    "INSERT" : glfw.KEY_INSERT,
    "DELETE" : glfw.KEY_DELETE,
    "RIGHT" : glfw.KEY_RIGHT,
    "LEFT" : glfw.KEY_LEFT,
    "DOWN" : glfw.KEY_DOWN,
    "UP" : glfw.KEY_UP,
    "PAGE_UP" : glfw.KEY_PAGE_UP,
    "PAGE_DOWN" : glfw.KEY_PAGE_DOWN,
    "HOME" : glfw.KEY_HOME,
    "END" : glfw.KEY_END,
    "CAPS_LOCK" : glfw.KEY_CAPS_LOCK,
    "SCROLL_LOCK" : glfw.KEY_SCROLL_LOCK,
    "NUM_LOCK" : glfw.KEY_NUM_LOCK,
    "PRINT_SCREEN" : glfw.KEY_PRINT_SCREEN,
    "PAUSE" : glfw.KEY_PAUSE,
    "F1" : glfw.KEY_F1,
    "F2" : glfw.KEY_F2,
    "F3" : glfw.KEY_F3,
    "F4" : glfw.KEY_F4,
    "F5" : glfw.KEY_F5,
    "F6" : glfw.KEY_F6,
    "F7" : glfw.KEY_F7,
    "F8" : glfw.KEY_F8,
    "F9" : glfw.KEY_F9,
    "F10" : glfw.KEY_F10,
    "F11" : glfw.KEY_F11,
    "F12" : glfw.KEY_F12,
    "F13" : glfw.KEY_F13,
    "F14" : glfw.KEY_F14,
    "F15" : glfw.KEY_F15,
    "F16" : glfw.KEY_F16,
    "F17" : glfw.KEY_F17,
    "F18" : glfw.KEY_F18,
    "F19" : glfw.KEY_F19,
    "F20" : glfw.KEY_F20,
    "F21" : glfw.KEY_F21,
    "F22" : glfw.KEY_F22,
    "F23" : glfw.KEY_F23,
    "F24" : glfw.KEY_F24,
    "F25" : glfw.KEY_F25,
    "KP_0" : glfw.KEY_KP_0,
    "KP_1" : glfw.KEY_KP_1,
    "KP_2" : glfw.KEY_KP_2,
    "KP_3" : glfw.KEY_KP_3,
    "KP_4" : glfw.KEY_KP_4,
    "KP_5" : glfw.KEY_KP_5,
    "KP_6" : glfw.KEY_KP_6,
    "KP_7" : glfw.KEY_KP_7,
    "KP_8" : glfw.KEY_KP_8,
    "KP_9" : glfw.KEY_KP_9,
    "KP_DECIMAL" : glfw.KEY_KP_DECIMAL,
    "KP_DIVIDE" : glfw.KEY_KP_DIVIDE,
    "KP_MULTIPLY" : glfw.KEY_KP_MULTIPLY,
    "KP_SUBTRACT" : glfw.KEY_KP_SUBTRACT,
    "KP_ADD" : glfw.KEY_KP_ADD,
    "KP_ENTER" : glfw.KEY_KP_ENTER,
    "KP_EQUAL" : glfw.KEY_KP_EQUAL,
    "LEFT_SHIFT" : glfw.KEY_LEFT_SHIFT,
    "LEFT_CONTROL" : glfw.KEY_LEFT_CONTROL,
    "LEFT_ALT" : glfw.KEY_LEFT_ALT,
    "LEFT_SUPER" : glfw.KEY_LEFT_SUPER,
    "RIGHT_SHIFT" : glfw.KEY_RIGHT_SHIFT,
    "RIGHT_CONTROL" : glfw.KEY_RIGHT_CONTROL,
    "RIGHT_ALT" : glfw.KEY_RIGHT_ALT,
    "RIGHT_SUPER" : glfw.KEY_RIGHT_SUPER,
    "MENU" : glfw.KEY_MENU,
}
