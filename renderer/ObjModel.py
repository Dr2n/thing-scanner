from OpenGL.GL import *
import os
from PIL import Image
from ctypes import sizeof, c_float, c_void_p, c_uint, string_at
import magic
import lab_utils as lu


def flatten(*lll):
	return [u for ll in lll for l in ll for u in l]



def bindTexture(texUnit, textureId, defaultTexture):
	glActiveTexture(GL_TEXTURE0 + texUnit);
	glBindTexture(GL_TEXTURE_2D, textureId if textureId != -1 else defaultTexture);




class ObjModel:
    RF_Transparent = 1
    RF_AlphaTested = 2
    RF_Opaque = 4
    RF_All = RF_Opaque | RF_AlphaTested | RF_Transparent

    AA_Position = 0
    AA_Normal = 1
    AA_TexCoord = 2
    AA_Tangent = 3
    AA_Bitangent = 4

    TU_Diffuse = 0
    TU_Opacity = 1
    TU_Specular = 2
    TU_Normal = 3
    TU_Max = 4


    def __init__(self, fileName):
        self.defaultTextureOne = glGenTextures(1);
        glBindTexture(GL_TEXTURE_2D, self.defaultTextureOne);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, 1, 1, 0, GL_RGBA, GL_FLOAT, [1.0, 1.0, 1.0, 1.0]);

        self.defaultNormalTexture = glGenTextures(1);
        glBindTexture(GL_TEXTURE_2D, self.defaultNormalTexture);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA32F, 1, 1, 0, GL_RGBA, GL_FLOAT, [0.5, 0.5, 0.5, 1.0]);
        glBindTexture(GL_TEXTURE_2D, 0);

        self.overrideDiffuseTextureWithDefault = False
        self.load(fileName)

        self.defaultShader = magic.buildShader(self.defaultVertexShader, self.defaultFragmentShader, self.getDefaultAttributeBindings())
        glUseProgram(self.defaultShader)
        self.setDefaultUniformBindings(self.defaultShader)
        glUseProgram(0)

    def load(self, fileName):
        basePath,_ = os.path.split(fileName)
        with open(fileName, "r") as inFile:
            self.loadObj(inFile.readlines(), basePath)

    def loadObj(self, objLines, basePath):
        positions = []
        normals = []
        uvs = []
        materialChunks = []
        materials = {}
        
        for l in objLines:
            #1 standardize line
            if len(l) > 0 and l[:1] != "#":
                tokens = l.split()
                if len(tokens):
                    if tokens[0] == "mtllib":
                        assert len(tokens) >= 2
                        materialName = " ".join(tokens[1:])
                        materials = self.loadMaterials(os.path.join(basePath, materialName), basePath)
                    if tokens[0] == "usemtl":
                        assert len(tokens) >= 2
                        materialName = " ".join(tokens[1:])
                        if len(materialChunks) == 0 or materialChunks[-1][0]  != materialName:
                            materialChunks.append([materialName, []])
                    elif tokens[0] == "v":
                        assert len(tokens[1:]) >= 3
                        positions.append([float(v) for v in tokens[1:4]])
                    elif tokens[0] == "vn":
                        assert len(tokens[1:]) >= 3
                        normals.append([float(v) for v in tokens[1:4]])
                    elif tokens[0] == "vt":
                        assert len(tokens[1:]) >= 2
                        uvs.append([float(v) for v in tokens[1:3]])
                    elif tokens[0] == "f":
                        materialChunks[-1][1] += self.parseFace(tokens[1:])
        self.numVerts = 0
        for mc in materialChunks:
            self.numVerts += len(mc[1])
        
        self.positions = [None]*self.numVerts
        self.normals = [None]*self.numVerts
        self.uvs = [[0.0,0.0]]*self.numVerts
        self.tangents = [[0.0,1.0,0.0]]*self.numVerts
        self.bitangents = [[1.0,0.0,0.0]]*self.numVerts
        self.chunks = []

        start = 0
        end = 0

        for matId, tris in materialChunks:
            material = materials[matId]
            renderFlags = 0
            if material["alpha"] != 1.0:
                renderFlags |= self.RF_Transparent 
            elif material["texture"]["opacity"] != -1:
                renderFlags |= self.RF_AlphaTested
            else:
                renderFlags |= self.RF_Opaque
            start = end
            end = start + int(len(tris)/3)

            chunkOffset = start * 3
            chunkCount = len(tris)

            # De-index mesh and (TODO) compute tangent frame
            for k in range(0,len(tris),3):
                for j in [0,1,2]:
                    p = positions[tris[k + j][0]]
                    oo = chunkOffset + k + j
                    self.positions[oo]= p
                    if tris[k + j][1] != -1:
                        self.uvs[oo] = uvs[tris[k + j][1]]
                    self.normals[oo] = normals[tris[k + j][2]]
            self.chunks.append((material, chunkOffset, chunkCount, renderFlags))
        
        self.vertexArrayObject = glGenVertexArrays(1)
        glBindVertexArray(self.vertexArrayObject)

        def createBindVertexAttribArrayFloat(data, attribLoc):
            bufId = glGenBuffers(1)
            glBindBuffer(GL_ARRAY_BUFFER, bufId)
            flatData = flatten(data)
            data_buffer = (c_float * len(flatData))(*flatData)
            glBufferData(GL_ARRAY_BUFFER, data_buffer, GL_STATIC_DRAW)
            glVertexAttribPointer(attribLoc, int(len(flatData) / len(data)), GL_FLOAT, GL_FALSE, 0, None)
            glEnableVertexAttribArray(attribLoc)
            return bufId

        self.positionBuffer = createBindVertexAttribArrayFloat(self.positions, self.AA_Position)
        self.normalBuffer = createBindVertexAttribArrayFloat(self.normals, self.AA_Normal)
        self.uvBuffer = createBindVertexAttribArrayFloat(self.uvs, self.AA_TexCoord)
        self.tangentBuffer = createBindVertexAttribArrayFloat(self.tangents, self.AA_Tangent)
        self.biTangentBuffer = createBindVertexAttribArrayFloat(self.bitangents, self.AA_Bitangent)

        glBindBuffer(GL_ARRAY_BUFFER, 0)
        glBindVertexArray(0)


    def parseFloats(self, tokens, minNum):
        assert len(tokens) >= minNum
        return [float(v) for v in tokens[0:minNum]]

    def parseFaceIndexSet(self, s):
        inds = s.split('/')
        assert len(inds) == 3
        return [int(ind) - 1 if ind != '' else -1 for ind in inds]

    def parseFace(self, tokens):
        assert len(tokens) >= 3
        result = []
        v0 = self.parseFaceIndexSet(tokens[0])
        v1 = self.parseFaceIndexSet(tokens[1])
        for t in tokens[2:]:
            v2 = self.parseFaceIndexSet(t)
            result += [v0, v1, v2]
            v1 = v2
        return result

    def loadMaterials(self, materialFileName, basePath):
        materials = {}
        with open(materialFileName, "r") as inFile:
            currentMaterial = ""
            for l in inFile.readlines():
                tokens = l.split()
                if len(tokens):
                    if tokens[0] == "newmtl":
                        assert len(tokens) >= 2
                        currentMaterial = " ".join(tokens[1:])
                        materials[currentMaterial] = {
                            "color" : {
                                "diffuse"  : [ 0.5, 0.5, 0.5 ],
                                "ambient"  : [ 0.5, 0.5, 0.5 ],
                                "specular" : [ 0.5, 0.5, 0.5 ],
                                "emissive" : [ 0.0, 0.0, 0.0 ]
                            },
                            "texture" : {
                                "diffuse" : -1,
                                "opacity" : -1,
                                "specular" : -1,
                                "normal" : -1,
                            },
                            "alpha" : 1.0,
                            "specularExponent" : 22.0,
                            "offset" : 0,
                        }
                    elif tokens[0] == "Ka":
                        materials[currentMaterial]["color"]["ambient"] = self.parseFloats(tokens[1:], 3)
                    elif tokens[0] == "Ns":
                        materials[currentMaterial]["specularExponent"] = float(tokens[1])
                    elif tokens[0] == "Kd":
                        materials[currentMaterial]["color"]["diffuse"] = self.parseFloats(tokens[1:], 3)
                    elif tokens[0] == "Ks":
                        materials[currentMaterial]["color"]["specular"] = self.parseFloats(tokens[1:], 3)
                    elif tokens[0] == "Ke":
                        materials[currentMaterial]["color"]["emissive"] = self.parseFloats(tokens[1:], 3)
                    elif tokens[0] == "map_Kd":
                        materials[currentMaterial]["texture"]["diffuse"] = self.loadTexture(" ".join(tokens[1:]), basePath, True)
                    elif tokens[0] == "map_Ks":
                        materials[currentMaterial]["texture"]["specular"] = self.loadTexture(" ".join(tokens[1:]), basePath, True)
                    elif tokens[0] == "map_bump" or tokens[0] == "bump":
                        materials[currentMaterial]["texture"]["normal"] = self.loadTexture(" ".join(tokens[1:]), basePath, False)
                    elif tokens[0] == "map_d":
                        materials[currentMaterial]["texture"]["opacity"] = self.loadTexture(" ".join(tokens[1:]), basePath, False)
                    elif tokens[0] == "d":
                        materials[currentMaterial]["alpha"] = float(tokens[1])

        # check of there is a colour texture but the coour is zero and then change it to 1, Maya exporter does this to us...
        for id,m in materials.items():
            for ch in ["diffuse", "specular"]:
                if m["texture"][ch] != -1 and sum(m["color"][ch]) == 0.0:
                    m["color"][ch] = [1,1,1]
                if m["texture"][ch] != -1 and sum(m["color"][ch]) == 0.0:
                    m["color"][ch] = [1,1,1]
        return materials

    def loadTexture(self, fileName, basePath, srgb):
        fullFileName = os.path.join(basePath, fileName)

        width = 0;
        height = 0;
        channels = 0;
        try:
            im = Image.open(fullFileName)
            texId = glGenTextures(1)
            glActiveTexture(GL_TEXTURE0)
            glBindTexture(GL_TEXTURE_2D, texId)

            # NOTE: srgb is used to store pretty much all texture image data (except HDR images, which we don't support)
            # Thus we use the GL_SRGB_ALPHA to ensure they are correctly converted to linear space when loaded into the shader.
            # However: normal/bump maps/alpha masks, are typically authored in linear space, and so should not be stored as SRGB texture format.
            data = im.tobytes("raw", "RGBX" if im.mode == 'RGB' else "RGBA", 0, -1)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_SRGB_ALPHA if srgb else GL_RGBA, im.size[0], im.size[1], 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
            #print("    Loaded texture '%s' (%d x %d)"%(fileName, im.size[0], im.size[1]));
            glGenerateMipmap(GL_TEXTURE_2D);

            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
            glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
            #glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, 16);
            glBindTexture(GL_TEXTURE_2D, 0);
            return texId
        except:
            print("WARNING: FAILED to load texture '%s'"%fileName);
            #print("Could not load image :(")

        return -1;



    def render(self, shaderProgram = None, renderFlags = None, transforms = {}):
        if not renderFlags:
            renderFlags = self.RF_All

        if not shaderProgram:
            shaderProgram = self.defaultShader

        # Filter chunks based of render flags
        chunks = [ch for ch in self.chunks if ch[3] & renderFlags]

        glBindVertexArray(self.vertexArrayObject)
        glUseProgram(shaderProgram)

        # define defaults (identity)
        defaultTfms = {
            "modelToClipTransform" : lu.Mat4(),
            "modelToViewTransform" : lu.Mat4(),
            "modelToViewNormalTransform" : lu.Mat3(),
        }
        # overwrite defaults
        defaultTfms.update(transforms)
        # upload map of transforms
        for tfmName,tfm in defaultTfms.items():
            loc = magic.getUniformLocationDebug(shaderProgram, tfmName)
            tfm._set_open_gl_uniform(loc);

        previousMaterial = None
        for material, chunkOffset, chunkCount, renderFlags in chunks:
            # as an optimization we only do this if the material has changed between chunks.
            # for more efficiency still consider sorting chunks based on material (or fusing them?)
            if material != previousMaterial:
                previousMaterial = material
                if self.overrideDiffuseTextureWithDefault:
                    bindTexture(self.TU_Diffuse, self.defaultTextureOne, self.defaultTextureOne);
                else:
                    bindTexture(self.TU_Diffuse, material["texture"]["diffuse"], self.defaultTextureOne);
                bindTexture(self.TU_Opacity, material["texture"]["opacity"], self.defaultTextureOne);
                bindTexture(self.TU_Specular, material["texture"]["specular"], self.defaultTextureOne);
                bindTexture(self.TU_Normal, material["texture"]["normal"], self.defaultNormalTexture);
                # TODO: can I do uniform buffers from python (yes, I need to use that struct thingo!)
                #uint32_t matUniformSize = sizeof(MaterialProperties_Std140);
                #glBindBufferRange(GL_UNIFORM_BUFFER, UBS_MaterialProperties, m_materialPropertiesBuffer, (uint32_t)chunk.material->offset * matUniformSize, matUniformSize);
                # TODO: this is very slow, it should be packed into an uniform buffer as per above!
                for k,v in material["color"].items():
                    glUniform3fv(magic.getUniformLocationDebug(shaderProgram, "material_%s_color"%k), 1, v)
                glUniform1f(magic.getUniformLocationDebug(shaderProgram, "material_specular_exponent"), material["specularExponent"])
                glUniform1f(magic.getUniformLocationDebug(shaderProgram, "material_alpha"), material["alpha"])
    
            glDrawArrays(GL_TRIANGLES, chunkOffset, chunkCount)

        glUseProgram(0);
        # deactivate texture units...
        #for (int i = TU_Max - 1; i >= 0; --i)
        #{
	       # glActiveTexture(GL_TEXTURE0 + i);
	       # glBindTexture(GL_TEXTURE_2D, 0);
        #}

    # useful to get the default bindings that the ObjModel will use when rendering, use to set up own shaders
    # for example an optimized shadow shader perhaps?
    def getDefaultAttributeBindings(self):
        return {
            "positionAttribute" : self.AA_Position,
            "normalAttribute" : self.AA_Normal,
            "texCoordAttribute" : self.AA_TexCoord,
            "tangentAttribute" : self.AA_Tangent,
            "bitangentAttribute" : self.AA_Bitangent,
        }


	#
	# Helper to set the default uniforms provided by ObjModel. This only needs to be done once after creating the shader
	# NOTE: the shader must be bound when calling this function.
    def setDefaultUniformBindings(self, shaderProgram):
        assert glGetIntegerv(GL_CURRENT_PROGRAM) == shaderProgram

        glUniform1i(magic.getUniformLocationDebug(shaderProgram, "diffuse_texture"), self.TU_Diffuse);
        glUniform1i(magic.getUniformLocationDebug(shaderProgram, "opacity_texture"), self.TU_Opacity);
        glUniform1i(magic.getUniformLocationDebug(shaderProgram, "specular_texture"), self.TU_Specular);
        glUniform1i(magic.getUniformLocationDebug(shaderProgram, "normal_texture"), self.TU_Normal);
        #glUniformBlockBinding(shaderProgram, glGetUniformBlockIndex(shaderProgram, "MaterialProperties"), UBS_MaterialProperties);

    defaultVertexShader = """
#version 330

in vec3 positionAttribute;
in vec3	normalAttribute;
in vec2	texCoordAttribute;

uniform mat4 modelToClipTransform;
uniform mat4 modelToViewTransform;
uniform mat3 modelToViewNormalTransform;

// Out variables decalred in a vertex shader can be accessed in the subsequent stages.
// For a pixel shader the variable is interpolated (the type of interpolation can be modified, try placing 'flat' in front, and also in the fragment shader!).
out VertexData
{
	vec3 v2f_viewSpaceNormal;
	vec2 v2f_texCoord;
};

void main() 
{
	// gl_Position is a buit in out variable that gets passed on to the clipping and rasterization stages.
  // it must be written in order to produce any drawn geometry. 
  // We transform the position using one matrix multiply from model to clip space, note the added 1 at the end of the position.
	gl_Position = modelToClipTransform * vec4(positionAttribute, 1.0);
	// We transform the normal to view space using the normal transform (which is the inverse-transpose of the rotation part of the modelToViewTransform)
  // Just using the rotation is only valid if the matrix contains only rotation and uniform scaling.
	v2f_viewSpaceNormal = normalize(modelToViewNormalTransform * normalAttribute);
	// The texture coordinate is just passed through
	v2f_texCoord = texCoordAttribute;
}
"""

    defaultFragmentShader = """
#version 330

// Input from the vertex shader, will contain the interpolated (i.e., distance weighted average) vaule out put for each of the three vertex shaders that 
// produced the vertex data for the triangle this fragmet is part of.
in VertexData
{
	vec3 v2f_viewSpaceNormal;
	vec2 v2f_texCoord;
};

// Material properties uniform buffer, required by OBJModel.
// 'MaterialProperties' must be bound to a uniform buffer, OBJModel::setDefaultUniformBindings is of help!
//layout(std140) uniform MaterialProperties
//{
uniform vec3 material_diffuse_color; 
uniform float material_alpha;
uniform vec3 material_specular_color; 
uniform vec3 material_emissive_color; 
uniform float material_specular_exponent;
//};
// Textures set by OBJModel (names must be bound to the right texture unit, OBJModel::setDefaultUniformBindings helps with that.
uniform sampler2D diffuse_texture;
uniform sampler2D opacity_texture;
uniform sampler2D specular_texture;
uniform sampler2D normal_texture;

// Other uniforms used by the shader
uniform vec3 viewSpaceLightDirection;

out vec4 fragmentColor;

// If we do not convert the colour to srgb before writing it out it looks terrible! All our lighting is done in linear space
// (which it should be!), and the frame buffer is srgb by default. So we must convert, or somehow create a linear frame buffer...
vec3 toSrgb(vec3 color)
{
  return pow(color, vec3(1.0 / 2.2));
}

void main() 
{
	fragmentColor = vec4(v2f_texCoord[0], v2f_texCoord[1], 0., 1.);
    // Manual alpha test (note: alpha test is no longer part of Opengl 3.3).
	// if (texture(opacity_texture, v2f_texCoord).r < 0.5)
	// {
	// 	discard;
	// } 
	// vec3 materialDiffuse = texture(diffuse_texture, v2f_texCoord).xyz * material_diffuse_color;
	// vec3 color = materialDiffuse * (0.1 + 0.9 * max(0.0, dot(v2f_viewSpaceNormal, -viewSpaceLightDirection))) + material_emissive_color;
	// fragmentColor = vec4(toSrgb(color), material_alpha);
}
"""
