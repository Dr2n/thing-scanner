
module.exports = function (videoName) {

    const config = require('./config.js')
    const { spawn } = require('child_process')

    return new Promise((resolve, reject) => {
        let uuid = videoName.split('.')[0]
        let extension = videoName.split('.')[1]
    
        console.log(`Processing video ${uuid}...`)
    
        // use ffmpeg to split the video into frame images
        ffmpegJob(videoName)
        // use colmap to do sparse reconstruction, output .ply file
        .then(sparseReconstruction)
        // export image pose data
        .then(exportImageData)
        // use colmap to export as .PLY
        .then(exportModel)
        // call poisson reconstruction program to create surface
        .then(poissonSurface)
        // use blender to add a UV map
        .then((poissonModel) => {
            resolve()
        }).catch(err => {
            console.log(err)
            reject(err)
        })
        
        
        //.then(blenderUVMap)
        // generate texture atlas
        //.then(generateTextureAtlas)
        // put created model into a folder
    })
    
    // Functions
    function ffmpegJob(videoName) {
        return new Promise((resolve, reject) => {
            const ffmpegJob = spawn('ffmpeg', [
                '-i', `./videos/${videoName}`, '-vf', 'select=not(mod(n\\,15))',
                '-vsync', 'vfr', '-q:v', '2', './frames/img_%03d.jpg'
            ])
        
            ffmpegJob.stdout.on('data', (data) => {
                console.log(`ffmpeg stdout:  ${data}`)
            })
        
            ffmpegJob.stderr.on('data', (data) => {
                console.log(`ffmpeg stderr:  ${data}`)
            })
        
            ffmpegJob.on('close', (exitCode) => {
                console.log(`ffmpeg closed with exit code ${exitCode}`)
                if (exitCode != 0) {
                    reject(`FFMPEG exited with error code: ${exitCode}`)
                } else {
                    console.log('Frame extraction completed successfully!')
                    resolve()
                }
            })
        })
    }
    
    function sparseReconstruction() {
        return new Promise((resolve, reject) => {
            const colmapJob = spawn(config["colmap-location"], [
                'automatic_reconstructor', 
                '--workspace_path', './colmap-workspace',
                '--image_path', './frames'
            ])

            console.log("Running sparse reconstruction")

            colmapJob.stdout.on('data', (data) => {
                console.log(data)
            })
        
            colmapJob.stderr.on('data', (data) => {
                console.log(`colmap stderr:  ${data}`)
            })
        
            colmapJob.on('close', (exitCode) => {
                console.log(`colmap closed with exit code ${exitCode}`)
                if (exitCode != 0) {
                    reject(`colmap exited with error code: ${exitCode}`)
                } else {
                    console.log('Sparse model reconstruction completed successfully!')
                    resolve()
                }
            })
        })
    }

    function exportImageData() {
        return new Promise((resolve, reject) => {
            const colmapJob = spawn(config["colmap-location"], [
                'model_converter', 
                '--input_path', './colmap-workspace/sparse/0',
                '--output_path', './colmap-workspace',
                '--output_type', 'TXT'
            ])

            colmapJob.stdout.on('data', (data) => {
                console.log(`image data export stdout:  ${data}`)
            })
        
            colmapJob.stderr.on('data', (data) => {
                console.log(`image data export stderr:  ${data}`)
            })
        
            colmapJob.on('close', (exitCode) => {
                console.log(`image data export closed with exit code ${exitCode}`)
                if (exitCode != 0) {
                    reject(`image data export exited with error code: ${exitCode}`)
                } else {
                    console.log('Image data export completed successfully!')
                    resolve()
                }
            })
        })
    }

    function exportModel() {
        // ../COLMAP.app/Contents/MacOS/colmap model_converter --input_path ./sparse/0 ./sparse --output_path ./camera.ply --output_type PLY
        return new Promise((resolve, reject) => {
            const colmapJob = spawn(config["colmap-location"], [
                'model_converter', 
                '--input_path', './colmap-workspace/sparse/0',
                '--output_path', `./models/${uuid}.ply`,
                '--output_type', 'PLY'
            ])

            colmapJob.stdout.on('data', (data) => {
                console.log(`colmap-converter stdout:  ${data}`)
            })
        
            colmapJob.stderr.on('data', (data) => {
                console.log(`colmap-converter stderr:  ${data}`)
            })
        
            colmapJob.on('close', (exitCode) => {
                console.log(`colmap-converter closed with exit code ${exitCode}`)
                if (exitCode != 0) {
                    reject(`colmap-converter exited with error code: ${exitCode}`)
                } else {
                    console.log('Model exported successfully!')
                    resolve()
                }
            })
        })
    }
    
    function poissonSurface() {
        return new Promise((resolve, reject) => {
            let outputPath = `./models/${uuid}-surface.ply` 
            const poissonJob = spawn(config["poisson-location"], [
                `./models/${uuid}.ply`,
                outputPath
            ])

            poissonJob.stdout.on('data', (data) => {
                console.log(`poisson-surface stdout:  ${data}`)
            })
        
            poissonJob.stderr.on('data', (data) => {
                console.log(`poisson-surface stderr:  ${data}`)
            })
        
            poissonJob.on('close', (exitCode) => {
                console.log(`poisson-surface closed with exit code ${exitCode}`)
                if (exitCode != 0) {
                    reject(`poisson-surface exited with error code: ${exitCode}`)
                } else {
                    console.log('Successfully converted to surface!')
                    resolve(outputPath)
                }
            })
        })
    }

    function blenderUVMap(modelPath) {
        return new Promise((resolve, reject) => {
            // /Applications/Blender/blender.app/Contents/MacOS/blender --background --python ./uv_mapping_gen.py ./models/detergent-surface.ply ./models/detergent-texture ./models/detergent-mapped.obj
            let outputPaths = {
                texture: `./models/${uuid}-texture`, 
                obj: `./models/${uuid}-mapped.obj`
            }

            const blenderJob = spawn(config["blender-location"], [
                '--background',
                '--python',
                './uv-mapping-gen.py',
                modelPath,
                outputPaths[0],
                outputPaths[1]
            ])

            blenderJob.stdout.on('data', (data) => {
                console.log(`blender stdout:  ${data}`)
            })
        
            blenderJob.stderr.on('data', (data) => {
                console.log(`blender stderr:  ${data}`)
            })
        
            blenderJob.on('close', (exitCode) => {
                console.log(`blender closed with exit code ${exitCode}`)
                if (exitCode != 0) {
                    reject(`blender exited with error code: ${exitCode}`)
                } else {
                    console.log('Successfully added UV Map!')
                    resolve(outputPaths)
                }
            })
        })
    }

    function generateTextureAtlas(modelPath) {
        return new Promise((resolve, reject) => {
            const textureAtlasJob = spawn("python3", [
                "./renderer/render-model.py",
                modelPaths.texture,
                modelPaths.obj,
                "./colmap-workspace/images.txt",
                "./frames/",
                `./models/${uuid}-texture.png`
            ])

            textureAtlasJob.stdout.on('data', (data) => {
                console.log(`python stdout:  ${data}`)
            })
        
            textureAtlasJob.stderr.on('data', (data) => {
                console.log(`python stderr:  ${data}`)
            })
        
            textureAtlasJob.on('close', (exitCode) => {
                console.log(`python closed with exit code ${exitCode}`)
                if (exitCode != 0) {
                    reject(`python exited with error code: ${exitCode}`)
                } else {
                    console.log('Successfully generated texture atlas!')
                    resolve()
                }
            })
        })
    }
}
