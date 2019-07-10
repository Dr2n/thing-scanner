
module.exports = function (videoName, err) {
    const config = require('./config.js')
    const { spawn } = require('child_process')
    
    let uuid = videoName.split('.')[0]
    let extension = videoName.split('.')[1]

    console.log(`Processing video ${uuid}...`)

    // use ffmpeg to split the video into frame images
    // ffmpeg -i test-video.avi -vf "select=not(mod(n\,5))" -vsync vfr -q:v 2 img_%03d.jpg
    ffmpegJob(videoName)
    // use colmap to do sparse reconstruction, output .ply file
    .then(sparseReconstruction)
    // use colmap to export as .PLY
    .then(exportModel)
    // call poisson reconstruction program to create surface
    .then(poissonSurface)
    // put created model into a folder
    .catch(err => {
        console.log(err)
    })
    
    // Functions
    function ffmpegJob(videoName) {
        return new Promise((resolve, reject) => {
            const ffmpegJob = spawn('ffmpeg', [
                '-i', `./videos/${videoName}`, '-vf', 'select=not(mod(n\\,5))',
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

            colmapJob.stdout.on('data', (data) => {
                console.log(`colmap stdout:  ${data}`)
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

    function sparseReconstruction() {
        return new Promise((resolve, reject) => {
            const colmapJob = spawn(config["colmap-location"], [
                'automatic_reconstructor', 
                '--workspace_path', './colmap-workspace',
                '--image_path', './frames'
            ])

            colmapJob.stdout.on('data', (data) => {
                console.log(`colmap stdout:  ${data}`)
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
            const poissonJob = spawn(config["poisson-location"], [
                `models/${uuid}.ply`,
                `models/${uuid}-surface.ply`
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
                    resolve()
                }
            })
        })
    }
}
