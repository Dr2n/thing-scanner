module.exports = function (videoName, err) {
    const { spawn } = require('child_process')
    
    let uuid = videoName.split('.')[0]
    let extension = videoName.split('.')[1]

    console.log(`Processing video ${uuid}...`)

    // use ffmpeg to split the video into frame images
    // ffmpeg -i test-video.avi -vf "select=not(mod(n\,5))" -vsync vfr -q:v 2 img_%03d.jpg
    
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
    })

    // use colmap to do sparse reconstruction, output .ply file

    // call poisson reconstruction program to create surface

    // put created model into a folder



}