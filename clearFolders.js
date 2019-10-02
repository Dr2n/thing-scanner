// start process to clear folders
module.exports = function() {
    const { spawn } = require('child_process')

    return new Promise((resolve, reject) => {
        Promise.all([
            deleteFolder('frames'),
            deleteFolder('videos')
        ]).then(() => {
            resolve()
        }).catch((error) => {
            reject(error)
        })
    })


    function deleteFolder(dirPath) {
        return new Promise((resolve, reject) => {
            const deleteJob = spawn('rm', [
                '-rf', `./${dirPath}/*`
            ])
            
            console.log(`Deleting ${dirPath}`)
        
            deleteJob.stdout.on('data', (data) => {
                console.log(data)
            })
        
            deleteJob.stderr.on('data', (data) => {
                console.log(`stderr: ${data}`)
            })
        
            deleteJob.on('close', (exitCode) => {
                console.log(`rm -r closed with exit code ${exitCode}`)
                if (exitCode != 0) {
                    reject(`rm -r exited with error code: ${exitCode}`)
                } else {
                    console.log('Deleted successfully.')
                    resolve()
                }
            })
        })
    }
}