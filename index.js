const PORT = 3000

const express = require('express')
const reconstruct = require('./reconstruct')
const clearFolders = require('./clearFolders')
const fileUpload = require('express-fileupload')
const uuid = require('uuid/v4')
const https = require('https')
const fs = require('fs')
const server = express()

server.use(fileUpload())
server.use(express.static('static'))
server.use('/models', express.static('models'))

server.post('/upload', (req, res) => {
    if (Object.keys(req.files).length == 0) {
        return res.status(400).send('No files were uploaded')
    }

    console.log(req.files.video)
    let video = req.files.video
    let extension = video.name.split('.').slice(-1)[0]
    let id = uuid()
    let newVideoName = `${id}.${extension}`
    video.mv(`./videos/${newVideoName}`, (err) => {
        if (err) {
            return res.status(500).send(err)
        }

        reconstruct(newVideoName).then((outputPath) => {
            console.log("Reconstruction complete!")
            console.log(`See /disp/?model=${outputPath}`)
        })
        res.send(`Model is now processing... <br> <a href='/view-model?id=${id}'>It will be available from here.</a>`)
    })
})

clearFolders().then(() => {
    https.createServer({
        key: fs.readFileSync('server.key'),
        cert: fs.readFileSync('server.cert')
      }, server)
      .listen(PORT, () => {
        console.log('Example app listening on port 3000! Go to https://localhost:3000/')
      })
})
