const PORT = 3000

const express = require('express')
const reconstruct = require('./reconstruct')
const fileUpload = require('express-fileupload')
const uuid = require('uuid/v4')
const server = express()

server.use(fileUpload())
server.use(express.static('static'))

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

        reconstruct(newVideoName, ()=>{})
        res.send(`Model is now processing... <br> <a href='/view-model?id=${id}'>It will be available from here.</a>`)
    })
})


server.listen(PORT, () => {
    console.log(`Server has started on ${PORT}`)
})
