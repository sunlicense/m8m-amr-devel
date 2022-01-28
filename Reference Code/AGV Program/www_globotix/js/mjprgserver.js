        setCamera: function() {
            console.log('set camera method')
            this.cameraViewer = new MJPEGCANVAS.Viewer({
                divID: 'mjpeg',
                host: '54.167.21.209',
                width: 640,
                height: 480,
                topic: '/camera/rgb/image_raw',
                port: 11315,
            })
        },
