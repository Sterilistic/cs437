const { ipcRenderer } = require('electron')

const connectBtn = document.getElementById('connectBtn')
const ipAddressInput = document.getElementById('ipAddress')
const portInput = document.getElementById('port')
const statusDiv = document.getElementById('connectionStatus')

// Control buttons
const forwardBtn = document.getElementById('forward')
const backwardBtn = document.getElementById('backward')
const leftBtn = document.getElementById('left')
const rightBtn = document.getElementById('right')
const stopBtn = document.getElementById('stop')

let connected = false

function sendControlCommand(command) {
    if (connected) {
        const buffer = Buffer.alloc(1)
        switch(command) {
            case 'forward':
                buffer[0] = 1
                break
            case 'backward':
                buffer[0] = 2
                break
            case 'left':
                buffer[0] = 3
                break
            case 'right':
                buffer[0] = 4
                break
            case 'stop':
                buffer[0] = 0
                break
        }
        ipcRenderer.send('control-command', buffer)
        console.log('Sending command:', command)
    } else {
        console.log('Not connected - command not sent')
    }
}

function updateSensorData(data) {
    try {
        // Convert data to Buffer if it isn't already
        const buffer = Buffer.isBuffer(data) ? data : Buffer.from(data)
        console.log('Raw buffer data:', buffer)

        // Make sure we have enough data
        if (buffer.length >= 4) {
            const battery = buffer[0]
            const direction = buffer[1]
            const distance = (buffer[2] << 8) | buffer[3] // Combine two bytes for distance

            document.getElementById('battery').textContent = `Battery: ${battery}%`
            document.getElementById('direction').textContent = 
                `Direction: ${['Stopped', 'Forward', 'Backward', 'Left', 'Right'][direction] || 'Unknown'}`
            document.getElementById('obstacle').textContent = `Front Distance: ${distance}cm`
            
            console.log('Updated sensor data:', { battery, direction, distance })
        } else {
            console.warn('Received incomplete data packet:', buffer)
        }
    } catch (e) {
        console.error('Error parsing sensor data:', e, 'Data:', data)
    }
}

connectBtn.addEventListener('click', () => {
    if (!connected) {
        const ipAddress = ipAddressInput.value
        const port = parseInt(portInput.value)

        if (!ipAddress) {
            alert('Please enter an IP address')
            return
        }

        ipcRenderer.send('connect-to-server', { ipAddress, port })
    } else {
        // If already connected, disconnect
        ipcRenderer.send('disconnect')
    }
})

// Listen for connection status updates
ipcRenderer.on('connection-status', (event, status) => {
    connected = status.connected
    statusDiv.textContent = status.message
    statusDiv.className = `status ${status.connected ? 'connected' : 'disconnected'}`
    connectBtn.textContent = status.connected ? 'Disconnect' : 'Connect'
    console.log('Connection status:', status)
})

// Listen for sensor data from the server
ipcRenderer.on('server-data', (event, data) => {
    updateSensorData(data)
})

// Control button event listeners
forwardBtn.addEventListener('click', () => {
    console.log('Forward button clicked')
    sendControlCommand('forward')
})
backwardBtn.addEventListener('click', () => {
    console.log('Backward button clicked')
    sendControlCommand('backward')
})
leftBtn.addEventListener('click', () => {
    console.log('Left button clicked')
    sendControlCommand('left')
})
rightBtn.addEventListener('click', () => {
    console.log('Right button clicked')
    sendControlCommand('right')
})
stopBtn.addEventListener('click', () => {
    console.log('Stop button clicked')
    sendControlCommand('stop')
})

// Keyboard controls
document.addEventListener('keydown', (event) => {
    switch(event.key) {
        case 'ArrowUp':
            console.log('Up arrow pressed')
            sendControlCommand('forward')
            break
        case 'ArrowDown':
            console.log('Down arrow pressed')
            sendControlCommand('backward')
            break
        case 'ArrowLeft':
            console.log('Left arrow pressed')
            sendControlCommand('left')
            break
        case 'ArrowRight':
            console.log('Right arrow pressed')
            sendControlCommand('right')
            break
        case ' ': // Space bar
            console.log('Space pressed')
            sendControlCommand('stop')
            break
    }
})

// Clean up on window close
window.addEventListener('beforeunload', () => {
    ipcRenderer.send('disconnect')
}) 