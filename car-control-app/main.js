const { app, BrowserWindow, ipcMain } = require('electron')
const path = require('path')
const net = require('net')

let mainWindow
let tcpClient

function createWindow() {
  mainWindow = new BrowserWindow({
    width: 800,
    height: 600,
    webPreferences: {
      nodeIntegration: true,
      contextIsolation: false
    }
  })

  mainWindow.loadFile('index.html')
  
  // Open DevTools for debugging
  mainWindow.webContents.openDevTools()
}

// Handle TCP connection request from renderer
ipcMain.on('connect-to-server', (event, data) => {
  if (tcpClient) {
    tcpClient.destroy()
  }

  tcpClient = new net.Socket()

  tcpClient.connect(data.port, data.ipAddress, () => {
    console.log(`Connected to ${data.ipAddress}:${data.port}`)
    mainWindow.webContents.send('connection-status', {
      connected: true,
      message: `Connected to ${data.ipAddress}:${data.port}`
    })
  })

  tcpClient.on('data', (data) => {
    console.log('Raw data received:', data.toString());
    
    const messages = data.toString().split('\n');
    
    messages.forEach(message => {
      if (!message.trim()) return;
      
      try {
        const jsonData = JSON.parse(message.trim());
        console.log('Parsed JSON:', jsonData);
        
        if (jsonData.type === 'metrics') {
          mainWindow.webContents.send('metrics-update', jsonData.data);
        }
      } catch (e) {
        console.log('Parse error:', e.message);
        // Not JSON data, ignore
      }
    });
  })

  tcpClient.on('error', (err) => {
    console.error('Connection error:', err)
    mainWindow.webContents.send('connection-status', {
      connected: false,
      message: `Connection error: ${err.message}`
    })
  })

  tcpClient.on('close', () => {
    console.log('Connection closed')
    mainWindow.webContents.send('connection-status', {
      connected: false,
      message: 'Connection closed'
    })
  })
})

// Handle disconnect request
ipcMain.on('disconnect', () => {
  if (tcpClient) {
    tcpClient.destroy()
    tcpClient = null
  }
})

// Handle control commands from renderer
ipcMain.on('control-command', (event, command) => {
  if (tcpClient && tcpClient.writable) {
    console.log('Sending command:', command)
    // Send simple command strings
    tcpClient.write(command + '\n')
  } else {
    console.log('Cannot send command - not connected')
    mainWindow.webContents.send('connection-status', {
      connected: false,
      message: 'Connection lost - please reconnect'
    })
  }
})

app.whenReady().then(createWindow)

app.on('window-all-closed', () => {
  if (tcpClient) {
    tcpClient.destroy()
  }
  if (process.platform !== 'darwin') {
    app.quit()
  }
})

app.on('activate', () => {
  if (BrowserWindow.getAllWindows().length === 0) {
    createWindow()
  }
}) 