const express = require('express');
const http = require('http');
const WebSocket = require('ws');

const app = express();
const server = http.createServer(app);
const wss = new WebSocket.Server({ server });

app.use(express.json()); // To parse JSON request bodies

// Serve static files from the current directory (e.g., index.html)
app.use(express.static(__dirname));

let latestCommand = { linear: 0.0, angular: 0.0 }; // Store the latest command from web client

// HTTP POST endpoint for the ROS 2 node
app.post('/', (req, res) => {
    const rosData = req.body;
    // console.log('Received data from ROS node:', Object.keys(rosData)); // Uncomment for debugging

    // Broadcast ROS data to all connected WebSocket clients
    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) {
            client.send(JSON.stringify(rosData));
        }
    });

    // Send the latest command received from a web client back to the ROS node
    res.json(latestCommand);
    // Note: We don't reset latestCommand here. The web client is responsible for sending
    // zero commands when the joystick/keys are released, ensuring the robot stops.
});

// WebSocket endpoint for the web client (index.html)
wss.on('connection', ws => {
    console.log('Web client connected');

    ws.on('message', message => {
        try {
            const command = JSON.parse(message);
            if (command.type === 'TELEOP' && typeof command.linear === 'number' && typeof command.angular === 'number') {
                latestCommand = { linear: command.linear, angular: command.angular };
                // console.log('Received teleop command from web client:', latestCommand); // Uncomment for debugging
            }
        } catch (error) {
            console.error('Failed to parse WebSocket message:', error);
        }
    });

    ws.on('close', () => {
        console.log('Web client disconnected');
    });

    ws.on('error', error => {
        console.error('WebSocket error:', error);
    });
});

const PORT = 8080;
server.listen(PORT, () => {
    console.log(`Web server listening on http://localhost:${PORT}`);
    console.log(`WebSocket server listening on ws://localhost:${PORT}`);
});