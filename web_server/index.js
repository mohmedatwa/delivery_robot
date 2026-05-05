import express from 'express';
import { WebSocketServer, WebSocket } from 'ws';
import path from 'path';
import { fileURLToPath } from 'url';

const __dirname = path.dirname(fileURLToPath(import.meta.url));
const app = express();
const HTTP_PORT = 8080;

app.use(express.json({ limit: '50mb' }));
app.get('/', (req, res) => res.sendFile(path.join(__dirname, 'index.html')));

const wss = new WebSocketServer({ noServer: true });
let cmdQueue = { linear: 0, angular: 0 };

// استقبال الأوامر من المتصفح عبر WebSocket
wss.on('connection', (ws) => {
    ws.on('message', (msg) => {
        const data = JSON.parse(msg);
        if (data.type === 'TELEOP') {
            cmdQueue = { linear: data.linear, angular: data.angular };
        }
    });
});

// استقبال بيانات الروبوت وإرسالها للمتصفح
app.post('/', (req, res) => {
    const robotMsg = JSON.stringify(req.body);
    wss.clients.forEach(client => {
        if (client.readyState === WebSocket.OPEN) client.send(robotMsg);
    });

    // الرد على الروبوت بأحدث أمر تحكم
    res.json(cmdQueue);
});

const server = app.listen(HTTP_PORT, () => {
    console.log(`✅ Dashboard: http://localhost:${HTTP_PORT}`);
});
 
server.on('upgrade', (req, socket, head) => {
    wss.handleUpgrade(req, socket, head, (ws) => wss.emit('connection', ws, req));
});