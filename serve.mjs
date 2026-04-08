import https from 'https';
import { readFileSync } from 'fs';
import { join, extname } from 'path';
import { fileURLToPath } from 'url';
import { readFile } from 'fs/promises';

const __dirname = fileURLToPath(new URL('.', import.meta.url));
const PORT = 8443;

const MIME = {
  '.html': 'text/html',
  '.js':   'application/javascript',
  '.mjs':  'application/javascript',
  '.css':  'text/css',
  '.json': 'application/json',
  '.wasm': 'application/wasm',
  '.obj':  'text/plain',
  '.png':  'image/png',
  '.jpg':  'image/jpeg',
  '.svg':  'image/svg+xml',
};

const server = https.createServer({
  key:  readFileSync(join(__dirname, '.key.pem')),
  cert: readFileSync(join(__dirname, '.cert.pem')),
}, async (req, res) => {
  let url = req.url.split('?')[0];
  if (url === '/') url = '/index.html';

  // Allow serving from parent dir (for node_modules symlinks)
  const filePath = join(__dirname, url);

  try {
    const data = await readFile(filePath);
    const ext = extname(filePath);
    res.writeHead(200, {
      'Content-Type': MIME[ext] || 'application/octet-stream',
      'Cross-Origin-Opener-Policy': 'same-origin',
      'Cross-Origin-Embedder-Policy': 'require-corp',
    });
    res.end(data);
  } catch {
    res.writeHead(404);
    res.end('Not found: ' + url);
  }
});

server.listen(PORT, '0.0.0.0', () => {
  console.log(`\n  HTTPS server running at:`);
  console.log(`    https://localhost:${PORT}`);
  console.log(`    https://10.0.0.226:${PORT}  (local network)\n`);
  console.log(`  On iOS: open the URL, tap "Advanced" > "Proceed" to accept the self-signed cert.\n`);
});
