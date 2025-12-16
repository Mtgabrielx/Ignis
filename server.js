const amqp = require("amqplib");
const WebSocket = require("ws");

const RABBIT = {
  host: "",
  port: 5672,
  username: "",
  password: "",
  heartbeat: 60,
  timeoutMs: 300000,
};

const QUEUES = ["temperatura", "umidade", "fumaca", "alertaTemp", "alertaFuma"];

// WebSocket server
const wss = new WebSocket.Server({ port: 8080 });

// Mapa: ws -> Set(queues) que o cliente assinou (opcional)
const subscriptions = new Map();

/* =========================
   WebSocket: conexões e subscribe opcional
   ========================= */
wss.on("connection", (ws) => {
  subscriptions.set(ws, new Set()); // vazio = não assinou nada

  ws.on("message", (raw) => {
    // Espera: {"action":"subscribe","queue":"temperatura"} ou {"action":"unsubscribe","queue":"temperatura"}
    try {
      const msg = JSON.parse(raw.toString("utf8"));
      if (!msg || typeof msg !== "object") return;

      const set = subscriptions.get(ws);
      if (!set) return;

      if (msg.action === "subscribe" && QUEUES.includes(msg.queue)) {
        set.add(msg.queue);
        ws.send(JSON.stringify({ type: "subscribed", queue: msg.queue }));
      } else if (msg.action === "unsubscribe" && QUEUES.includes(msg.queue)) {
        set.delete(msg.queue);
        ws.send(JSON.stringify({ type: "unsubscribed", queue: msg.queue }));
      }
    } catch {
      // Se não for JSON, ignora silenciosamente
    }
  });

  ws.on("close", () => {
    subscriptions.delete(ws);
  });
});

/* =========================
   Função de envio WS
   ========================= */
function sendToClients(queue, payload) {
  const message = JSON.stringify({
    queue,
    ts: Date.now(),
    data: payload,
  });

  for (const client of wss.clients) {
    if (client.readyState !== WebSocket.OPEN) continue;

    // Modo 1: broadcast para todos
    // client.send(message);

    // Modo 2: somente para quem assinou (se ninguém assinou nada, você pode optar por enviar ou não)
    const set = subscriptions.get(client);
    const hasSubscriptions = set && set.size > 0;

    // Regra sugerida:
    // - Se o cliente assinou algo: envia só se ele assinou este queue
    // - Se não assinou nada: faz broadcast (assim funciona “out of the box”)
    const shouldSend = hasSubscriptions ? set.has(queue) : true;

    if (shouldSend) client.send(message);
  }
}

/* =========================
   Consumidor RabbitMQ
   ========================= */
async function startRabbitConsumers() {
  const url = `amqp://${encodeURIComponent(RABBIT.username)}:${encodeURIComponent(
    RABBIT.password
  )}@${RABBIT.host}:${RABBIT.port}`;

  const conn = await amqp.connect(url, {
    heartbeat: RABBIT.heartbeat,
    timeout: RABBIT.timeoutMs,
  });

  conn.on("error", (err) => console.error("RabbitMQ connection error:", err.message));
  conn.on("close", () => console.error("RabbitMQ connection closed"));

  const channel = await conn.createChannel();

  // Garante filas duráveis
  for (const q of QUEUES) {
    await channel.assertQueue(q, { durable: true });
  }

  // Consome cada fila
  for (const q of QUEUES) {
    await channel.consume(
      q,
      (msg) => {
        if (!msg) return;

        let payload;
        const raw = msg.content.toString("utf8");

        // Tenta JSON; se não for, envia como string
        try {
          payload = JSON.parse(raw);
        } catch {
          payload = raw;
        }

        sendToClients(q, payload);

        // auto_ack=True no Python => noAck: true aqui (não precisa ack)
        // Se você quiser ACK manual (mais confiável), mude noAck:false e use channel.ack(msg).
      },
      { noAck: true }
    );

    console.log(`Consumindo fila: ${q}`);
  }

  // Encerramento gracioso
  const shutdown = async () => {
    console.log("Encerrando (SIGINT/SIGTERM)...");
    try {
      await channel.close();
    } catch {}
    try {
      await conn.close();
    } catch {}
    process.exit(0);
  };

  process.on("SIGINT", shutdown);
  process.on("SIGTERM", shutdown);
}

// Start
startRabbitConsumers().catch((e) => {
  console.error("Falha ao iniciar consumidores:", e);
  process.exit(1);
});

